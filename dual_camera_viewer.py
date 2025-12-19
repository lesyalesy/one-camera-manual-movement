#!/usr/bin/env python3
"""
Dual Camera Viewer
Shows both cameras together with optimized HSV settings and detected lines
"""

import cv2
import numpy as np
import time
from collections import deque

class DualCameraViewer:
    def __init__(self):
        self.side_cap = None
        self.front_cap = None
        
        # Side camera optimized HSV ranges
        self.side_colors = {
            'yellow': {
                'lower': [16, 72, 145], 
                'upper': [35, 255, 255], 
                'name': 'Yellow Needle'
            },
            'green': {
                'lower': [40, 67, 42], 
                'upper': [95, 142, 248], 
                'name': 'Green Hair'
            },
            'blue': {
                'lower': [68, 115, 39], 
                'upper': [156, 221, 255], 
                'name': 'Blue Hair'
            }
        }
        
        # Front camera optimized HSV ranges
        self.front_colors = {
            'green': {
                'lower': [53, 66, 83], 
                'upper': [100, 255, 255], 
                'name': 'Green Hair'
            },
            'blue': {
                'lower': [113, 70, 132], 
                'upper': [142, 236, 244], 
                'name': 'Blue Hair'
            },
            'yellow': {
                'lower': [2, 32, 128], 
                'upper': [39, 255, 255], 
                'name': 'Yellow Needle'
            }
        }
        
        # Line detection parameters
        self.min_area = 50
        self.angle_threshold = 30  # Increased to 30 for maximum stability
        
        # Stable angle tracking
        self.side_stable_angles = {'green': None, 'blue': None, 'yellow': None}
        self.front_stable_angles = {'green': None, 'blue': None, 'yellow': None}
        
        # Angle history for smoothing
        self.side_angle_history = {'green': deque(maxlen=20), 'blue': deque(maxlen=20), 'yellow': deque(maxlen=20)}
        self.front_angle_history = {'green': deque(maxlen=20), 'blue': deque(maxlen=20), 'yellow': deque(maxlen=20)}
        
    def initialize_cameras(self):
        """Initialize both cameras"""
        # Initialize side camera (Camera 1)
        self.side_cap = cv2.VideoCapture(1)
        if not self.side_cap.isOpened():
            print("❌ Could not open side camera (Camera 1)")
            return False
        
        # Initialize front camera (Camera 0)
        self.front_cap = cv2.VideoCapture(0)
        if not self.front_cap.isOpened():
            print("❌ Could not open front camera (Camera 0)")
            return False
        
        # Set camera properties
        for cap in [self.side_cap, self.front_cap]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            cap.set(cv2.CAP_PROP_FPS, 30)
        
        print("✅ Both cameras initialized")
        return True
    
    def process_frame(self, frame):
        """Process frame with 2x zoom and 400x400"""
        if frame is None:
            return None
        
        height, width = frame.shape[:2]
        
        # Calculate crop size for 2x zoom
        crop_size = int(min(width, height) / 2.0)
        
        # Calculate crop coordinates (center crop)
        center_x = width // 2
        center_y = height // 2
        
        x1 = center_x - crop_size // 2
        y1 = center_y - crop_size // 2
        x2 = x1 + crop_size
        y2 = y1 + crop_size
        
        # Ensure crop coordinates are within bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(width, x2)
        y2 = min(height, y2)
        
        # Crop the frame
        cropped = frame[y1:y2, x1:x2]
        
        # Resize to 400x400 square frame
        square_frame = cv2.resize(cropped, (400, 400))
        
        return square_frame
    
    def stabilize_angle(self, color_name, new_angle, camera_type):
        """Stabilize angle to prevent shaking"""
        if camera_type == 'side':
            stable_angles = self.side_stable_angles
            angle_history = self.side_angle_history
        else:
            stable_angles = self.front_stable_angles
            angle_history = self.front_angle_history
        
        if new_angle is None:
            return stable_angles[color_name]
        
        # If no stable angle yet, use the new angle
        if stable_angles[color_name] is None:
            stable_angles[color_name] = new_angle
            angle_history[color_name].append(new_angle)
            return new_angle
        
        # Check if angle change is significant
        angle_diff = abs(new_angle - stable_angles[color_name])
        
        # If angle change is small, keep the stable angle
        if angle_diff < self.angle_threshold:
            return stable_angles[color_name]
        
        # If angle change is significant, update stable angle
        stable_angles[color_name] = new_angle
        angle_history[color_name].append(new_angle)
        
        # Smooth the angle using history with median filtering for maximum stability
        if len(angle_history[color_name]) > 3:
            # Use median filtering for maximum stability
            angles = list(angle_history[color_name])
            smoothed_angle = np.median(angles)  # Median is more stable than average
            stable_angles[color_name] = smoothed_angle
        elif len(angle_history[color_name]) > 1:
            # Fallback to weighted average for smaller buffers
            angles = list(angle_history[color_name])
            weights = np.linspace(0.3, 1.0, len(angles))  # Even more weight on recent angles
            smoothed_angle = np.average(angles, weights=weights)
            stable_angles[color_name] = smoothed_angle
        
        return stable_angles[color_name]
    
    def detect_stable_lines(self, frame, color_name, camera_type):
        """Detect lines with stable angles"""
        if frame is None:
            return []
        
        # Get colors based on camera type
        colors = self.side_colors if camera_type == 'side' else self.front_colors
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get color range
        color_info = colors[color_name]
        lower = np.array(color_info['lower'])
        upper = np.array(color_info['upper'])
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphological operations to clean up mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        lines = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                # Fit line to contour
                [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                
                # Calculate raw angle
                raw_angle = np.arctan2(vy[0], vx[0]) * 180 / np.pi
                if raw_angle < 0:
                    raw_angle += 180
                
                # Stabilize the angle
                stable_angle = self.stabilize_angle(color_name, raw_angle, camera_type)
                
                # Calculate line endpoints using stable angle
                line_length = 50
                x1 = int(x[0] - np.cos(np.radians(stable_angle)) * line_length)
                y1 = int(y[0] - np.sin(np.radians(stable_angle)) * line_length)
                x2 = int(x[0] + np.cos(np.radians(stable_angle)) * line_length)
                y2 = int(y[0] + np.sin(np.radians(stable_angle)) * line_length)
                
                lines.append({
                    'start': (x1, y1),
                    'end': (x2, y2),
                    'center': (int(x[0]), int(y[0])),
                    'angle': stable_angle,
                    'raw_angle': raw_angle,
                    'area': area,
                    'color': color_name
                })
        
        return lines
    
    def draw_stable_lines(self, frame, lines, color_name):
        """Draw stable lines"""
        result = frame.copy()
        
        # Get color for drawing
        color_map = {
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255)
        }
        
        color = color_map[color_name]
        
        for line in lines:
            # Draw stable line
            cv2.line(result, line['start'], line['end'], color, 2)
            
            # Draw center point
            cv2.circle(result, line['center'], 3, color, -1)
            
            # Draw angle line
            angle_line_length = 20
            end_x = int(line['center'][0] + angle_line_length * np.cos(np.radians(line['angle'])))
            end_y = int(line['center'][1] + angle_line_length * np.sin(np.radians(line['angle'])))
            cv2.line(result, line['center'], (end_x, end_y), color, 2)
            
            # Draw label with stable angle
            label = f"{color_name} Line: {line['angle']:.1f} deg"
            cv2.putText(result, label, (line['center'][0] - 50, line['center'][1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return result
    
    def display_dual_cameras(self, duration=120):
        """Display both cameras together with detected lines"""
        if not self.side_cap or not self.front_cap:
            print("❌ Cameras not initialized")
            return
        
        print("🎨 Dual Camera Viewer")
        print("=" * 60)
        print("🎯 Features:")
        print("   🔍 Both cameras with optimized HSV settings")
        print("   📐 Side camera (left) and Front camera (right)")
        print("   🔍 Stable angles (no shaking)")
        print("   📐 2x zoom, 400x400 frames")
        print("Press 'q' to quit, 's' to save frames")
        
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < duration:
            # Get frames from both cameras
            side_ret, side_frame = self.side_cap.read()
            front_ret, front_frame = self.front_cap.read()
            
            if not side_ret or not front_ret:
                print("❌ Failed to read from cameras")
                break
            
            # Process frames
            side_processed = self.process_frame(side_frame)
            front_processed = self.process_frame(front_frame)
            
            if side_processed is not None and front_processed is not None:
                # Detect stable lines for each camera
                side_all_lines = []
                side_result = side_processed.copy()
                
                for color_name in self.side_colors.keys():
                    lines = self.detect_stable_lines(side_processed, color_name, 'side')
                    side_all_lines.extend(lines)
                    side_result = self.draw_stable_lines(side_result, lines, color_name)
                
                front_all_lines = []
                front_result = front_processed.copy()
                
                for color_name in self.front_colors.keys():
                    lines = self.detect_stable_lines(front_processed, color_name, 'front')
                    front_all_lines.extend(lines)
                    front_result = self.draw_stable_lines(front_result, lines, color_name)
                
                # Add labels to side camera
                cv2.putText(side_result, "Side Camera", (10, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(side_result, f"Lines: {len(side_all_lines)}", (10, 45), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Add labels to front camera
                cv2.putText(front_result, "Front Camera", (10, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(front_result, f"Lines: {len(front_all_lines)}", (10, 45), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Combine frames side by side
                combined = np.hstack((side_result, front_result))
                
                # Add overall labels
                cv2.putText(combined, f"Frame: {frame_count}", (10, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.putText(combined, f"Total Lines: {len(side_all_lines) + len(front_all_lines)}", (10, 45), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # Display combined frame
                cv2.imshow('Dual Camera Viewer - Side (Left) & Front (Right)', combined)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("👋 Quit requested")
                    break
                elif key == ord('s'):
                    cv2.imwrite(f"dual_camera_{frame_count}.jpg", combined)
                    print(f"💾 Saved frame {frame_count}")
            
            frame_count += 1
        
        print(f"✅ Dual camera viewing completed: {frame_count} frames processed")
    
    def release_cameras(self):
        """Release camera resources"""
        if self.side_cap:
            self.side_cap.release()
            print("🧹 Side camera released")
        if self.front_cap:
            self.front_cap.release()
            print("🧹 Front camera released")
        cv2.destroyAllWindows()

def main():
    """Main function"""
    print("🎨 Dual Camera Viewer")
    print("=" * 70)
    print("🎯 Features:")
    print("   🔍 Both cameras with optimized HSV settings")
    print("   📐 Side camera (left) and Front camera (right)")
    print("   🔍 Stable angles (no shaking)")
    print("   📐 2x zoom, 400x400 frames")
    print("=" * 70)
    
    # Initialize viewer
    viewer = DualCameraViewer()
    
    if viewer.initialize_cameras():
        print("✅ Both cameras initialized with optimized settings")
        print("\n📋 Instructions:")
        print("1. Side camera (left) and Front camera (right)")
        print("2. Press 's' to save test images")
        print("3. Press 'q' to quit")
        
        viewer.display_dual_cameras(duration=120)  # 2 minutes
        viewer.release_cameras()
    else:
        print("❌ Failed to initialize cameras")
    
    print("✅ Dual camera viewing completed!")

if __name__ == "__main__":
    main()
