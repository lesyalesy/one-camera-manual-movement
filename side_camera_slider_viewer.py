#!/usr/bin/env python3
"""
Side Camera Slider Viewer
HSV sliders for side camera with data recording for adjustments.
"""

import cv2
import numpy as np
import time
import json
from collections import deque

class SideCameraSliderViewer:
    def __init__(self, camera_id: int = 1):
        self.camera_id = camera_id
        self.cap = None
        
        # Optimized HSV ranges from your data
        self.colors = {
            'green': {
                'lower': [41, 99, 113], 
                'upper': [96, 245, 255], 
                'name': 'Green Hair'
            },
            'blue': {
                'lower': [32, 96, 148], 
                'upper': [147, 221, 255], 
                'name': 'Blue Hair'
            },
            'yellow': {
                'lower': [15, 59, 144], 
                'upper': [89, 255, 255], 
                'name': 'Yellow Needle'
            }
        }
        
        # Line detection parameters
        self.min_area = 50
        self.angle_threshold = 5
        
        # Stable angle tracking
        self.stable_angles = {'green': None, 'blue': None, 'yellow': None}
        
        # Angle history for smoothing
        self.angle_history = {'green': deque(maxlen=5), 'blue': deque(maxlen=5), 'yellow': deque(maxlen=5)}
        
        # Data recording for adjustments
        self.adjustment_data = []
        self.start_time = time.time()
        
    def initialize_camera(self):
        """Initialize side camera"""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"❌ Could not open camera {self.camera_id}")
            return False
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        print(f"✅ Side Camera {self.camera_id} initialized")
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
    
    def create_hsv_sliders(self):
        """Create HSV adjustment sliders for side camera - yellow separate"""
        # Yellow sliders in separate window
        cv2.namedWindow('Yellow Controls - Top', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Yellow Controls - Top', 1000, 200)
        cv2.moveWindow('Yellow Controls - Top', 100, 100)
        
        # Create yellow trackbars
        cv2.createTrackbar('Y_H_min', 'Yellow Controls - Top', 
                          self.colors['yellow']['lower'][0], 179, lambda x: None)
        cv2.createTrackbar('Y_S_min', 'Yellow Controls - Top', 
                          self.colors['yellow']['lower'][1], 255, lambda x: None)
        cv2.createTrackbar('Y_V_min', 'Yellow Controls - Top', 
                          self.colors['yellow']['lower'][2], 255, lambda x: None)
        cv2.createTrackbar('Y_H_max', 'Yellow Controls - Top', 
                          self.colors['yellow']['upper'][0], 179, lambda x: None)
        cv2.createTrackbar('Y_S_max', 'Yellow Controls - Top', 
                          self.colors['yellow']['upper'][1], 255, lambda x: None)
        cv2.createTrackbar('Y_V_max', 'Yellow Controls - Top', 
                          self.colors['yellow']['upper'][2], 255, lambda x: None)
        
        # Green and blue sliders in separate window
        cv2.namedWindow('Green/Blue Controls - Bottom', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Green/Blue Controls - Bottom', 1000, 300)
        cv2.moveWindow('Green/Blue Controls - Bottom', 100, 350)
        
        # Create green and blue trackbars
        for color_name in ['green', 'blue']:
            color_short = color_name[0].upper()  # G, B
            cv2.createTrackbar(f'{color_short}_H_min', 'Green/Blue Controls - Bottom', 
                             self.colors[color_name]['lower'][0], 179, lambda x: None)
            cv2.createTrackbar(f'{color_short}_S_min', 'Green/Blue Controls - Bottom', 
                             self.colors[color_name]['lower'][1], 255, lambda x: None)
            cv2.createTrackbar(f'{color_short}_V_min', 'Green/Blue Controls - Bottom', 
                             self.colors[color_name]['lower'][2], 255, lambda x: None)
            cv2.createTrackbar(f'{color_short}_H_max', 'Green/Blue Controls - Bottom', 
                             self.colors[color_name]['upper'][0], 179, lambda x: None)
            cv2.createTrackbar(f'{color_short}_S_max', 'Green/Blue Controls - Bottom', 
                             self.colors[color_name]['upper'][1], 255, lambda x: None)
            cv2.createTrackbar(f'{color_short}_V_max', 'Green/Blue Controls - Bottom', 
                             self.colors[color_name]['upper'][2], 255, lambda x: None)
    
    def update_hsv_ranges(self):
        """Update HSV ranges from sliders and record adjustments"""
        current_time = time.time() - self.start_time
        
        # Update yellow from top window
        h_min = cv2.getTrackbarPos('Y_H_min', 'Yellow Controls - Top')
        s_min = cv2.getTrackbarPos('Y_S_min', 'Yellow Controls - Top')
        v_min = cv2.getTrackbarPos('Y_V_min', 'Yellow Controls - Top')
        h_max = cv2.getTrackbarPos('Y_H_max', 'Yellow Controls - Top')
        s_max = cv2.getTrackbarPos('Y_S_max', 'Yellow Controls - Top')
        v_max = cv2.getTrackbarPos('Y_V_max', 'Yellow Controls - Top')
        
        # Check if yellow values changed
        old_lower = self.colors['yellow']['lower']
        old_upper = self.colors['yellow']['upper']
        new_lower = [h_min, s_min, v_min]
        new_upper = [h_max, s_max, v_max]
        
        if old_lower != new_lower or old_upper != new_upper:
            adjustment = {
                'timestamp': current_time,
                'color': 'yellow',
                'old_lower': old_lower.copy(),
                'old_upper': old_upper.copy(),
                'new_lower': new_lower.copy(),
                'new_upper': new_upper.copy()
            }
            self.adjustment_data.append(adjustment)
            print(f"📝 yellow adjusted: {new_lower} -> {new_upper}")
        
        self.colors['yellow']['lower'] = new_lower
        self.colors['yellow']['upper'] = new_upper
        
        # Update green and blue from bottom window
        for color_name in ['green', 'blue']:
            color_short = color_name[0].upper()  # G, B
            h_min = cv2.getTrackbarPos(f'{color_short}_H_min', 'Green/Blue Controls - Bottom')
            s_min = cv2.getTrackbarPos(f'{color_short}_S_min', 'Green/Blue Controls - Bottom')
            v_min = cv2.getTrackbarPos(f'{color_short}_V_min', 'Green/Blue Controls - Bottom')
            h_max = cv2.getTrackbarPos(f'{color_short}_H_max', 'Green/Blue Controls - Bottom')
            s_max = cv2.getTrackbarPos(f'{color_short}_S_max', 'Green/Blue Controls - Bottom')
            v_max = cv2.getTrackbarPos(f'{color_short}_V_max', 'Green/Blue Controls - Bottom')
            
            # Check if values changed
            old_lower = self.colors[color_name]['lower']
            old_upper = self.colors[color_name]['upper']
            
            new_lower = [h_min, s_min, v_min]
            new_upper = [h_max, s_max, v_max]
            
            if old_lower != new_lower or old_upper != new_upper:
                # Record the adjustment
                adjustment = {
                    'timestamp': current_time,
                    'color': color_name,
                    'old_lower': old_lower.copy(),
                    'old_upper': old_upper.copy(),
                    'new_lower': new_lower.copy(),
                    'new_upper': new_upper.copy()
                }
                self.adjustment_data.append(adjustment)
                print(f"📝 {color_name} adjusted: {new_lower} -> {new_upper}")
            
            self.colors[color_name]['lower'] = new_lower
            self.colors[color_name]['upper'] = new_upper
    
    def stabilize_angle(self, color_name, new_angle):
        """Stabilize angle to prevent shaking"""
        if new_angle is None:
            return self.stable_angles[color_name]
        
        # If no stable angle yet, use the new angle
        if self.stable_angles[color_name] is None:
            self.stable_angles[color_name] = new_angle
            self.angle_history[color_name].append(new_angle)
            return new_angle
        
        # Check if angle change is significant
        angle_diff = abs(new_angle - self.stable_angles[color_name])
        
        # If angle change is small, keep the stable angle
        if angle_diff < self.angle_threshold:
            return self.stable_angles[color_name]
        
        # If angle change is significant, update stable angle
        self.stable_angles[color_name] = new_angle
        self.angle_history[color_name].append(new_angle)
        
        # Smooth the angle using history
        if len(self.angle_history[color_name]) > 1:
            smoothed_angle = np.mean(list(self.angle_history[color_name]))
            self.stable_angles[color_name] = smoothed_angle
        
        return self.stable_angles[color_name]
    
    def detect_stable_lines(self, frame, color_name):
        """Detect lines with stable angles"""
        if frame is None:
            return []
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get color range
        color_info = self.colors[color_name]
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
                stable_angle = self.stabilize_angle(color_name, raw_angle)
                
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
            label = f"{color_name} Line: {line['angle']:.1f}°"
            cv2.putText(result, label, (line['center'][0] - 50, line['center'][1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return result
    
    def display_side_camera_with_sliders(self, duration=120):
        """Display side camera with HSV sliders and data recording"""
        if not self.cap:
            print("❌ Camera not initialized")
            return
        
        # Create HSV sliders
        self.create_hsv_sliders()
        
        print("🎨 Side Camera Slider Viewer")
        print("=" * 60)
        print("🎯 Features:")
        print("   🔍 HSV sliders for side camera")
        print("   📝 Data recording for adjustments")
        print("   🔍 Stable angles (no shaking)")
        print("   📐 2x zoom, 400x400 frame")
        print("Press 'q' to quit, 's' to save frames, 'r' to record data")
        
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < duration:
            # Get frame from side camera
            ret, frame = self.cap.read()
            
            if not ret:
                print("❌ Failed to read from camera")
                break
            
            # Process frame
            processed = self.process_frame(frame)
            
            if processed is not None:
                # Update HSV ranges from sliders and record adjustments
                self.update_hsv_ranges()
                
                # Detect stable lines for each color
                all_lines = []
                result = processed.copy()
                
                for color_name in self.colors.keys():
                    lines = self.detect_stable_lines(processed, color_name)
                    all_lines.extend(lines)
                    result = self.draw_stable_lines(result, lines, color_name)
                
                # Add labels
                cv2.putText(result, "Side Camera - Needle Detection", (10, 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(result, f"Lines: {len(all_lines)}", (10, 45), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(result, f"Frame: {frame_count}", (10, 65), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.putText(result, f"Adjustments: {len(self.adjustment_data)}", (10, 85), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # Display frame
                cv2.imshow('Side Camera - Slider Viewer (2x zoom, 400x400)', result)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("👋 Quit requested")
                    break
                elif key == ord('s'):
                    cv2.imwrite(f"side_camera_slider_{frame_count}.jpg", result)
                    print(f"💾 Saved frame {frame_count}")
                elif key == ord('r'):
                    self.save_adjustment_data()
                    print("📝 Adjustment data saved")
            
            frame_count += 1
        
        print(f"✅ Side camera slider viewing completed: {frame_count} frames processed")
    
    def save_adjustment_data(self):
        """Save adjustment data to file"""
        filename = "side_camera_adjustments.json"
        
        data = {
            'camera_id': self.camera_id,
            'total_adjustments': len(self.adjustment_data),
            'adjustments': self.adjustment_data,
            'final_colors': self.colors
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"💾 Saved adjustment data to {filename}")
        print(f"📊 Total adjustments: {len(self.adjustment_data)}")
    
    def release_camera(self):
        """Release camera resources"""
        if self.cap:
            self.cap.release()
            print("🧹 Side camera released")
        cv2.destroyAllWindows()

def main():
    """Main function"""
    print("🎨 Side Camera Slider Viewer")
    print("=" * 70)
    print("🎯 Features:")
    print("   🔍 HSV sliders for side camera")
    print("   📝 Data recording for adjustments")
    print("   🔍 Stable angles (no shaking)")
    print("   📐 2x zoom, 400x400 frame")
    print("=" * 70)
    
    # Initialize viewer
    viewer = SideCameraSliderViewer(camera_id=1)
    
    if viewer.initialize_camera():
        print("✅ Side camera initialized with slider viewer")
        print("\n📋 Instructions:")
        print("1. Use HSV sliders to adjust side camera")
        print("2. Adjustments are automatically recorded")
        print("3. Press 's' to save test images")
        print("4. Press 'r' to save adjustment data")
        print("5. Press 'q' to quit")
        
        viewer.display_side_camera_with_sliders(duration=180)  # 3 minutes
        viewer.save_adjustment_data()
        viewer.release_camera()
    else:
        print("❌ Failed to initialize side camera")
    
    print("✅ Side camera slider viewing completed!")

if __name__ == "__main__":
    main()
