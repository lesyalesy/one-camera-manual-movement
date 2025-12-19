#!/usr/bin/env python3
"""
Two-Camera Automatic Alignment System
======================================

Automatically aligns the yellow needle with blue/green hair lines using both cameras.
Uses efficient control algorithms to minimize total movement and converge quickly.

Features:
- Automatic alignment mode (no manual input needed)
- Works with both front and side cameras simultaneously
- Efficient movement algorithm (minimizes total servo travel)
- Adaptive step sizing for fast convergence
- Visual feedback showing alignment progress
"""

import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports
import json
import os
from collections import deque


class TwoCameraAutoAlignment:
    """
    Automatic two-camera alignment system.
    
    Automatically moves servos to align needle with hair lines using both cameras.
    """
    
    def __init__(self, front_camera_id=0, side_camera_id=1, arduino_port=None):
        """
        Initialize the automatic alignment system.
        
        Args:
            front_camera_id: Camera index for front camera (default: 0)
            side_camera_id: Camera index for side camera (default: 1)
            arduino_port: Serial port for Arduino (None = auto-detect)
        """
        # Camera setup
        self.front_camera_id = front_camera_id
        self.side_camera_id = side_camera_id
        self.front_cap = None
        self.side_cap = None
        
        # Servo control
        self.arduino_port = arduino_port
        self.arduino_conn = None
        self.current_pan_angle = 45.0  # degrees (0-180)
        self.current_tilt_angle = 120.0  # degrees (0-180)
        
        # Load calibrated HSV ranges for both cameras
        self.front_colors = self._load_color_ranges('front_camera_adjustments.json')
        self.side_colors = self._load_color_ranges('side_camera_adjustments.json')
        
        # Target hair color
        self.target_hair_color = 'blue'  # 'blue' or 'green'
        
        # Line detection parameters
        self.min_area = 50
        self.angle_threshold = 30
        
        # Angle stabilization (separate for each camera)
        self.front_stable_angles = {'yellow': None, 'blue': None, 'green': None}
        self.side_stable_angles = {'yellow': None, 'blue': None, 'green': None}
        self.front_angle_history = {
            'yellow': deque(maxlen=15),
            'blue': deque(maxlen=15),
            'green': deque(maxlen=15)
        }
        self.side_angle_history = {
            'yellow': deque(maxlen=15),
            'blue': deque(maxlen=15),
            'green': deque(maxlen=15)
        }
        
        # Error stabilization
        self.front_error_history = deque(maxlen=10)
        self.side_error_history = deque(maxlen=10)
        self.front_stable_error = None
        self.side_stable_error = None
        
        # Control parameters
        self.angle_tolerance = 0.5  # degrees - consider aligned if error <= this
        
        # Color calibration mode
        self.calibration_mode = True  # Start in calibration mode by default
        self.calibration_camera = 'front'  # Which camera to calibrate ('front' or 'side')
        
        # Manual servo control
        self.input_mode = None  # 'pan' or 'tilt' when typing degrees
        self.input_text = ""  # Text being typed for degree input
        self.joystick_step_size = 1.0  # Degrees per WASD movement
        self.last_joystick_move_time = 0
        self.joystick_repeat_delay = 0.15  # Seconds between repeated movements
        
        # Servo timing (for manual control if needed)
        self.last_servo_command_time = 0
        self.min_command_interval = 0.15
        
        # Frame processing
        self.frame_width = 400
        self.frame_height = 400
        
        # Timing
        self.last_frame_time = time.time()
        self.target_fps = 30
        self.frame_interval = 1.0 / self.target_fps
    
    def _load_color_ranges(self, filename):
        """Load HSV color ranges from JSON file"""
        if not os.path.exists(filename):
            print(f"⚠️  Color calibration file '{filename}' not found. Using defaults.")
            return {
                'yellow': {'lower': [20, 100, 100], 'upper': [30, 255, 255]},
                'blue': {'lower': [100, 100, 100], 'upper': [130, 255, 255]},
                'green': {'lower': [40, 100, 100], 'upper': [80, 255, 255]}
            }
        
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            # Handle both old format (direct colors) and new format (final_colors key)
            if 'final_colors' in data:
                colors = data['final_colors']
            else:
                colors = data
            
            result = {}
            for color_name in ['yellow', 'blue', 'green']:
                if color_name in colors:
                    result[color_name] = {
                        'lower': colors[color_name]['lower'],
                        'upper': colors[color_name]['upper']
                    }
                else:
                    # Fallback defaults
                    defaults = {
                        'yellow': {'lower': [20, 100, 100], 'upper': [30, 255, 255]},
                        'blue': {'lower': [100, 100, 100], 'upper': [130, 255, 255]},
                        'green': {'lower': [40, 100, 100], 'upper': [80, 255, 255]}
                    }
                    result[color_name] = defaults[color_name]
            
            return result
        except Exception as e:
            print(f"⚠️  Error loading color ranges from '{filename}': {e}")
            return {
                'yellow': {'lower': [20, 100, 100], 'upper': [30, 255, 255]},
                'blue': {'lower': [100, 100, 100], 'upper': [130, 255, 255]},
                'green': {'lower': [40, 100, 100], 'upper': [80, 255, 255]}
            }
    
    def initialize_camera(self, camera_id, camera_name):
        """Initialize a camera"""
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            print(f"❌ Failed to open {camera_name} (ID: {camera_id})")
            return None
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        print(f"✅ {camera_name} initialized (ID: {camera_id})")
        return cap
    
    def initialize_arduino(self):
        """Initialize Arduino connection"""
        if self.arduino_port is None:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if 'Arduino' in port.description or 'USB' in port.description or 'CH340' in port.description:
                    self.arduino_port = port.device
                    break
        
        if self.arduino_port is None:
            print("⚠️  No Arduino port found. Running in simulation mode.")
            return False
        
        try:
            self.arduino_conn = serial.Serial(self.arduino_port, 9600, timeout=1)
            time.sleep(2)
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            return False
    
    def set_servo_angles(self, pan_angle=None, tilt_angle=None):
        """Set servo angles"""
        if self.arduino_conn is None:
            # Simulation mode
            if pan_angle is not None:
                old_pan = self.current_pan_angle
                self.current_pan_angle = pan_angle
                self.total_movement += abs(pan_angle - old_pan)
                print(f"   [SIMULATION] Pan={pan_angle:.1f}°")
            if tilt_angle is not None:
                old_tilt = self.current_tilt_angle
                self.current_tilt_angle = tilt_angle
                self.total_movement += abs(tilt_angle - old_tilt)
                print(f"   [SIMULATION] Tilt={tilt_angle:.1f}°")
            return True
        
        try:
            if not self.arduino_conn.is_open:
                print("   ⚠️  Arduino connection lost!")
                return False
            
            # Clear buffer
            if self.arduino_conn.in_waiting > 0:
                self.arduino_conn.reset_input_buffer()
            
            commands = []
            
            if pan_angle is not None:
                pan_angle = max(0, min(180, int(pan_angle)))
                self.current_pan_angle = pan_angle
                commands.append(f"P{pan_angle}\n")
            
            if tilt_angle is not None:
                tilt_angle = max(0, min(180, int(tilt_angle)))
                self.current_tilt_angle = tilt_angle
                commands.append(f"T{tilt_angle}\n")
            
            # Send commands
            for cmd in commands:
                self.arduino_conn.reset_output_buffer()
                self.arduino_conn.write(cmd.encode())
                self.arduino_conn.flush()
                time.sleep(0.1)
            
            self.last_servo_command_time = time.time()
            return True
        except Exception as e:
            print(f"   ❌ Error sending servo command: {e}")
            return False
    
    def process_frame(self, frame):
        """Process frame: crop center region and resize to 400x400"""
        if frame is None:
            return None
        
        height, width = frame.shape[:2]
        crop_size = int(min(width, height) / 2.0)
        center_x = width // 2
        center_y = height // 2
        
        x1 = max(0, center_x - crop_size // 2)
        y1 = max(0, center_y - crop_size // 2)
        x2 = min(width, x1 + crop_size)
        y2 = min(height, y1 + crop_size)
        
        cropped = frame[y1:y2, x1:x2]
        square_frame = cv2.resize(cropped, (400, 400))
        
        return square_frame
    
    def stabilize_angle(self, color_name, new_angle, camera_type='front'):
        """Stabilize angle using median filtering"""
        if camera_type == 'front':
            angle_history = self.front_angle_history[color_name]
            stable_angles = self.front_stable_angles
        else:
            angle_history = self.side_angle_history[color_name]
            stable_angles = self.side_stable_angles
        
        if new_angle is None:
            return stable_angles[color_name]
        
        angle_history.append(new_angle)
        
        if stable_angles[color_name] is None:
            stable_angles[color_name] = new_angle
            return new_angle
        
        # Use median of recent samples
        if len(angle_history) >= 5:
            angles = list(angle_history)
            smoothed_angle = np.median(angles)
            angle_diff = abs(smoothed_angle - stable_angles[color_name])
            
            # Only update if change is significant
            if angle_diff >= 1.0:
                stable_angles[color_name] = smoothed_angle
        
        return stable_angles[color_name]
    
    def detect_line(self, frame, color_name, camera_type='front'):
        """Detect a line of a specific color in the frame"""
        if frame is None:
            return None
        
        # Get color ranges for this camera
        colors = self.front_colors if camera_type == 'front' else self.side_colors
        
        if color_name not in colors:
            return None
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get color range
        color_info = colors[color_name]
        lower = np.array(color_info['lower'])
        upper = np.array(color_info['upper'])
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.min_area:
            return None
        
        # Use cv2.fitLine for robust line fitting
        points = largest_contour.reshape(-1, 2)
        if len(points) < 2:
            return None
        
        line_params = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(line_params[0].item() if hasattr(line_params[0], 'item') else line_params[0])
        vy = float(line_params[1].item() if hasattr(line_params[1], 'item') else line_params[1])
        x0 = float(line_params[2].item() if hasattr(line_params[2], 'item') else line_params[2])
        y0 = float(line_params[3].item() if hasattr(line_params[3], 'item') else line_params[3])
        
        # Normalize direction vector
        if vx < 0:
            vx = -vx
            vy = -vy
        elif abs(vx) < 1e-6:
            if vy < 0:
                vy = -vy
        
        # Calculate angle
        raw_angle = np.arctan2(vy, vx) * 180 / np.pi
        while raw_angle < 0:
            raw_angle += 180
        while raw_angle >= 180:
            raw_angle -= 180
        
        # Stabilize angle
        stable_angle = self.stabilize_angle(color_name, raw_angle, camera_type)
        
        # Calculate endpoints
        t_values = []
        for pt in points:
            t = (pt[0] - x0) * vx + (pt[1] - y0) * vy
            t_values.append(t)
        
        if t_values:
            t_min = min(t_values)
            t_max = max(t_values)
        else:
            t_min, t_max = -50, 50
        
        point1 = (int(x0 + t_min * vx), int(y0 + t_min * vy))
        point2 = (int(x0 + t_max * vx), int(y0 + t_max * vy))
        
        if point1[0] > point2[0] or (point1[0] == point2[0] and point1[1] > point2[1]):
            point1, point2 = point2, point1
        
        center_x = (point1[0] + point2[0]) // 2
        center_y = (point1[1] + point2[1]) // 2
        
        return {
            'point1': point1,
            'point2': point2,
            'center': (center_x, center_y),
            'angle': raw_angle,
            'stable_angle': stable_angle,
            'area': area,
            'mask': mask
        }
    
    def calculate_angle_difference(self, needle_line, hair_line):
        """Calculate angle difference between needle and hair lines"""
        if needle_line is None or hair_line is None:
            return None
        
        needle_angle = needle_line['angle']
        hair_angle = hair_line['angle']
        
        raw_diff = needle_angle - hair_angle
        abs_diff = abs(raw_diff)
        
        if abs_diff > 90:
            angle_diff = 180 - abs_diff
            if raw_diff > 0:
                angle_diff = -angle_diff
        else:
            angle_diff = raw_diff
        
        return {
            'needle_angle': needle_angle,
            'hair_angle': hair_angle,
            'angle_diff': angle_diff
        }
    
    def draw_detection_results(self, frame, needle_line, hair_line, angle_info, camera_type='front'):
        """Draw detection results on frame"""
        result = frame.copy()
        
        # Draw needle line
        if needle_line:
            cv2.line(result, needle_line['point1'], needle_line['point2'], (0, 255, 255), 3)
            cv2.circle(result, needle_line['center'], 5, (0, 255, 255), -1)
            angle = needle_line['angle']
            center = needle_line['center']
            cv2.putText(result, f"Needle: {angle:.1f}°", 
                       (center[0] - 50, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw hair line
        if hair_line:
            color = (255, 100, 0) if self.target_hair_color == 'blue' else (0, 255, 0)
            cv2.line(result, hair_line['point1'], hair_line['point2'], color, 3)
            cv2.circle(result, hair_line['center'], 5, color, -1)
            angle = hair_line['angle']
            center = hair_line['center']
            cv2.putText(result, f"Hair: {angle:.1f}°", 
                       (center[0] - 50, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw error info
        if angle_info:
            error = angle_info['angle_diff']
            is_aligned = abs(error) <= self.angle_tolerance
            
            error_display = f"{error:+.2f}°"
            color = (0, 255, 0) if is_aligned else (0, 0, 255)
            
            cv2.putText(result, f"Error: {error_display}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            if is_aligned:
                cv2.putText(result, "ALIGNED", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw camera label
        camera_label = "FRONT" if camera_type == 'front' else "SIDE"
        cv2.putText(result, camera_label, 
                   (10, result.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return result
    
    def create_calibration_sliders(self, camera_type='front'):
        """Create separate HSV sliders for each color with visual display"""
        colors = self.front_colors if camera_type == 'front' else self.side_colors
        
        # Create separate window for Yellow
        yellow_window = f'Yellow - {camera_type.upper()} Camera'
        cv2.namedWindow(yellow_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(yellow_window, 1000, 250)
        cv2.moveWindow(yellow_window, 100, 100)
        
        if 'yellow' in colors:
            lower = colors['yellow']['lower']
            upper = colors['yellow']['upper']
            cv2.createTrackbar('Y_H_min', yellow_window, lower[0], 179, lambda x: None)
            cv2.createTrackbar('Y_S_min', yellow_window, lower[1], 255, lambda x: None)
            cv2.createTrackbar('Y_V_min', yellow_window, lower[2], 255, lambda x: None)
            cv2.createTrackbar('Y_H_max', yellow_window, upper[0], 179, lambda x: None)
            cv2.createTrackbar('Y_S_max', yellow_window, upper[1], 255, lambda x: None)
            cv2.createTrackbar('Y_V_max', yellow_window, upper[2], 255, lambda x: None)
        
        # Create separate window for Blue
        blue_window = f'Blue - {camera_type.upper()} Camera'
        cv2.namedWindow(blue_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(blue_window, 1000, 250)
        cv2.moveWindow(blue_window, 100, 400)
        
        if 'blue' in colors:
            lower = colors['blue']['lower']
            upper = colors['blue']['upper']
            cv2.createTrackbar('B_H_min', blue_window, lower[0], 179, lambda x: None)
            cv2.createTrackbar('B_S_min', blue_window, lower[1], 255, lambda x: None)
            cv2.createTrackbar('B_V_min', blue_window, lower[2], 255, lambda x: None)
            cv2.createTrackbar('B_H_max', blue_window, upper[0], 179, lambda x: None)
            cv2.createTrackbar('B_S_max', blue_window, upper[1], 255, lambda x: None)
            cv2.createTrackbar('B_V_max', blue_window, upper[2], 255, lambda x: None)
        
        # Create separate window for Green
        green_window = f'Green - {camera_type.upper()} Camera'
        cv2.namedWindow(green_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(green_window, 1000, 250)
        cv2.moveWindow(green_window, 100, 700)
        
        if 'green' in colors:
            lower = colors['green']['lower']
            upper = colors['green']['upper']
            cv2.createTrackbar('G_H_min', green_window, lower[0], 179, lambda x: None)
            cv2.createTrackbar('G_S_min', green_window, lower[1], 255, lambda x: None)
            cv2.createTrackbar('G_V_min', green_window, lower[2], 255, lambda x: None)
            cv2.createTrackbar('G_H_max', green_window, upper[0], 179, lambda x: None)
            cv2.createTrackbar('G_S_max', green_window, upper[1], 255, lambda x: None)
            cv2.createTrackbar('G_V_max', green_window, upper[2], 255, lambda x: None)
    
    def draw_calibration_window(self, window_name, color_name, colors):
        """Draw calibration window with current values and labels"""
        # Create a display image for the window
        display = np.ones((250, 1000, 3), dtype=np.uint8) * 240  # Light gray background
        
        if color_name not in colors:
            return display
        
        color_info = colors[color_name]
        lower = color_info['lower']
        upper = color_info['upper']
        prefix = color_name[0].upper()
        
        # Title
        cv2.putText(display, f"{color_name.upper()} Color Calibration", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        
        # Get current slider values
        try:
            h_min = cv2.getTrackbarPos(f'{prefix}_H_min', window_name)
            s_min = cv2.getTrackbarPos(f'{prefix}_S_min', window_name)
            v_min = cv2.getTrackbarPos(f'{prefix}_V_min', window_name)
            h_max = cv2.getTrackbarPos(f'{prefix}_H_max', window_name)
            s_max = cv2.getTrackbarPos(f'{prefix}_S_max', window_name)
            v_max = cv2.getTrackbarPos(f'{prefix}_V_max', window_name)
        except:
            h_min, s_min, v_min = lower
            h_max, s_max, v_max = upper
        
        # Display current values
        y_pos = 70
        cv2.putText(display, f"Lower: H={h_min:3d} S={s_min:3d} V={v_min:3d}", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        y_pos += 30
        cv2.putText(display, f"Upper: H={h_max:3d} S={s_max:3d} V={v_max:3d}", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Instructions
        y_pos += 40
        cv2.putText(display, "Adjust sliders above to calibrate color detection", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        y_pos += 25
        cv2.putText(display, "Changes apply immediately to camera views", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        return display
    
    def update_calibration_from_sliders(self, camera_type='front'):
        """Update HSV ranges from calibration sliders (separate windows)"""
        colors = self.front_colors if camera_type == 'front' else self.side_colors
        
        # Update Yellow
        if 'yellow' in colors:
            yellow_window = f'Yellow - {camera_type.upper()} Camera'
            try:
                h_min = cv2.getTrackbarPos('Y_H_min', yellow_window)
                s_min = cv2.getTrackbarPos('Y_S_min', yellow_window)
                v_min = cv2.getTrackbarPos('Y_V_min', yellow_window)
                h_max = cv2.getTrackbarPos('Y_H_max', yellow_window)
                s_max = cv2.getTrackbarPos('Y_S_max', yellow_window)
                v_max = cv2.getTrackbarPos('Y_V_max', yellow_window)
                colors['yellow']['lower'] = [h_min, s_min, v_min]
                colors['yellow']['upper'] = [h_max, s_max, v_max]
            except:
                pass
        
        # Update Blue
        if 'blue' in colors:
            blue_window = f'Blue - {camera_type.upper()} Camera'
            try:
                h_min = cv2.getTrackbarPos('B_H_min', blue_window)
                s_min = cv2.getTrackbarPos('B_S_min', blue_window)
                v_min = cv2.getTrackbarPos('B_V_min', blue_window)
                h_max = cv2.getTrackbarPos('B_H_max', blue_window)
                s_max = cv2.getTrackbarPos('B_S_max', blue_window)
                v_max = cv2.getTrackbarPos('B_V_max', blue_window)
                colors['blue']['lower'] = [h_min, s_min, v_min]
                colors['blue']['upper'] = [h_max, s_max, v_max]
            except:
                pass
        
        # Update Green
        if 'green' in colors:
            green_window = f'Green - {camera_type.upper()} Camera'
            try:
                h_min = cv2.getTrackbarPos('G_H_min', green_window)
                s_min = cv2.getTrackbarPos('G_S_min', green_window)
                v_min = cv2.getTrackbarPos('G_V_min', green_window)
                h_max = cv2.getTrackbarPos('G_H_max', green_window)
                s_max = cv2.getTrackbarPos('G_S_max', green_window)
                v_max = cv2.getTrackbarPos('G_V_max', green_window)
                colors['green']['lower'] = [h_min, s_min, v_min]
                colors['green']['upper'] = [h_max, s_max, v_max]
            except:
                pass
    
    def handle_joystick_move(self, servo_type, delta):
        """Handle manual servo movement with rate limiting"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_servo_command_time < self.min_command_interval:
            return
        
        if abs(delta) <= 1.0:
            if current_time - self.last_joystick_move_time < self.joystick_repeat_delay:
                return
            self.last_joystick_move_time = current_time
        
        current_pos = self.current_pan_angle if servo_type == 'pan' else self.current_tilt_angle
        new_pos = current_pos + delta
        new_pos = max(0, min(180, new_pos))
        
        if abs(new_pos - current_pos) < 0.1:
            return
        
        print(f"   🔧 Moving {servo_type.upper()} from {current_pos:.1f}° to {new_pos:.1f}° (Δ{delta:+.1f}°)")
        
        self.last_servo_command_time = time.time()
        
        success = False
        if servo_type == 'pan':
            success = self.set_servo_angles(pan_angle=new_pos, tilt_angle=None)
        else:
            success = self.set_servo_angles(pan_angle=None, tilt_angle=new_pos)
        
        if success:
            time.sleep(0.1)  # Small delay for servo movement
    
    def handle_input_key(self, key):
        """Handle keyboard input for angle input mode"""
        if key == 13:  # Enter key
            if self.input_text and self.input_mode:
                try:
                    angle = float(self.input_text)
                    angle = max(0, min(180, angle))
                    print(f"   🎮 Setting {self.input_mode.upper()} to {angle:.1f}°")
                    if self.input_mode == 'pan':
                        self.set_servo_angles(pan_angle=angle, tilt_angle=None)
                    else:
                        self.set_servo_angles(pan_angle=None, tilt_angle=angle)
                    self.input_text = ""
                    self.input_mode = None
                except ValueError:
                    print(f"   ❌ Invalid input: '{self.input_text}'")
                    self.input_text = ""
                    self.input_mode = None
            return True
        elif key == 27:  # ESC key
            if self.input_mode:
                print(f"   ❌ Cancelled {self.input_mode.upper()} input")
                self.input_text = ""
                self.input_mode = None
            return True
        elif self.input_mode:  # Number input
            if key == ord('-') and len(self.input_text) == 0:
                self.input_text = '-'
                return True
            elif key == ord('.') and '.' not in self.input_text:
                self.input_text += '.'
                return True
            elif key >= ord('0') and key <= ord('9'):
                self.input_text += chr(key)
                return True
        return False
    
    def save_calibration(self, camera_type='front'):
        """Save calibration data to JSON file"""
        colors = self.front_colors if camera_type == 'front' else self.side_colors
        filename = f'{camera_type}_camera_adjustments.json'
        
        # Prepare final colors structure
        final_colors = {}
        for color_name, color_info in colors.items():
            final_colors[color_name] = {
                'lower': color_info['lower'],
                'upper': color_info['upper']
            }
        
        # Save to file
        data = {
            'camera_id': self.front_camera_id if camera_type == 'front' else self.side_camera_id,
            'final_colors': final_colors
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"💾 Saved {camera_type} camera calibration to {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to save calibration: {e}")
            return False
    
    def run(self):
        """Main execution loop"""
        # Initialize cameras
        self.front_cap = self.initialize_camera(self.front_camera_id, "Front Camera")
        if self.front_cap is None:
            return
        
        self.side_cap = self.initialize_camera(self.side_camera_id, "Side Camera")
        if self.side_cap is None:
            print("⚠️  Failed to open side camera. Continuing with front only.")
        
        # Initialize Arduino
        arduino_connected = self.initialize_arduino()
        
        if arduino_connected:
            # Send initial servo positions to physically move servos to starting positions
            print(f"\n📌 Moving servos to initial positions: Pan={self.current_pan_angle:.1f}°, Tilt={self.current_tilt_angle:.1f}°")
            self.set_servo_angles(pan_angle=self.current_pan_angle, tilt_angle=self.current_tilt_angle)
            time.sleep(1.5)  # Wait for servos to move to initial positions
            print(f"✅ Servos initialized at starting positions")
        
        print("\n" + "="*70)
        print("🎨 TWO-CAMERA COLOR CALIBRATION SYSTEM")
        print("="*70)
        print("\n📋 Mode: COLOR CALIBRATION WITH MANUAL MOVEMENT")
        print("   Calibrate color detection for both cameras")
        print("\n📋 Controls:")
        print("   'q' - Quit")
        print("   '1' - Calibrate FRONT camera")
        print("   '2' - Calibrate SIDE camera")
        print("   's' - Save calibration")
        print("\n📋 Manual Servo Control:")
        print("   WASD - Move servos (W=Tilt↑, S=Tilt↓, A=Pan←, D=Pan→)")
        print("   'p' - Enter PAN angle (type number, press Enter)")
        print("   't' - Enter TILT angle (type number, press Enter)")
        print("="*70)
        
        # Start in calibration mode - create sliders immediately
        if self.calibration_mode:
            print("\n🎨 Starting in COLOR CALIBRATION mode")
            print("   Calibration windows will appear shortly...")
            self.create_calibration_sliders(self.calibration_camera)
        
        window_name = "Two-Camera Auto Alignment"
        cv2.namedWindow(window_name)
        
        while True:
            # Frame rate limiting
            current_time = time.time()
            elapsed = current_time - self.last_frame_time
            if elapsed < self.frame_interval:
                time.sleep(self.frame_interval - elapsed)
            self.last_frame_time = time.time()
            
            # Read front camera
            ret_front, frame_front = self.front_cap.read()
            if not ret_front or frame_front is None:
                print("❌ Failed to read from front camera")
                break
            
            processed_front = self.process_frame(frame_front)
            if processed_front is None:
                continue
            
            needle_line_front = self.detect_line(processed_front, 'yellow', 'front')
            hair_line_front = self.detect_line(processed_front, self.target_hair_color, 'front')
            
            angle_info_front = None
            front_error = None
            if needle_line_front and hair_line_front:
                angle_info_front = self.calculate_angle_difference(needle_line_front, hair_line_front)
                if angle_info_front:
                    front_error = angle_info_front['angle_diff']
                    # Stabilize error
                    self.front_error_history.append(front_error)
                    if len(self.front_error_history) >= 5:
                        self.front_stable_error = np.median(list(self.front_error_history))
                    else:
                        self.front_stable_error = front_error
            
            # Draw front camera view
            result_front = self.draw_detection_results(processed_front, needle_line_front, hair_line_front, angle_info_front, 'front')
            
            # Read and process side camera if available
            result_side = None
            side_error = None
            if self.side_cap is not None:
                ret_side, frame_side = self.side_cap.read()
                if ret_side and frame_side is not None:
                    processed_side = self.process_frame(frame_side)
                    if processed_side is not None:
                        needle_line_side = self.detect_line(processed_side, 'yellow', 'side')
                        hair_line_side = self.detect_line(processed_side, self.target_hair_color, 'side')
                        
                        angle_info_side = None
                        if needle_line_side and hair_line_side:
                            angle_info_side = self.calculate_angle_difference(needle_line_side, hair_line_side)
                            if angle_info_side:
                                side_error = angle_info_side['angle_diff']
                                # Stabilize error
                                self.side_error_history.append(side_error)
                                if len(self.side_error_history) >= 5:
                                    self.side_stable_error = np.median(list(self.side_error_history))
                                else:
                                    self.side_stable_error = side_error
                        
                        result_side = self.draw_detection_results(processed_side, needle_line_side, hair_line_side, angle_info_side, 'side')
            
            # Update calibration if in calibration mode
            if self.calibration_mode:
                self.update_calibration_from_sliders(self.calibration_camera)
                # Update calibration window displays
                colors = self.front_colors if self.calibration_camera == 'front' else self.side_colors
                try:
                    yellow_display = self.draw_calibration_window(
                        f'Yellow - {self.calibration_camera.upper()} Camera', 'yellow', colors)
                    cv2.imshow(f'Yellow - {self.calibration_camera.upper()} Camera', yellow_display)
                except:
                    pass
                try:
                    blue_display = self.draw_calibration_window(
                        f'Blue - {self.calibration_camera.upper()} Camera', 'blue', colors)
                    cv2.imshow(f'Blue - {self.calibration_camera.upper()} Camera', blue_display)
                except:
                    pass
                try:
                    green_display = self.draw_calibration_window(
                        f'Green - {self.calibration_camera.upper()} Camera', 'green', colors)
                    cv2.imshow(f'Green - {self.calibration_camera.upper()} Camera', green_display)
                except:
                    pass
            
            # Combine views
            if result_side is not None:
                combined = np.hstack([result_front, result_side])
            else:
                combined = result_front
            
            # Add status info
            status_y = combined.shape[0] - 60
            cv2.putText(combined, f"Pan: {self.current_pan_angle:.1f}° | Tilt: {self.current_tilt_angle:.1f}°", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Show calibration mode status
            mode_text = f"CALIBRATION MODE - {self.calibration_camera.upper()} Camera"
            cv2.putText(combined, mode_text, 
                       (10, status_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(combined, "Press '1' for Front, '2' for Side, 's' to Save, 'q' to Quit", 
                       (10, status_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            
            # Show input mode if active
            if self.input_mode:
                input_text_display = self.input_text if self.input_text else "_"
                cv2.putText(combined, f"{self.input_mode.upper()} Input: {input_text_display} (Enter=Set, ESC=Cancel)", 
                           (10, status_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            cv2.imshow(window_name, combined)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            # Check for P/T keys first to activate input mode
            elif key == ord('p') or key == ord('P'):
                self.input_mode = 'pan'
                self.input_text = ""
                print("   📝 PAN input mode activated. Type numbers, then press Enter.")
            elif key == ord('t') or key == ord('T'):
                self.input_mode = 'tilt'
                self.input_text = ""
                print("   📝 TILT input mode activated. Type numbers, then press Enter.")
            # Handle input mode (number input, Enter, ESC)
            elif self.input_mode:
                self.handle_input_key(key)
            elif key == ord('b'):
                self.target_hair_color = 'blue'
                print("   🎯 Target hair: BLUE")
            elif key == ord('g'):
                self.target_hair_color = 'green'
                print("   🎯 Target hair: GREEN")
            elif key == ord('w') or key == ord('W'):
                self.handle_joystick_move('tilt', 1.0)
            elif key == ord('s') or key == ord('S'):
                self.handle_joystick_move('tilt', -1.0)
            elif key == ord('a') or key == ord('A'):
                self.handle_joystick_move('pan', -1.0)
            elif key == ord('d') or key == ord('D'):
                self.handle_joystick_move('pan', 1.0)
            elif key == ord('1'):
                # Switch to front camera calibration
                try:
                    cv2.destroyWindow(f'Yellow - {self.calibration_camera.upper()} Camera')
                    cv2.destroyWindow(f'Blue - {self.calibration_camera.upper()} Camera')
                    cv2.destroyWindow(f'Green - {self.calibration_camera.upper()} Camera')
                except:
                    pass
                self.calibration_camera = 'front'
                self.create_calibration_sliders('front')
                print("   📷 Calibrating FRONT camera")
            elif key == ord('2'):
                # Switch to side camera calibration
                try:
                    cv2.destroyWindow(f'Yellow - {self.calibration_camera.upper()} Camera')
                    cv2.destroyWindow(f'Blue - {self.calibration_camera.upper()} Camera')
                    cv2.destroyWindow(f'Green - {self.calibration_camera.upper()} Camera')
                except:
                    pass
                self.calibration_camera = 'side'
                self.create_calibration_sliders('side')
                print("   📷 Calibrating SIDE camera")
            elif key == ord('s'):
                # Save calibration (only if not in input mode and not WASD)
                if not self.input_mode:
                    self.save_calibration(self.calibration_camera)
        
        # Cleanup
        if self.front_cap:
            self.front_cap.release()
        if self.side_cap:
            self.side_cap.release()
        if self.arduino_conn:
            self.arduino_conn.close()
        cv2.destroyAllWindows()
        
        print(f"\n✅ Color calibration complete!")
        print("✅ System shutdown complete.")


if __name__ == "__main__":
    system = TwoCameraAutoAlignment(front_camera_id=0, side_camera_id=1)
    system.run()

