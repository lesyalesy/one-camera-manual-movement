#!/usr/bin/env python3
"""
Two-Camera Manual Alignment System
===================================

Manual alignment system for both front and side cameras.
Allows manual control of servos while viewing both camera feeds.

Features:
- Both cameras open simultaneously
- Manual servo control (WASD, P/T keys)
- Visual control panel with buttons
- Color calibration support
- Real-time angle detection and display
"""

import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports
import json
import os
from collections import deque


class TwoCameraManualAlignment:
    """
    Two-camera manual alignment system.
    
    Opens both cameras and allows manual control of servos.
    """
    
    def __init__(self, front_camera_id=0, side_camera_id=1, arduino_port=None):
        """
        Initialize the two-camera alignment system.
        
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
        
        # Locked display (frozen until movement)
        self.locked_front_needle = None
        self.locked_front_hair = None
        self.locked_front_angle_info = None
        self.locked_side_needle = None
        self.locked_side_hair = None
        self.locked_side_angle_info = None
        self.last_movement_time = 0
        self.movement_cooldown = 0.5  # Seconds to wait after movement before locking
        
        # Control parameters
        self.angle_tolerance = 0.5  # degrees - consider aligned if error <= this
        
        # Manual control
        self.input_mode = None  # 'pan' or 'tilt' when typing degrees
        self.input_text = ""  # Text being typed for degree input
        self.joystick_step_size = 1.0  # Degrees per WASD movement
        self.last_joystick_move_time = 0
        self.joystick_repeat_delay = 0.05  # Seconds between repeated movements (reduced for faster response)
        
        # Servo timing
        self.last_servo_command_time = 0
        self.min_command_interval = 0.05  # Reduced for faster response
        
        # Control panel
        self.control_panel_width = 300
        self.camera_frame_width = 400
        
        # Frame processing
        self.frame_width = 400
        self.frame_height = 400
        
        # Timing
        self.last_frame_time = time.time()
        self.target_fps = 60  # Increased for smoother display
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
            time.sleep(2)  # Wait for Arduino to initialize
            
            # Clear any startup messages from Arduino
            self.arduino_conn.reset_input_buffer()
            time.sleep(0.5)
            
            # Read and display any Arduino startup messages
            if self.arduino_conn.in_waiting > 0:
                print("   📥 Arduino startup messages:")
                while self.arduino_conn.in_waiting > 0:
                    line = self.arduino_conn.readline().decode().strip()
                    if line:
                        print(f"      {line}")
            
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
                self.current_pan_angle = pan_angle
                print(f"   [SIMULATION] Pan={int(round(pan_angle))}°")
            if tilt_angle is not None:
                self.current_tilt_angle = tilt_angle
                print(f"   [SIMULATION] Tilt={int(round(tilt_angle))}°")
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
                # Don't reset output buffer - might interfere with command
                cmd_bytes = cmd.encode()
                bytes_written = self.arduino_conn.write(cmd_bytes)
                self.arduino_conn.flush()
                print(f"   📤 Sent to Arduino: {cmd.strip()} ({bytes_written} bytes)")  # Debug output
                
                # Wait for Arduino to process command
                time.sleep(0.15)  # Increased delay to ensure Arduino processes command
                
                # Read Arduino response to confirm command was processed
                response_lines = []
                start_time = time.time()
                while time.time() - start_time < 0.5:  # Wait up to 0.5s for response
                    if self.arduino_conn.in_waiting > 0:
                        try:
                            line = self.arduino_conn.readline().decode().strip()
                            if line and not line.startswith("⚠️") and not "DISABLED" in line:
                                # Skip startup messages, only show actual command responses
                                response_lines.append(line)
                        except:
                            break
                    time.sleep(0.01)
                
                if response_lines:
                    for response in response_lines:
                        print(f"   📥 Arduino: {response}")
                else:
                    print(f"   ⚠️  No response from Arduino for {cmd.strip()}")
            
            self.last_servo_command_time = time.time()
            return True
        except Exception as e:
            print(f"   ❌ Error sending servo command: {e}")
            return False
    
    def free_servos(self):
        """Disable servos by sending STOP command to Arduino"""
        if self.arduino_conn and self.arduino_conn.is_open:
            try:
                print("\n🔓 Freeing servos (disabling PWM)...")
                self.arduino_conn.reset_output_buffer()
                self.arduino_conn.write("STOP\n".encode())
                self.arduino_conn.flush()
                time.sleep(0.2)  # Give Arduino time to process
                print("✅ Servos freed (PWM disabled)")
            except Exception as e:
                print(f"   ⚠️  Error freeing servos: {e}")
    
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
        
        # Find actual start and end points of the line (farthest points apart)
        points = largest_contour.reshape(-1, 2)
        if len(points) < 2:
            return None
        
        # Find the two points that are farthest apart (start and end of line)
        max_dist = 0
        p1_idx = None
        p2_idx = None
        
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                dist = np.sqrt((points[i][0] - points[j][0])**2 + (points[i][1] - points[j][1])**2)
                if dist > max_dist:
                    max_dist = dist
                    p1_idx = i
                    p2_idx = j
        
        if p1_idx is None or p2_idx is None:
            return None
        
        # Ensure consistent endpoint ordering: use leftmost point as point1
        # If x-coordinates are equal, use topmost point
        if points[p1_idx][0] < points[p2_idx][0]:
            point1 = tuple(points[p1_idx])
            point2 = tuple(points[p2_idx])
        elif points[p1_idx][0] > points[p2_idx][0]:
            point1 = tuple(points[p2_idx])
            point2 = tuple(points[p1_idx])
        else:
            # x-coordinates are equal, use y-coordinate (topmost as point1)
            if points[p1_idx][1] < points[p2_idx][1]:
                point1 = tuple(points[p1_idx])
                point2 = tuple(points[p2_idx])
            else:
                point1 = tuple(points[p2_idx])
                point2 = tuple(points[p1_idx])
        
        # Calculate angle directly from start point to end point
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        
        # Calculate angle in degrees
        raw_angle = np.arctan2(dy, dx) * 180 / np.pi
        
        # Normalize to [0, 180) range (lines are undirected, so θ and θ+180° are the same)
        if raw_angle < 0:
            raw_angle += 180
        elif raw_angle >= 180:
            raw_angle -= 180
        
        # Stabilize angle
        stable_angle = self.stabilize_angle(color_name, raw_angle, camera_type)
        
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
        """Calculate angle difference between needle and hair lines
        Uses rounded displayed values for consistency with screen display"""
        if needle_line is None or hair_line is None:
            return None
        
        # Round angles to match what's displayed on screen
        needle_angle = int(round(needle_line['angle']))
        hair_angle = int(round(hair_line['angle']))
        
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
            # Use black color for text for better readability
            cv2.putText(result, f"Needle: {int(round(angle))}", 
                       (center[0] - 50, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # Draw hair line
        if hair_line:
            color = (255, 100, 0) if self.target_hair_color == 'blue' else (0, 255, 0)
            cv2.line(result, hair_line['point1'], hair_line['point2'], color, 3)
            cv2.circle(result, hair_line['center'], 5, color, -1)
            angle = hair_line['angle']
            center = hair_line['center']
            # Use black color for text for better readability
            cv2.putText(result, f"Hair: {int(round(angle))}", 
                       (center[0] - 50, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # Draw error info
        if angle_info:
            error = angle_info['angle_diff']
            is_aligned = abs(error) <= self.angle_tolerance
            
            error_display = f"{int(round(error)):+d}"
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
        
        print(f"   🔧 Moving {servo_type.upper()} from {int(round(current_pos))}° to {int(round(new_pos))}° (Δ{int(round(delta)):+d}°)")
        
        self.last_servo_command_time = time.time()
        self.last_movement_time = time.time()  # Mark movement to unlock display
        
        success = False
        if servo_type == 'pan':
            success = self.set_servo_angles(pan_angle=new_pos, tilt_angle=None)
        else:
            success = self.set_servo_angles(pan_angle=None, tilt_angle=new_pos)
        
        if success:
            time.sleep(0.05)  # Reduced delay for faster response
    
    def handle_input_key(self, key):
        """Handle keyboard input for angle input mode"""
        if key == 0 or key == 255:  # No key pressed
            return False
        if key == 13:  # Enter key
            if self.input_text and self.input_mode:
                try:
                    angle = float(self.input_text)
                    angle = max(0, min(180, angle))
                    print(f"   🎮 Setting {self.input_mode.upper()} to {int(round(angle))}°")
                    if self.input_mode == 'pan':
                        self.set_servo_angles(pan_angle=angle, tilt_angle=None)
                    else:
                        self.set_servo_angles(pan_angle=None, tilt_angle=angle)
                    self.last_movement_time = time.time()  # Mark movement to unlock display
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
                print(f"   📝 {self.input_mode.upper()} input: {self.input_text}")  # Debug
                return True
            elif key == ord('.') and '.' not in self.input_text:
                self.input_text += '.'
                print(f"   📝 {self.input_mode.upper()} input: {self.input_text}")  # Debug
                return True
            elif key >= ord('0') and key <= ord('9'):
                self.input_text += chr(key)
                print(f"   📝 {self.input_mode.upper()} input: {self.input_text}")  # Debug
                return True
        return False
    
    def create_control_panel(self, target_height=400):
        """Create control panel with buttons"""
        panel_height = target_height
        panel = np.ones((panel_height, self.control_panel_width, 3), dtype=np.uint8) * 240
        
        # Title
        cv2.putText(panel, "CONTROL PANEL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # Buttons
        button_y = 60
        button_height = 40
        button_spacing = 50
        
        # PAN buttons
        cv2.rectangle(panel, (10, button_y), (140, button_y + button_height), (200, 200, 200), -1)
        cv2.rectangle(panel, (10, button_y), (140, button_y + button_height), (0, 0, 0), 2)
        cv2.putText(panel, "PAN -", (20, button_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        cv2.rectangle(panel, (150, button_y), (280, button_y + button_height), (200, 200, 200), -1)
        cv2.rectangle(panel, (150, button_y), (280, button_y + button_height), (0, 0, 0), 2)
        cv2.putText(panel, "PAN +", (160, button_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        button_y += button_spacing
        
        # TILT buttons
        cv2.rectangle(panel, (10, button_y), (140, button_y + button_height), (200, 200, 200), -1)
        cv2.rectangle(panel, (10, button_y), (140, button_y + button_height), (0, 0, 0), 2)
        cv2.putText(panel, "TILT -", (20, button_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        cv2.rectangle(panel, (150, button_y), (280, button_y + button_height), (200, 200, 200), -1)
        cv2.rectangle(panel, (150, button_y), (280, button_y + button_height), (0, 0, 0), 2)
        cv2.putText(panel, "TILT +", (160, button_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        button_y += button_spacing + 10
        
        # Input fields
        cv2.putText(panel, "PAN Input:", (10, button_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        button_y += 25
        field_color = (200, 255, 200) if self.input_mode == 'pan' else (255, 255, 255)
        border_color = (0, 200, 0) if self.input_mode == 'pan' else (0, 0, 0)
        cv2.rectangle(panel, (10, button_y), (280, button_y + 30), field_color, -1)
        cv2.rectangle(panel, (10, button_y), (280, button_y + 30), border_color, 2)
        if self.input_mode == 'pan':
            cv2.putText(panel, self.input_text, (15, button_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        button_y += 50  # Increased spacing to move TILT Input lower
        
        cv2.putText(panel, "TILT Input:", (10, button_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        button_y += 25
        field_color = (200, 255, 200) if self.input_mode == 'tilt' else (255, 255, 255)
        border_color = (0, 200, 0) if self.input_mode == 'tilt' else (0, 0, 0)
        cv2.rectangle(panel, (10, button_y), (280, button_y + 30), field_color, -1)
        cv2.rectangle(panel, (10, button_y), (280, button_y + 30), border_color, 2)
        if self.input_mode == 'tilt':
            cv2.putText(panel, self.input_text, (15, button_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        button_y += 50
        
        # Status
        cv2.putText(panel, f"Pan: {int(round(self.current_pan_angle))}", (10, button_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        button_y += 25
        cv2.putText(panel, f"Tilt: {int(round(self.current_tilt_angle))}", (10, button_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return panel
    
    def handle_control_panel_click(self, x, y):
        """Handle mouse click on control panel"""
        button_y = 60
        button_height = 40
        button_spacing = 50
        
        # PAN - button
        if 10 <= x <= 140 and button_y <= y <= button_y + button_height:
            self.handle_joystick_move('pan', -1.0)
            return
        
        # PAN + button
        if 150 <= x <= 280 and button_y <= y <= button_y + button_height:
            self.handle_joystick_move('pan', 1.0)
            return
        
        button_y += button_spacing
        
        # TILT - button
        if 10 <= x <= 140 and button_y <= y <= button_y + button_height:
            self.handle_joystick_move('tilt', -1.0)
            return
        
        # TILT + button
        if 150 <= x <= 280 and button_y <= y <= button_y + button_height:
            self.handle_joystick_move('tilt', 1.0)
            return
        
        button_y += button_spacing + 10
        
        # PAN Input field
        label_y = button_y
        input_field_y = button_y + 25
        if 10 <= x <= 280:
            if (label_y - 15 <= y <= label_y + 10) or (input_field_y <= y <= input_field_y + 30):
                self.input_mode = 'pan'
                self.input_text = ""
                print("   📝 PAN input mode activated. Type numbers, then press Enter.")
                return
        
        button_y += 40
        
        # TILT Input field
        label_y = button_y
        input_field_y = button_y + 25
        if 10 <= x <= 280:
            if (label_y - 15 <= y <= label_y + 10) or (input_field_y <= y <= input_field_y + 30):
                self.input_mode = 'tilt'
                self.input_text = ""
                print("   📝 TILT input mode activated. Type numbers, then press Enter.")
                return
    
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
            # Send initial servo positions
            print(f"\n📌 Moving servos to initial positions: Pan={int(round(self.current_pan_angle))}°, Tilt={int(round(self.current_tilt_angle))}°")
            self.set_servo_angles(pan_angle=self.current_pan_angle, tilt_angle=self.current_tilt_angle)
            time.sleep(0.5)  # Reduced delay for faster startup
            print(f"✅ Servos initialized at starting positions")
        
        print("\n" + "="*70)
        print("🎯 TWO-CAMERA MANUAL ALIGNMENT SYSTEM")
        print("="*70)
        print("\n📋 Controls:")
        print("   WASD = Quick 1° movements (W=Tilt↑, S=Tilt↓, A=Pan←, D=Pan→)")
        print("   P = Move PAN by typed amount")
        print("   T = Move TILT by typed amount")
        print("   'b' = Switch target hair to BLUE")
        print("   'g' = Switch target hair to GREEN")
        print("   'q' = Quit")
        print("="*70)
        
        window_name = "Two-Camera Manual Alignment"
        cv2.namedWindow(window_name)
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                # Check if click is in control panel area
                if self.side_cap is not None:
                    control_panel_start_x = self.camera_frame_width * 2
                else:
                    control_panel_start_x = self.camera_frame_width
                
                if x >= control_panel_start_x:
                    panel_x = x - control_panel_start_x
                    self.handle_control_panel_click(panel_x, y)
        
        cv2.setMouseCallback(window_name, mouse_callback)
        
        while True:
            # Frame rate limiting (reduced for faster response)
            current_time = time.time()
            elapsed = current_time - self.last_frame_time
            if elapsed < self.frame_interval:
                # Only sleep if we're running too fast (don't block unnecessarily)
                sleep_time = self.frame_interval - elapsed
                if sleep_time > 0.001:  # Only sleep if more than 1ms
                    time.sleep(sleep_time)
            self.last_frame_time = time.time()
            
            # Read front camera
            ret_front, frame_front = self.front_cap.read()
            if not ret_front or frame_front is None:
                print("❌ Failed to read from front camera")
                break
            
            processed_front = self.process_frame(frame_front)
            if processed_front is None:
                continue
            
            # Detect lines
            needle_line_front = self.detect_line(processed_front, 'yellow', 'front')
            hair_line_front = self.detect_line(processed_front, self.target_hair_color, 'front')
            
            angle_info_front = None
            if needle_line_front and hair_line_front:
                angle_info_front = self.calculate_angle_difference(needle_line_front, hair_line_front)
            
            # Update locked display if movement detected or first detection
            current_time = time.time()
            time_since_movement = current_time - self.last_movement_time
            
            # Update locked lines if:
            # 1. First detection (locked lines are None)
            # 2. Movement just happened (within cooldown period)
            if (self.locked_front_needle is None or time_since_movement < self.movement_cooldown):
                if needle_line_front and hair_line_front:
                    self.locked_front_needle = dict(needle_line_front)  # Create new dict
                    self.locked_front_hair = dict(hair_line_front)  # Create new dict
                    self.locked_front_angle_info = dict(angle_info_front) if angle_info_front else None
            
            # Use locked lines for display
            display_needle_front = self.locked_front_needle if self.locked_front_needle else needle_line_front
            display_hair_front = self.locked_front_hair if self.locked_front_hair else hair_line_front
            display_angle_info_front = self.locked_front_angle_info if self.locked_front_angle_info else angle_info_front
            
            # Draw front camera view
            result_front = self.draw_detection_results(processed_front, display_needle_front, display_hair_front, display_angle_info_front, 'front')
            
            # Read and process side camera if available
            result_side = None
            if self.side_cap is not None:
                ret_side, frame_side = self.side_cap.read()
                if ret_side and frame_side is not None:
                    processed_side = self.process_frame(frame_side)
                    if processed_side is not None:
                        # Detect lines
                        needle_line_side = self.detect_line(processed_side, 'yellow', 'side')
                        hair_line_side = self.detect_line(processed_side, self.target_hair_color, 'side')
                        
                        angle_info_side = None
                        if needle_line_side and hair_line_side:
                            angle_info_side = self.calculate_angle_difference(needle_line_side, hair_line_side)
                        
                        # Update locked display if movement detected or first detection
                        time_since_movement = current_time - self.last_movement_time
                        
                        # Update locked lines if:
                        # 1. First detection (locked lines are None)
                        # 2. Movement just happened (within cooldown period)
                        if (self.locked_side_needle is None or time_since_movement < self.movement_cooldown):
                            if needle_line_side and hair_line_side:
                                self.locked_side_needle = dict(needle_line_side)  # Create new dict
                                self.locked_side_hair = dict(hair_line_side)  # Create new dict
                                self.locked_side_angle_info = dict(angle_info_side) if angle_info_side else None
                        
                        # Use locked lines for display
                        display_needle_side = self.locked_side_needle if self.locked_side_needle else needle_line_side
                        display_hair_side = self.locked_side_hair if self.locked_side_hair else hair_line_side
                        display_angle_info_side = self.locked_side_angle_info if self.locked_side_angle_info else angle_info_side
                        
                        result_side = self.draw_detection_results(processed_side, display_needle_side, display_hair_side, display_angle_info_side, 'side')
            
            # Combine views
            if result_side is not None:
                combined = np.hstack([result_front, result_side])
            else:
                combined = result_front
            
            # Add control panel
            H, W = combined.shape[:2]
            control_panel = self.create_control_panel(target_height=H)
            if control_panel.shape[0] != H:
                control_panel = cv2.resize(control_panel, (self.control_panel_width, H))
            combined = np.hstack([combined, control_panel])
            
            cv2.imshow(window_name, combined)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == 0 or key == 255:  # No key pressed
                pass
            elif key == ord('q'):
                break
            # If in input mode, handle input first (except P/T which activate input mode)
            elif self.input_mode:
                # Don't allow P/T to reset input mode while typing
                if key == ord('p') or key == ord('P') or key == ord('t') or key == ord('T'):
                    # Ignore P/T keys while in input mode
                    pass
                else:
                    self.handle_input_key(key)
            # Check for P/T keys to activate input mode (only when not already in input mode)
            elif key == ord('p') or key == ord('P'):
                self.input_mode = 'pan'
                self.input_text = ""
                print("   📝 PAN input mode activated. Type numbers, then press Enter.")
            elif key == ord('t') or key == ord('T'):
                self.input_mode = 'tilt'
                self.input_text = ""
                print("   📝 TILT input mode activated. Type numbers, then press Enter.")
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
        
        # Cleanup
        self.free_servos()  # Free servos before closing connection
        if self.front_cap:
            self.front_cap.release()
        if self.side_cap:
            self.side_cap.release()
        if self.arduino_conn:
            self.arduino_conn.close()
        cv2.destroyAllWindows()
        
        print(f"\n✅ System shutdown complete.")


if __name__ == "__main__":
    system = TwoCameraManualAlignment(front_camera_id=0, side_camera_id=1)
    try:
        system.run()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        system.free_servos()  # Ensure servos are freed even on interrupt
        if system.front_cap:
            system.front_cap.release()
        if system.side_cap:
            system.side_cap.release()
        if system.arduino_conn:
            system.arduino_conn.close()
        cv2.destroyAllWindows()
        print("✅ Servos freed and system shutdown complete.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        system.free_servos()  # Ensure servos are freed even on error
        if system.front_cap:
            system.front_cap.release()
        if system.side_cap:
            system.side_cap.release()
        if system.arduino_conn:
            system.arduino_conn.close()
        cv2.destroyAllWindows()
        print("✅ Servos freed and system shutdown complete.")

