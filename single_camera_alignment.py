#!/usr/bin/env python3
"""
Single Camera Alignment System
===============================

Simplified version that uses only the side camera for alignment.
Perfect for testing and debugging before moving to two-stage alignment.

Features:
- Uses only side camera
- Detects yellow needle and blue/green hair lines
- Aligns angles using pan/tilt servos
- All safety features: boundary protection, adaptive direction, smart servo switching
"""

import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports
import json
import os
from collections import deque

# Angle history log for display
ANGLE_LOG_MAX_LEN = 10


class SingleCameraAlignment:
    """
    Single camera alignment system for needle positioning.
    
    Uses only the side camera to align yellow needle with blue/green hair lines.
    """
    
    def __init__(self, camera_id=1, arduino_port=None):
        """
        Initialize the alignment system.
        
        Args:
            camera_id: Camera index for side camera (default: 1)
            arduino_port: Serial port for Arduino/RP2040 (None = auto-detect or simulation)
        """
        # Camera setup
        self.camera_id = camera_id
        self.cap = None
        
        # Servo control
        self.arduino_port = arduino_port
        self.arduino_conn = None
        self.current_pan_angle = 90.0  # degrees (0-180, 90 = center)
        self.current_tilt_angle = 90.0  # degrees (0-180, 90 = center)
        
        # Target hair color
        self.target_hair_color = 'blue'  # Which hair to align with ('blue' or 'green')
        
        # Load calibrated HSV ranges
        self.colors = self._load_color_ranges('side_camera_adjustments.json')
        
        # Line detection parameters
        self.min_area = 50
        self.angle_threshold = 30
        
        # Angle stabilization
        self.stable_angles = {'yellow': None, 'blue': None, 'green': None}
        self.angle_history = {
            'yellow': deque(maxlen=20),
            'blue': deque(maxlen=20),
            'green': deque(maxlen=20)
        }
        
        # Control parameters
        self.angle_tolerance = 2.0  # degrees - consider aligned if error < this
        self.pan_direction = 1  # 1 = normal, -1 = reversed
        self.tilt_direction = 1  # 1 = normal, -1 = reversed
        
        # Proportional controller gains
        self.Kp_pan = 0.5
        self.Kp_tilt = 0.5
        
        # Movement limits
        self.max_movement_step = 5.0
        self.movement_scale = 0.5
        self.initial_movement_scale = 0.1  # Very conservative for first few movements (10%)
        self.initial_movements_count = 0
        self.initial_movements_threshold = 5
        
        # Servo timing
        self.servo_settle_time = 2.0  # Increased from 1.5 to allow servos to fully settle
        self.calibration_settle_time = 3.0  # Even longer for calibration measurements
        self.angle_measurement_samples = 5  # Number of frames to average for stable angle measurement
        
        # Learning-based calibration
        self.calibration_done = False
        self.pan_sensitivity = None  # kA: angle_change per degree pan movement
        self.tilt_sensitivity = None  # kB: angle_change per degree tilt movement
        self.pan_position_sensitivity = None  # pA: (dx_per_deg, dy_per_deg) position shift per degree pan
        self.tilt_position_sensitivity = None  # pB: (dx_per_deg, dy_per_deg) position shift per degree tilt
        self.calibration_test_delta = 3.0  # degrees - test movement size (reduced from 5.0)
        self.min_servo_sensitivity = 0.1  # minimum |k| to consider servo useful
        self.primary_servo = None  # 'pan' or 'tilt' - chosen based on calibration
        self.stagnation_counter = 0  # Track if error is not changing
        self.stagnation_threshold = 5  # Switch servo after this many stagnant iterations
        
        # Adaptive direction correction
        self.adaptive_direction = True
        self.prev_angle_error = None
        self.prev_needle_angle = None
        self.prev_pan_adjustment = 0.0
        self.prev_tilt_adjustment = 0.0
        self.pan_direction_corrected = False
        self.tilt_direction_corrected = False
        self.no_change_count = 0
        self.pan_no_change_count = 0
        self.tilt_no_change_count = 0
        self.use_pan_only = False
        self.use_tilt_only = False
        
        # Boundary checking
        self.boundary_margin = 30
        self.frame_width = 400
        self.frame_height = 400
        self.enable_boundary_check = True
        
        # Recovery tracking
        self.last_needle_position = None
        self.missing_count = 0
        self.recovery_search_step = 0
        self.recovery_search_size = 3.0
        
        # Timing
        self.last_control_update = 0.0
        self.control_update_interval = 0.5
        # servo_settle_time is defined above in calibration section
        
        # Statistics
        self.iterations = 0
        self.is_aligned = False
        
        # Angle history log for display
        self.angle_log = []  # list of dicts with movement history
        self.step_idx = 0  # Step counter for angle log
        self.movement_delay = 0.3  # Delay after servo movement to let camera catch up (seconds)
        
        # Manual control mode
        self.manual_mode = False
        self.waiting_for_manual_input = False
        self.joystick_step_size = 1.0  # Degrees per joystick movement
        self.last_joystick_move_time = 0
        self.joystick_repeat_delay = 0.1  # Seconds between repeated movements
        
        # Visual joystick GUI
        self.control_panel_open = False
        self.input_text = ""  # Text being typed for degree input
        self.input_mode = None  # 'pan' or 'tilt' when typing degrees
        self.control_panel_width = 300
        self.control_panel_height = 500  # Increased to fit the Calculate Angle button
        self.table_panel_width = 420  # Store table panel width for click detection
        self.camera_frame_width = 0  # Will be updated each frame
    
    def _load_color_ranges(self, filename):
        """Load HSV color ranges from JSON file"""
        default_colors = {
            'yellow': {'lower': [5, 11, 111], 'upper': [56, 166, 254]},
            'blue': {'lower': [69, 162, 87], 'upper': [147, 195, 254]},
            'green': {'lower': [41, 73, 113], 'upper': [96, 245, 255]}
        }
        
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    if 'final_colors' in data:
                        colors = {}
                        for color_name, color_data in data['final_colors'].items():
                            colors[color_name] = {
                                'lower': color_data['lower'],
                                'upper': color_data['upper']
                            }
                        print(f"✅ Loaded color ranges from {filename}")
                        return colors
            except Exception as e:
                print(f"⚠️  Error loading {filename}: {e}, using defaults")
        
        print(f"⚠️  Using default color ranges (file {filename} not found)")
        return default_colors
    
    def initialize_camera(self):
        """Initialize side camera"""
        # Try to open camera
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"❌ Could not open camera {self.camera_id}")
            print(f"   Trying camera 0 instead...")
            # Try camera 0 as fallback
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print(f"❌ Could not open camera 0 either")
                print(f"   Please check camera connections")
                return False
            else:
                self.camera_id = 0
                print(f"✅ Using camera 0 instead")
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Test if we can actually read a frame
        ret, test_frame = self.cap.read()
        if not ret or test_frame is None:
            print(f"❌ Camera {self.camera_id} opened but cannot read frames")
            self.cap.release()
            return False
        
        print(f"✅ Camera {self.camera_id} initialized and working")
        return True
    
    def find_arduino_port(self):
        """Auto-detect Arduino/RP2040 serial port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            description = port.description.lower()
            device = port.device.lower()
            if ('arduino' in description or 'pico' in description or 'rp2040' in description or
                'usb serial' in description or 'usbmodem' in device or 'ttyacm' in device):
                print(f"🔍 Found potential Arduino port: {port.device}")
                return port.device
        return None
    
    def initialize_arduino(self):
        """Initialize serial connection to Arduino/RP2040"""
        if self.arduino_port is None:
            self.arduino_port = self.find_arduino_port()
            if self.arduino_port is None:
                print("⚠️  Could not auto-detect Arduino port")
                print("   Continuing in simulation mode")
                self.arduino_conn = None
                return False
        
        try:
            self.arduino_conn = serial.Serial(
                port=self.arduino_port,
                baudrate=9600,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)
            print(f"✅ Arduino connected on {self.arduino_port}")
            
            self.arduino_conn.reset_input_buffer()
            
            try:
                self.arduino_conn.write(b"STOP\n")
                time.sleep(0.2)
                self.arduino_conn.reset_input_buffer()
                print("   Servos disabled (STOP command sent)")
            except:
                pass
            
            return True
        except Exception as e:
            print(f"⚠️  Could not connect to Arduino: {e}")
            print("   Continuing in simulation mode")
            self.arduino_conn = None
            return False
    
    def set_servo_angles(self, pan_angle=None, tilt_angle=None):
        """Set pan/tilt servo angles"""
        if pan_angle is not None:
            pan_angle = max(0, min(180, int(pan_angle)))
            self.current_pan_angle = pan_angle
        
        if tilt_angle is not None:
            tilt_angle = max(0, min(180, int(tilt_angle)))
            self.current_tilt_angle = tilt_angle
        
        if self.arduino_conn is None:
            if pan_angle is not None and tilt_angle is not None:
                print(f"   [SIMULATION] Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
            elif pan_angle is not None:
                print(f"   [SIMULATION] Pan={pan_angle:.1f}°")
            elif tilt_angle is not None:
                print(f"   [SIMULATION] Tilt={tilt_angle:.1f}°")
            return True
        
        try:
            commands = []
            if pan_angle is not None:
                commands.append(f"P{pan_angle}\n")
            if tilt_angle is not None:
                commands.append(f"T{tilt_angle}\n")
            
            for cmd in commands:
                self.arduino_conn.write(cmd.encode())
                time.sleep(0.05)
            
            time.sleep(0.1)
            return True
        except Exception as e:
            print(f"❌ Error sending servo command: {e}")
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
    
    def round_to_half_degree(self, angle):
        """
        Round angle to nearest 0.5 degrees for stable display.
        
        Args:
            angle: Angle in degrees
        
        Returns:
            Angle rounded to nearest 0.5 degrees
        """
        return round(angle * 2) / 2.0
    
    def stabilize_angle(self, color_name, new_angle):
        """Stabilize angle using median filtering with larger buffer for smoother display"""
        if new_angle is None:
            return self.stable_angles[color_name]
        
        # Add to history
        self.angle_history[color_name].append(new_angle)
        
        # Keep only last 10 measurements for smoothing
        if len(self.angle_history[color_name]) > 10:
            self.angle_history[color_name].popleft()
        
        # If we don't have a stable angle yet, use the new one
        if self.stable_angles[color_name] is None:
            self.stable_angles[color_name] = new_angle
            return new_angle
        
        # Calculate median of recent measurements for stability
        if len(self.angle_history[color_name]) >= 3:
            angles = list(self.angle_history[color_name])
            smoothed_angle = np.median(angles)
            
            # Only update if the change is significant (larger threshold for display stability)
            angle_diff = abs(smoothed_angle - self.stable_angles[color_name])
            if angle_diff >= 0.5:  # Only update if change is at least 0.5 degrees
                self.stable_angles[color_name] = smoothed_angle
        else:
            # Not enough history yet, use weighted average
            angles = list(self.angle_history[color_name])
            weights = np.linspace(0.3, 1.0, len(angles))
            smoothed_angle = np.average(angles, weights=weights)
            angle_diff = abs(smoothed_angle - self.stable_angles[color_name])
            if angle_diff >= 0.5:
                self.stable_angles[color_name] = smoothed_angle
        
        return self.stable_angles[color_name]
    
    def detect_line(self, frame, color_name):
        """Detect a line of a specific color in the frame"""
        if frame is None:
            return None
        
        if color_name not in self.colors:
            return None
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.colors[color_name]['lower'])
        upper = np.array(self.colors[color_name]['upper'])
        mask = cv2.inRange(hsv, lower, upper)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.min_area:
            return None
        
        points = largest_contour.reshape(-1, 2)
        if len(points) < 2:
            return None
        
        # Use cv2.fitLine for more robust and accurate line fitting
        # This gives us a direction vector [vx, vy, x0, y0]
        # where (x0, y0) is a point on the line and (vx, vy) is the direction vector
        line_params = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        # Extract scalar values from numpy arrays (handle both array and scalar types)
        vx = float(line_params[0].item() if hasattr(line_params[0], 'item') else line_params[0])
        vy = float(line_params[1].item() if hasattr(line_params[1], 'item') else line_params[1])
        x0 = float(line_params[2].item() if hasattr(line_params[2], 'item') else line_params[2])
        y0 = float(line_params[3].item() if hasattr(line_params[3], 'item') else line_params[3])
        
        # CRITICAL: Normalize direction vector to ensure parallel lines get the same angle
        # cv2.fitLine can return (vx, vy) or (-vx, -vy) for the same line
        # Strategy: Always normalize to a consistent direction
        # We'll use a "canonical" direction: prefer pointing right (vx > 0), 
        # or if vertical, prefer pointing down (vy > 0)
        
        # Normalize the direction vector to a canonical form
        # First, if vx < 0, flip the vector (point it the other way)
        if vx < 0:
            vx = -vx
            vy = -vy
        elif abs(vx) < 1e-6:  # Nearly vertical line (vx ≈ 0)
            # For vertical lines, prefer pointing down (vy > 0)
            if vy < 0:
                vy = -vy
        
        # Now calculate angle from normalized direction vector
        # Use arctan2(vy, vx) to get angle in degrees
        # Note: In image coordinates, y increases downward, but arctan2 handles this correctly
        raw_angle = np.arctan2(vy, vx) * 180 / np.pi
        
        # Normalize to [0, 180) range
        # Lines are undirected, so θ and θ+180° represent the same line
        # This ensures parallel lines get the same angle
        while raw_angle < 0:
            raw_angle += 180
        while raw_angle >= 180:
            raw_angle -= 180
        
        # Calculate endpoints for visualization
        # Use the actual contour points to find the endpoints of the detected line
        # Project all contour points onto the fitted line and find the extremes
        t_values = []
        for pt in points:
            # Project point onto line: t = (pt - point_on_line) · direction_vector
            # Since direction vector is unit length, t = (pt_x - x0) * vx + (pt_y - y0) * vy
            t = (pt[0] - x0) * vx + (pt[1] - y0) * vy
            t_values.append(t)
        
        if t_values:
            t_min = min(t_values)
            t_max = max(t_values)
        else:
            # Fallback: use bounding box if projection fails
            x_coords = points[:, 0]
            y_coords = points[:, 1]
            min_x, max_x = int(x_coords.min()), int(x_coords.max())
            min_y, max_y = int(y_coords.min()), int(y_coords.max())
            
            t_values = []
            if abs(vx) > 1e-6:
                t_values.append((min_x - x0) / vx)
                t_values.append((max_x - x0) / vx)
            if abs(vy) > 1e-6:
                t_values.append((min_y - y0) / vy)
                t_values.append((max_y - y0) / vy)
            
            if t_values:
                t_min = min(t_values)
                t_max = max(t_values)
            else:
                t_min, t_max = -50, 50  # Small fallback
        
        # Calculate actual endpoints using the projected points
        point1 = (int(x0 + t_min * vx), int(y0 + t_min * vy))
        point2 = (int(x0 + t_max * vx), int(y0 + t_max * vy))
        
        # Ensure point1 is left of point2 (or above if vertical) for consistency
        if point1[0] > point2[0] or (point1[0] == point2[0] and point1[1] > point2[1]):
            point1, point2 = point2, point1
        
        # Use raw angle for display (more accurate, no drift)
        # Only stabilize for control logic if needed
        display_angle = raw_angle  # Use raw angle for display to avoid stabilization drift
        stable_angle = self.stabilize_angle(color_name, raw_angle)  # Keep stabilized for control
        
        center_x = (point1[0] + point2[0]) // 2
        center_y = (point1[1] + point2[1]) // 2
        
        return {
            'point1': point1,
            'point2': point2,
            'center': (center_x, center_y),
            'angle': display_angle,  # Use raw angle for display
            'stable_angle': stable_angle,  # Keep stabilized version for control
            'area': area,
            'mask': mask
        }
    
    def calculate_angle_difference(self, needle_line, hair_line):
        """Calculate angle difference between needle and hair lines"""
        if needle_line is None or hair_line is None:
            return None
        
        # Use display angles (raw angles) for calculation
        needle_angle = needle_line['angle']
        hair_angle = hair_line['angle']
        
        # Both angles should already be normalized to [0, 180) in detect_line
        # Calculate the difference, handling wrap-around
        raw_diff = needle_angle - hair_angle
        abs_diff = abs(raw_diff)
        
        # If the difference is > 90°, the lines might be parallel but measured in opposite directions
        # The actual difference should be the smaller angle between them
        if abs_diff > 90:
            # Use the complementary angle (180 - abs_diff)
            angle_diff = 180 - abs_diff
            # Preserve the sign based on which direction is shorter
            if raw_diff > 0:
                angle_diff = -angle_diff
        else:
            angle_diff = raw_diff
        
        # Include stabilized angles if available
        needle_stable = needle_line.get('stable_angle', needle_angle)
        hair_stable = hair_line.get('stable_angle', hair_angle)
        
        return {
            'needle_angle': needle_angle,
            'hair_angle': hair_angle,
            'needle_stable_angle': needle_stable,
            'hair_stable_angle': hair_stable,
            'angle_diff': angle_diff
        }
    
    def check_boundary_violation(self, needle_line):
        """Check if yellow line is too close to frame edges"""
        if not self.enable_boundary_check or needle_line is None:
            return {
                'near_left': False, 'near_right': False,
                'near_top': False, 'near_bottom': False,
                'allow_pan_positive': True, 'allow_pan_negative': True,
                'allow_tilt_positive': True, 'allow_tilt_negative': True
            }
        
        center = needle_line['center']
        point1 = needle_line.get('point1', center)
        point2 = needle_line.get('point2', center)
        
        min_x = min(point1[0], point2[0], center[0])
        max_x = max(point1[0], point2[0], center[0])
        min_y = min(point1[1], point2[1], center[1])
        max_y = max(point1[1], point2[1], center[1])
        
        near_left = min_x < self.boundary_margin
        near_right = max_x > (self.frame_width - self.boundary_margin)
        near_top = min_y < self.boundary_margin
        near_bottom = max_y > (self.frame_height - self.boundary_margin)
        
        allow_pan_positive = not near_right
        allow_pan_negative = not near_left
        allow_tilt_positive = not near_bottom
        allow_tilt_negative = not near_top
        
        return {
            'near_left': near_left, 'near_right': near_right,
            'near_top': near_top, 'near_bottom': near_bottom,
            'allow_pan_positive': allow_pan_positive,
            'allow_pan_negative': allow_pan_negative,
            'allow_tilt_positive': allow_tilt_positive,
            'allow_tilt_negative': allow_tilt_negative
        }
    
    def measure_stable_angle(self, num_samples=None):
        """
        Measure angle multiple times and return the median for stability.
        
        Args:
            num_samples: Number of frames to sample (default: self.angle_measurement_samples)
        
        Returns:
            angle: Median angle from multiple measurements, or None if failed
        """
        if num_samples is None:
            num_samples = self.angle_measurement_samples
        
        angles = []
        for i in range(num_samples):
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            processed = self.process_frame(frame)
            if processed is None:
                continue
            
            needle_line = self.detect_line(processed, 'yellow')
            if needle_line is None:
                continue
            
            angles.append(needle_line['angle'])
            time.sleep(0.1)  # Small delay between samples
        
        if len(angles) < num_samples // 2:  # Need at least half the samples
            return None
        
        # Return median for robustness against outliers
        return np.median(angles)
    
    def measure_stable_angle_and_position(self, num_samples=None):
        """
        Measure both angle and position (midpoint) multiple times and return medians.
        
        Args:
            num_samples: Number of frames to sample (default: self.angle_measurement_samples)
        
        Returns:
            (angle, midpoint): (median angle, median midpoint) or (None, None) if failed
        """
        if num_samples is None:
            num_samples = self.angle_measurement_samples
        
        angles = []
        midpoints_x = []
        midpoints_y = []
        
        for i in range(num_samples):
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            processed = self.process_frame(frame)
            if processed is None:
                continue
            
            needle_line = self.detect_line(processed, 'yellow')
            if needle_line is None:
                continue
            
            angles.append(needle_line['angle'])
            center = needle_line['center']
            # Handle both tuple and numpy array types for center
            if isinstance(center, (tuple, list)):
                midpoints_x.append(float(center[0]))
                midpoints_y.append(float(center[1]))
            else:
                midpoints_x.append(float(center[0].item() if hasattr(center[0], 'item') else center[0]))
                midpoints_y.append(float(center[1].item() if hasattr(center[1], 'item') else center[1]))
            time.sleep(0.1)  # Small delay between samples
        
        if len(angles) < num_samples // 2:  # Need at least half the samples
            return None, None
        
        # Return medians for robustness
        angle = np.median(angles)
        midpoint = (np.median(midpoints_x), np.median(midpoints_y))
        return angle, midpoint
    
    def test_servo_effect(self, servo_type, initial_angle, initial_midpoint, test_delta):
        """
        Test how moving a servo affects both the needle angle and position.
        
        Args:
            servo_type: 'pan' or 'tilt'
            initial_angle: Current needle angle before movement
            initial_midpoint: Current needle midpoint (x, y) before movement
            test_delta: How many degrees to move servo (e.g., +3.0)
        
        Returns:
            (k_estimated, p_estimated): 
                k_estimated: angle sensitivity (angle_change / servo_movement)
                p_estimated: position sensitivity ((dx_per_deg, dy_per_deg))
            or (None, None) if test failed
        """
        print(f"   Testing {servo_type.upper()} (move +{test_delta:.1f}°)...")
        
        # Get current servo position
        if servo_type == 'pan':
            current_pos = self.current_pan_angle
        else:
            current_pos = self.current_tilt_angle
        
        # Move servo
        new_pos = current_pos + test_delta
        new_pos = max(0, min(180, new_pos))
        
        print(f"      Moving servo from {current_pos:.1f}° to {new_pos:.1f}°...")
        if servo_type == 'pan':
            self.set_servo_angles(pan_angle=new_pos)
        else:
            self.set_servo_angles(tilt_angle=new_pos)
        
        # Wait longer for calibration to ensure servo fully settles
        print(f"      Waiting {self.calibration_settle_time:.1f}s for servo to settle...")
        time.sleep(self.calibration_settle_time)
        
        # Measure angle and position after movement (using multiple samples for stability)
        print(f"      Measuring angle and position ({self.angle_measurement_samples} samples)...")
        angle_after, midpoint_after = self.measure_stable_angle_and_position()
        
        if angle_after is None or midpoint_after is None:
            print(f"      ⚠️  Could not measure angle/position after {servo_type} movement")
            # Return servo to original position
            if servo_type == 'pan':
                self.set_servo_angles(pan_angle=current_pos)
            else:
                self.set_servo_angles(tilt_angle=current_pos)
            time.sleep(self.calibration_settle_time)
            return None, None
        
        # Calculate angle sensitivity
        angle_change = angle_after - initial_angle
        # Normalize angle change (handle wrap-around)
        if angle_change > 90:
            angle_change -= 180
        elif angle_change < -90:
            angle_change += 180
        
        actual_movement = new_pos - current_pos
        if abs(actual_movement) < 0.1:
            k_estimated = None
            print(f"      ⚠️  {servo_type.upper()} movement too small to measure")
        else:
            k_estimated = angle_change / actual_movement
            print(f"      📊 Angle change: {angle_change:.3f}° (from {initial_angle:.2f}° to {angle_after:.2f}°)")
            print(f"      📊 Servo movement: {actual_movement:.2f}°")
        
        # Calculate position sensitivity
        dx = midpoint_after[0] - initial_midpoint[0]
        dy = midpoint_after[1] - initial_midpoint[1]
        if abs(actual_movement) >= 0.1:
            dx_per_deg = dx / actual_movement
            dy_per_deg = dy / actual_movement
            p_estimated = (dx_per_deg, dy_per_deg)
            print(f"      📊 Position change: ({dx:.1f}, {dy:.1f}) px")
            print(f"      📊 Position sensitivity: ({dx_per_deg:.2f}, {dy_per_deg:.2f}) px/°")
        else:
            p_estimated = (0.0, 0.0)
        
        # Return servo to original position
        print(f"      Returning servo to original position ({current_pos:.1f}°)...")
        if servo_type == 'pan':
            self.set_servo_angles(pan_angle=current_pos)
        else:
            self.set_servo_angles(tilt_angle=current_pos)
        time.sleep(self.calibration_settle_time)
        
        if k_estimated is not None:
            print(f"      ✅ {servo_type.upper()} k={k_estimated:.3f}° angle/° servo, p=({p_estimated[0]:.2f}, {p_estimated[1]:.2f}) px/°")
        else:
            print(f"      ⚠️  {servo_type.upper()} sensitivity: Could not calculate")
        
        return k_estimated, p_estimated
    
    def calibrate_servos(self):
        """
        Calibrate both servos to learn their effect on needle angle.
        
        Returns:
            (pan_sensitivity, tilt_sensitivity): Sensitivities for both servos
        """
        print("\n" + "="*70)
        print("🔧 SERVO CALIBRATION: Learning how each servo affects angle")
        print("="*70)
        
        # Verify camera is working and needle is visible
        ret, frame = self.cap.read()
        if not ret:
            print("❌ Cannot calibrate: Failed to read from camera")
            return False
        
        processed = self.process_frame(frame)
        if processed is None:
            print("❌ Cannot calibrate: Failed to process frame")
            return False
        
        needle_line = self.detect_line(processed, 'yellow')
        if needle_line is None:
            print("❌ Cannot calibrate: Yellow needle not detected")
            print("   Please ensure yellow needle is visible in camera")
            return False
        
        # Measure initial angle and position multiple times for stability
        print(f"   Measuring initial angle and position ({self.angle_measurement_samples} samples)...")
        angle_initial, midpoint_initial = self.measure_stable_angle_and_position()
        if angle_initial is None or midpoint_initial is None:
            print("❌ Cannot calibrate: Failed to measure stable initial angle/position")
            return False
        
        print(f"   Initial needle angle (stable): {angle_initial:.2f}°")
        print(f"   Initial needle midpoint: ({midpoint_initial[0]:.1f}, {midpoint_initial[1]:.1f}) px")
        print(f"   Current pan: {self.current_pan_angle:.1f}°, tilt: {self.current_tilt_angle:.1f}°")
        
        # Test Pan servo (measures both angle and position sensitivity)
        self.pan_sensitivity, self.pan_position_sensitivity = self.test_servo_effect(
            'pan', angle_initial, midpoint_initial, self.calibration_test_delta)
        
        # Test Tilt servo (measures both angle and position sensitivity)
        self.tilt_sensitivity, self.tilt_position_sensitivity = self.test_servo_effect(
            'tilt', angle_initial, midpoint_initial, self.calibration_test_delta)
        
        # Validate calibration
        pan_effective = self.pan_sensitivity is not None and abs(self.pan_sensitivity) >= self.min_servo_sensitivity
        tilt_effective = self.tilt_sensitivity is not None and abs(self.tilt_sensitivity) >= self.min_servo_sensitivity
        
        print(f"\n📊 Calibration Results:")
        if self.pan_sensitivity is not None:
            p_str = f"p=({self.pan_position_sensitivity[0]:.2f}, {self.pan_position_sensitivity[1]:.2f})" if self.pan_position_sensitivity else "p=(?, ?)"
            print(f"   Pan: k={self.pan_sensitivity:.4f} {p_str} {'✅ Effective' if pan_effective else '⚠️  Too weak'}")
        else:
            print(f"   Pan: ❌ Failed to measure")
        
        if self.tilt_sensitivity is not None:
            p_str = f"p=({self.tilt_position_sensitivity[0]:.2f}, {self.tilt_position_sensitivity[1]:.2f})" if self.tilt_position_sensitivity else "p=(?, ?)"
            print(f"   Tilt: k={self.tilt_sensitivity:.4f} {p_str} {'✅ Effective' if tilt_effective else '⚠️  Too weak'}")
        else:
            print(f"   Tilt: ❌ Failed to measure")
        
        if not pan_effective and not tilt_effective:
            print("\n⚠️  Calibration warning: Neither servo shows significant angle change")
            print("   Possible reasons:")
            print("   - Servos may control position, not angle directly")
            print("   - Needle mounting may prevent angle changes from pan/tilt")
            print("   - Servos may need larger movements to affect angle")
            print("\n   🔄 Falling back to standard control mode (no learned sensitivities)")
            print("   System will still work, but may be less efficient")
            # Set default sensitivities to allow system to work
            self.pan_sensitivity = 1.0  # Default: assume 1:1 relationship
            self.tilt_sensitivity = 1.0
            self.primary_servo = 'pan'  # Default to pan
            self.calibration_done = True  # Mark as done so system proceeds
            return True  # Allow system to continue
        
        # Choose primary servo
        if not pan_effective:
            self.primary_servo = 'tilt'
            print(f"\n✅ Primary servo: TILT (pan not effective)")
        elif not tilt_effective:
            self.primary_servo = 'pan'
            print(f"\n✅ Primary servo: PAN (tilt not effective)")
        else:
            # Both effective - choose the one with larger sensitivity
            abs_pan_k = abs(self.pan_sensitivity)
            abs_tilt_k = abs(self.tilt_sensitivity)
            
            if abs_pan_k > abs_tilt_k * 1.5:
                self.primary_servo = 'pan'
                print(f"\n✅ Primary servo: PAN (more effective: {abs_pan_k:.3f} vs {abs_tilt_k:.3f})")
            elif abs_tilt_k > abs_pan_k * 1.5:
                self.primary_servo = 'tilt'
                print(f"\n✅ Primary servo: TILT (more effective: {abs_tilt_k:.3f} vs {abs_pan_k:.3f})")
            else:
                # Similar effectiveness - use pan as default
                self.primary_servo = 'pan'
                print(f"\n✅ Primary servo: PAN (similar effectiveness, using pan as default)")
        
        self.calibration_done = True
        print("="*70)
        return True
    
    def choose_primary_servo(self, needle_position=None):
        """
        Choose which servo to use as primary for angle control.
        
        Uses calibration data to make decision.
        """
        if not self.calibration_done:
            return None
        
        pan_effective = self.pan_sensitivity is not None and abs(self.pan_sensitivity) >= self.min_servo_sensitivity
        tilt_effective = self.tilt_sensitivity is not None and abs(self.tilt_sensitivity) >= self.min_servo_sensitivity
        
        if not pan_effective and not tilt_effective:
            return None
        
        if not pan_effective:
            return 'tilt'
        
        if not tilt_effective:
            return 'pan'
        
        # Both effective - use the one we chose during calibration
        return self.primary_servo
    
    def calculate_servo_adjustment(self, angle_error, boundary_status=None, needle_line=None):
        """
        Calculate servo angle adjustment using Proportional controller.
        
        Uses learned sensitivities from calibration.
        Includes position prediction for border safety.
        """
        # Determine which servo(s) to use
        if self.use_pan_only:
            use_pan = True
            use_tilt = False
        elif self.use_tilt_only:
            use_pan = False
            use_tilt = True
        elif self.primary_servo == 'pan' and self.pan_sensitivity is not None:
            use_pan = True
            use_tilt = False
        elif self.primary_servo == 'tilt' and self.tilt_sensitivity is not None:
            use_pan = False
            use_tilt = True
        else:
            # Fallback: use both
            use_pan = self.pan_sensitivity is not None
            use_tilt = self.tilt_sensitivity is not None
        
        # Calculate adjustments using learned sensitivities
        pan_adjustment = 0.0
        tilt_adjustment = 0.0
        
        if use_pan and self.pan_sensitivity is not None:
            # Use learned sensitivity: delta_servo = error / k
            # If sensitivity is default (1.0) or too small, use standard P control
            if abs(self.pan_sensitivity) > self.min_servo_sensitivity:
                # P controller: we want Δangle ≈ -angle_error
                # angle_change ≈ k * Δservo, so: k * Δservo ≈ -angle_error
                # => Δservo ≈ -angle_error / k
                pan_adjustment = (-angle_error / self.pan_sensitivity) * self.Kp_pan * self.pan_direction
            else:
                # Fallback to standard P control
                pan_adjustment = self.Kp_pan * angle_error * self.pan_direction
        
        if use_tilt and self.tilt_sensitivity is not None:
            # Use learned sensitivity
            if abs(self.tilt_sensitivity) > self.min_servo_sensitivity:
                tilt_adjustment = (-angle_error / self.tilt_sensitivity) * self.Kp_tilt * self.tilt_direction
            else:
                # Fallback to standard P control
                tilt_adjustment = self.Kp_tilt * angle_error * self.tilt_direction
        
        if boundary_status:
            if pan_adjustment > 0 and not boundary_status['allow_pan_positive']:
                pan_adjustment = 0
                print(f"   ⚠️  Boundary: Preventing pan+ (near right edge)")
            elif pan_adjustment < 0 and not boundary_status['allow_pan_negative']:
                pan_adjustment = 0
                print(f"   ⚠️  Boundary: Preventing pan- (near left edge)")
            
            if tilt_adjustment > 0 and not boundary_status['allow_tilt_positive']:
                tilt_adjustment = 0
                print(f"   ⚠️  Boundary: Preventing tilt+ (near bottom edge)")
            elif tilt_adjustment < 0 and not boundary_status['allow_tilt_negative']:
                tilt_adjustment = 0
                print(f"   ⚠️  Boundary: Preventing tilt- (near top edge)")
        
        if self.initial_movements_count < self.initial_movements_threshold:
            scale = self.initial_movement_scale
        else:
            scale = self.movement_scale
        
        pan_adjustment *= scale
        tilt_adjustment *= scale
        
        max_step = 1.0 if self.initial_movements_count < self.initial_movements_threshold else self.max_movement_step
        if abs(pan_adjustment) > max_step:
            pan_adjustment = max_step if pan_adjustment > 0 else -max_step
        if abs(tilt_adjustment) > max_step:
            tilt_adjustment = max_step if tilt_adjustment > 0 else -max_step
        
        return {
            'pan_adjustment': pan_adjustment,
            'tilt_adjustment': tilt_adjustment
        }
    
    def recover_needle_to_view(self):
        """Try to bring yellow needle back into camera view"""
        print(f"   🔍 Attempting to recover yellow needle to view...")
        
        search_patterns = [
            (0, 0),
            (-self.recovery_search_size, 0),
            (self.recovery_search_size, 0),
            (0, -self.recovery_search_size),
            (0, self.recovery_search_size),
            (-self.recovery_search_size, -self.recovery_search_size),
            (self.recovery_search_size, -self.recovery_search_size),
            (-self.recovery_search_size, self.recovery_search_size),
            (self.recovery_search_size, self.recovery_search_size),
        ]
        
        if self.recovery_search_step < len(search_patterns):
            pan_delta, tilt_delta = search_patterns[self.recovery_search_step]
            
            new_pan = max(0, min(180, self.current_pan_angle + pan_delta))
            new_tilt = max(0, min(180, self.current_tilt_angle + tilt_delta))
            
            print(f"   🔍 Recovery step {self.recovery_search_step + 1}/{len(search_patterns)}: "
                  f"Pan {self.current_pan_angle:.1f}°→{new_pan:.1f}°, "
                  f"Tilt {self.current_tilt_angle:.1f}°→{new_tilt:.1f}°")
            
            self.set_servo_angles(pan_angle=new_pan, tilt_angle=new_tilt)
            time.sleep(self.servo_settle_time)
            
            self.recovery_search_step += 1
        else:
            self.recovery_search_step = 0
            print(f"   🔍 Recovery pattern complete, restarting search...")
    
    def update_control(self, angle_info, needle_line=None):
        """Update servo control based on angle error"""
        if angle_info is None:
            return False
        
        angle_error = angle_info['angle_diff']
        abs_error = abs(angle_error)
        current_needle_angle = angle_info['needle_angle']
        
        if abs_error < self.angle_tolerance:
            if not self.is_aligned:
                print(f"✅ ALIGNED! Error: {abs_error:.2f}° < tolerance: {self.angle_tolerance}°")
                self.is_aligned = True
            return True
        
        self.is_aligned = False
        
        # Stagnation detection: check if error is not changing
        if self.prev_angle_error is not None:
            error_change = abs(angle_error - self.prev_angle_error)
            if error_change < 0.2:  # Error not changing much
                self.stagnation_counter += 1
                if self.stagnation_counter >= self.stagnation_threshold:
                    print(f"   ⚠️  Stagnation detected: Error not changing for {self.stagnation_counter} iterations")
                    # Switch to secondary servo if available
                    if self.primary_servo == 'pan' and self.tilt_sensitivity is not None and abs(self.tilt_sensitivity) > self.min_servo_sensitivity:
                        print(f"   🔄 Switching from PAN to TILT (secondary servo)")
                        self.primary_servo = 'tilt'
                        self.stagnation_counter = 0
                    elif self.primary_servo == 'tilt' and self.pan_sensitivity is not None and abs(self.pan_sensitivity) > self.min_servo_sensitivity:
                        print(f"   🔄 Switching from TILT to PAN (secondary servo)")
                        self.primary_servo = 'pan'
                        self.stagnation_counter = 0
            else:
                self.stagnation_counter = 0  # Reset if error is changing
        
        # Check if needle angle changed after previous movement
        if self.prev_needle_angle is not None:
            angle_change = abs(current_needle_angle - self.prev_needle_angle)
            if angle_change > 90:
                angle_change = 180 - angle_change
            
            if (abs(self.prev_pan_adjustment) > 0.1 or abs(self.prev_tilt_adjustment) > 0.1) and angle_change < 1.0:
                self.no_change_count += 1
                
                pan_moved = abs(self.prev_pan_adjustment) > 0.1
                tilt_moved = abs(self.prev_tilt_adjustment) > 0.1
                
                if pan_moved:
                    self.pan_no_change_count += 1
                if tilt_moved:
                    self.tilt_no_change_count += 1
                
                print(f"   ⚠️  Servos moved but angle didn't change! (change: {angle_change:.2f}°)")
                if pan_moved:
                    print(f"      Pan no-change count: {self.pan_no_change_count}")
                if tilt_moved:
                    print(f"      Tilt no-change count: {self.tilt_no_change_count}")
                
                if self.pan_no_change_count >= 2 and not self.use_tilt_only:
                    self.use_tilt_only = True
                    self.use_pan_only = False
                    print(f"   🔄 Pan not affecting angle - switching to TILT only")
                    self.pan_no_change_count = 0
                
                elif self.tilt_no_change_count >= 2 and not self.use_pan_only:
                    self.use_pan_only = True
                    self.use_tilt_only = False
                    print(f"   🔄 Tilt not affecting angle - switching to PAN only")
                    self.tilt_no_change_count = 0
            else:
                if angle_change > 1.0:
                    self.no_change_count = 0
                    self.pan_no_change_count = 0
                    self.tilt_no_change_count = 0
                    if self.use_pan_only or self.use_tilt_only:
                        print(f"   ✅ Angle responding - using both servos again")
                        self.use_pan_only = False
                        self.use_tilt_only = False
        
        # Adaptive direction correction
        if self.adaptive_direction and self.prev_angle_error is not None:
            prev_abs_error = abs(self.prev_angle_error)
            
            if abs_error > prev_abs_error + 1.0:
                print(f"   ⚠️  Error increased: {prev_abs_error:.2f}° → {abs_error:.2f}°")
                
                if abs(self.prev_pan_adjustment) > abs(self.prev_tilt_adjustment):
                    if not self.pan_direction_corrected:
                        self.pan_direction *= -1
                        self.pan_direction_corrected = True
                        print(f"   🔄 Auto-reversing PAN direction (now: {self.pan_direction})")
                elif abs(self.prev_tilt_adjustment) > 0:
                    if not self.tilt_direction_corrected:
                        self.tilt_direction *= -1
                        self.tilt_direction_corrected = True
                        print(f"   🔄 Auto-reversing TILT direction (now: {self.tilt_direction})")
        
        self.prev_angle_error = angle_error
        self.prev_needle_angle = current_needle_angle
        
        boundary_status = self.check_boundary_violation(needle_line)
        
        if boundary_status['near_left'] or boundary_status['near_right'] or \
           boundary_status['near_top'] or boundary_status['near_bottom']:
            warnings = []
            if boundary_status['near_left']:
                warnings.append("LEFT")
            if boundary_status['near_right']:
                warnings.append("RIGHT")
            if boundary_status['near_top']:
                warnings.append("TOP")
            if boundary_status['near_bottom']:
                warnings.append("BOTTOM")
            print(f"   ⚠️  Yellow line near edge: {', '.join(warnings)}")
        
        adjustment = self.calculate_servo_adjustment(angle_error, boundary_status, needle_line)
        
        self.prev_pan_adjustment = adjustment['pan_adjustment']
        self.prev_tilt_adjustment = adjustment['tilt_adjustment']
        
        if boundary_status and (boundary_status['near_left'] or boundary_status['near_right'] or 
                                boundary_status['near_top'] or boundary_status['near_bottom']):
            adjustment['pan_adjustment'] *= 0.25
            adjustment['tilt_adjustment'] *= 0.25
            print(f"   ⚠️  Near boundary - reducing movement by 75%")
        
        if self.initial_movements_count == 0:
            adjustment['pan_adjustment'] *= 0.5
            adjustment['tilt_adjustment'] *= 0.5
            print(f"   ⚠️  First movement - extra conservative (50% additional reduction)")
        
        if adjustment['pan_adjustment'] == 0 and adjustment['tilt_adjustment'] == 0:
            print(f"   ⚠️  Cannot move: Yellow line at boundary, preventing movement")
            return False
        
        new_pan = max(0, min(180, self.current_pan_angle + adjustment['pan_adjustment']))
        new_tilt = max(0, min(180, self.current_tilt_angle + adjustment['tilt_adjustment']))
        
        movement_scale_used = self.initial_movement_scale if self.initial_movements_count < self.initial_movements_threshold else self.movement_scale
        movement_type = "INITIAL (conservative)" if self.initial_movements_count < self.initial_movements_threshold else "NORMAL"
        print(f"   📐 Error: {angle_error:+.2f}° → Pan: {self.current_pan_angle:.1f}°→{new_pan:.1f}°, "
              f"Tilt: {self.current_tilt_angle:.1f}°→{new_tilt:.1f}° [{movement_type}]")
        
        # Log angle before movement
        angle_before = angle_info['needle_angle']
        
        # Determine which servo moved (for logging)
        servo_moved = None
        delta_servo = 0.0
        if abs(adjustment['pan_adjustment']) > 0.1:
            servo_moved = 'pan'
            delta_servo = adjustment['pan_adjustment']
        elif abs(adjustment['tilt_adjustment']) > 0.1:
            servo_moved = 'tilt'
            delta_servo = adjustment['tilt_adjustment']
        
        # Move servos
        self.set_servo_angles(pan_angle=new_pan, tilt_angle=new_tilt)
        self.initial_movements_count += 1
        
        # Wait for servo movement and camera to catch up
        time.sleep(self.movement_delay)
        
        # Log angle change if a servo moved
        if servo_moved:
            # Capture frame after movement to measure new angle
            ret, frame_after = self.cap.read()
            if ret:
                processed_after = self.process_frame(frame_after)
                if processed_after is not None:
                    needle_line_after = self.detect_line(processed_after, 'yellow')
                    if needle_line_after is not None:
                        angle_after = needle_line_after['angle']
                        self.log_angle_change(servo_moved, delta_servo, angle_before, angle_after)
        
        # Also wait for servo to fully settle (for control stability)
        remaining_settle = max(0, self.servo_settle_time - self.movement_delay)
        if remaining_settle > 0:
            time.sleep(remaining_settle)
        
        return False
    
    def draw_angle_table_panel(self, frame_height, panel_width=420):
        """
        Create a white panel with a simple text table showing recent angle changes.
        
        Args:
            frame_height: height of the camera frame
            panel_width: width of the panel in pixels
        
        Returns:
            panel: white image with angle history table
        """
        panel = np.ones((frame_height, panel_width, 3), dtype=np.uint8) * 255  # white
        
        # Header
        header = "Step  Srv  dServo  N_before  N_after  dAngle"
        cv2.putText(panel, header, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Rows (reverse so latest at top after header)
        row_y = 55
        for entry in reversed(self.angle_log[-ANGLE_LOG_MAX_LEN:]):
            text = f"{entry['step']:>4}  {entry['servo']}   {entry['delta_servo']:>6.1f}  " \
                   f"{entry['angle_before']:>8.1f}  {entry['angle_after']:>7.1f}  {entry['delta_angle']:>7.1f}"
            cv2.putText(panel, text, (10, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            row_y += 22
            if row_y > frame_height - 10:
                break
        
        return panel
    
    def log_angle_change(self, servo_id, delta_servo, angle_before, angle_after):
        """
        Append an angle change entry to the history log.
        
        Args:
            servo_id: 'pan' or 'tilt' (or 'A'/'B')
            delta_servo: servo movement in degrees
            angle_before: needle angle before movement
            angle_after: needle angle after movement
        """
        delta_angle = angle_after - angle_before
        # Normalize angle change (handle wrap-around)
        if delta_angle > 90:
            delta_angle -= 180
        elif delta_angle < -90:
            delta_angle += 180
        
        self.step_idx += 1
        self.angle_log.append({
            "step": self.step_idx,
            "servo": servo_id.upper()[:1] if len(servo_id) > 1 else servo_id.upper(),  # 'pan' -> 'P', 'tilt' -> 'T'
            "delta_servo": float(delta_servo),
            "angle_before": float(angle_before),
            "angle_after": float(angle_after),
            "delta_angle": float(delta_angle)
        })
        
        # Keep list bounded
        if len(self.angle_log) > ANGLE_LOG_MAX_LEN:
            self.angle_log.pop(0)
    
    def save_angle_log_to_file(self, filename=None):
        """
        Save the angle history log to a JSON file.
        
        Args:
            filename: Optional filename. If None, generates timestamped filename.
        
        Returns:
            filename: The filename used
        """
        if filename is None:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"angle_history_{timestamp}.json"
        
        # Ensure .json extension
        if not filename.endswith('.json'):
            filename += '.json'
        
        # Prepare data to save
        data = {
            'timestamp': datetime.now().isoformat(),
            'total_steps': len(self.angle_log),
            'pan_sensitivity': self.pan_sensitivity,
            'tilt_sensitivity': self.tilt_sensitivity,
            'pan_position_sensitivity': self.pan_position_sensitivity,
            'tilt_position_sensitivity': self.tilt_position_sensitivity,
            'primary_servo': self.primary_servo,
            'angle_log': self.angle_log
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"\n💾 Angle history saved to: {filename}")
            print(f"   Total steps: {len(self.angle_log)}")
            return filename
        except Exception as e:
            print(f"\n❌ Failed to save angle history: {e}")
            return None
    
    def load_angle_log_from_file(self, filename):
        """
        Load angle history from a JSON file.
        
        Args:
            filename: Path to JSON file
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.angle_log = data.get('angle_log', [])
            self.step_idx = len(self.angle_log)
            
            # Optionally restore calibration data
            if 'pan_sensitivity' in data:
                self.pan_sensitivity = data.get('pan_sensitivity')
            if 'tilt_sensitivity' in data:
                self.tilt_sensitivity = data.get('tilt_sensitivity')
            
            print(f"\n📂 Loaded angle history from: {filename}")
            print(f"   Total steps: {len(self.angle_log)}")
            return True
        except Exception as e:
            print(f"\n❌ Failed to load angle history: {e}")
            return False
    
    def draw_detection_results(self, frame, needle_line, hair_line, angle_info):
        """Draw detection results on frame"""
        result = frame.copy()
        
        if needle_line:
            if 'point1' in needle_line and 'point2' in needle_line:
                cv2.line(result, needle_line['point1'], needle_line['point2'], (0, 255, 255), 4)
                cv2.circle(result, needle_line['point1'], 6, (0, 200, 255), -1)
                cv2.circle(result, needle_line['point2'], 6, (0, 255, 200), -1)
                cv2.circle(result, needle_line['center'], 4, (0, 255, 255), -1)
                
                # Display angle number near the line (use stabilized angle, rounded to whole degrees)
                center_x = int(needle_line['center'][0]) if isinstance(needle_line['center'][0], (int, float)) else int(needle_line['center'][0].item())
                center_y = int(needle_line['center'][1]) if isinstance(needle_line['center'][1], (int, float)) else int(needle_line['center'][1].item())
                # Use stabilized angle if available, otherwise use raw angle
                angle_to_display = needle_line.get('stable_angle', needle_line['angle'])
                display_angle = round(angle_to_display)  # Round to whole number
                # Always show angle number (no "ALIGNED" text near lines)
                angle_text = f"{display_angle}"
                # Position text slightly above and to the right of the center
                text_x = center_x + 15
                text_y = center_y - 15
                # Draw background rectangle for better visibility
                (text_width, text_height), baseline = cv2.getTextSize(angle_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(result, (text_x - 5, text_y - text_height - 5), 
                             (text_x + text_width + 5, text_y + baseline + 5), 
                             (0, 0, 0), -1)  # Black background
                cv2.putText(result, angle_text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                angle_rad = np.radians(needle_line['angle'])
                arrow_length = 20
                arrow_end = (
                    int(center_x + arrow_length * np.cos(angle_rad)),
                    int(center_y + arrow_length * np.sin(angle_rad))
                )
                cv2.arrowedLine(result, needle_line['center'], arrow_end, (0, 255, 255), 2, tipLength=0.3)
                
                if self.enable_boundary_check:
                    boundary_status = self.check_boundary_violation(needle_line)
                    center = needle_line['center']
                    
                    margin = self.boundary_margin
                    cv2.rectangle(result, (margin, margin), 
                                 (self.frame_width - margin, self.frame_height - margin),
                                 (0, 165, 255), 1)
                    
                    if (boundary_status['near_left'] or boundary_status['near_right'] or
                        boundary_status['near_top'] or boundary_status['near_bottom']):
                        cv2.putText(result, "NEAR EDGE!", (center[0] - 50, center[1] - 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        if hair_line:
            color = (255, 100, 0) if self.target_hair_color == 'blue' else (0, 255, 0)
            if 'point1' in hair_line and 'point2' in hair_line:
                cv2.line(result, hair_line['point1'], hair_line['point2'], color, 4)
                cv2.circle(result, hair_line['point1'], 6, tuple(int(c*0.8) for c in color), -1)
                cv2.circle(result, hair_line['point2'], 6, color, -1)
                cv2.circle(result, hair_line['center'], 4, color, -1)
                
                # Display angle number near the line (use stabilized angle, rounded to whole degrees)
                center_x = int(hair_line['center'][0]) if isinstance(hair_line['center'][0], (int, float)) else int(hair_line['center'][0].item())
                center_y = int(hair_line['center'][1]) if isinstance(hair_line['center'][1], (int, float)) else int(hair_line['center'][1].item())
                # Use stabilized angle if available, otherwise use raw angle
                angle_to_display = hair_line.get('stable_angle', hair_line['angle'])
                display_angle = round(angle_to_display)  # Round to whole number
                # Always show angle number (no "ALIGNED" text near lines)
                angle_text = f"{display_angle}"
                # Position text slightly above and to the left of the center
                text_x = center_x - 50
                text_y = center_y - 15
                # Draw background rectangle for better visibility
                (text_width, text_height), baseline = cv2.getTextSize(angle_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(result, (text_x - 5, text_y - text_height - 5), 
                             (text_x + text_width + 5, text_y + baseline + 5), 
                             (0, 0, 0), -1)  # Black background
                cv2.putText(result, angle_text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                angle_rad = np.radians(hair_line['angle'])
                arrow_length = 20
                arrow_end = (
                    int(center_x + arrow_length * np.cos(angle_rad)),
                    int(center_y + arrow_length * np.sin(angle_rad))
                )
                cv2.arrowedLine(result, hair_line['center'], arrow_end, color, 2, tipLength=0.3)
        
        info_y = 20
        if angle_info:
            # Use stabilized angles if available, then round to whole degrees
            needle_angle = angle_info.get('needle_stable_angle', angle_info['needle_angle'])
            hair_angle = angle_info.get('hair_stable_angle', angle_info['hair_angle'])
            needle_display = round(needle_angle)
            hair_display = round(hair_angle)
            error_display = round(angle_info['angle_diff'])
            
            # Check if aligned (within tolerance)
            is_aligned = abs(error_display) <= self.angle_tolerance
            
            # Always show angle numbers, and add "ALIGNED" status when aligned
            cv2.putText(result, f"Needle: {needle_display}", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(result, f"Hair: {hair_display}", 
                       (10, info_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                       (255, 100, 0) if self.target_hair_color == 'blue' else (0, 255, 0), 2)
            cv2.putText(result, f"Error: {error_display:+}", 
                       (10, info_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            if is_aligned:
                # Show "ALIGNED" status below the error
                cv2.putText(result, "ALIGNED", 
                           (10, info_y + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        status_y = result.shape[0] - 60
        mode_text = "MANUAL MODE" if self.manual_mode else "AUTO MODE"
        mode_color = (0, 255, 255) if self.manual_mode else (0, 255, 0)
        cv2.putText(result, f"Mode: {mode_text}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1)
        cv2.putText(result, f"Pan: {self.current_pan_angle:.1f} | Tilt: {self.current_tilt_angle:.1f}", 
                   (10, status_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(result, f"Iterations: {self.iterations}", 
                   (10, status_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return result
    
    def create_control_panel(self, target_height=None):
        """
        Create a visual joystick control panel with buttons and input field.
        Redesigned compact layout to fit all buttons.
        
        Args:
            target_height: If provided, resize panel to this height (default: self.control_panel_height)
        
        Returns:
            panel: OpenCV image with control buttons
        """
        if target_height is None:
            target_height = self.control_panel_height
        panel = np.ones((target_height, self.control_panel_width, 3), dtype=np.uint8) * 240  # Light gray background
        
        # Title
        cv2.putText(panel, "CONTROL PANEL", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Current positions (compact)
        y_pos = 45
        cv2.putText(panel, f"Pan: {self.current_pan_angle:.1f}  Tilt: {self.current_tilt_angle:.1f}", 
                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        
        # Joystick buttons (more compact)
        y_pos = 70
        button_size = 45
        button_spacing = 55
        center_x = self.control_panel_width // 2
        
        # Pan buttons (horizontal)
        pan_y = y_pos
        # Left button
        pan_left_rect = (center_x - button_spacing - button_size//2, pan_y - button_size//2, button_size, button_size)
        cv2.rectangle(panel, (pan_left_rect[0], pan_left_rect[1]), 
                     (pan_left_rect[0] + pan_left_rect[2], pan_left_rect[1] + pan_left_rect[3]), 
                     (100, 100, 255), -1)
        cv2.putText(panel, "<", (pan_left_rect[0] + 16, pan_left_rect[1] + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(panel, "PAN", (pan_left_rect[0] - 8, pan_left_rect[1] - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
        
        # Right button
        pan_right_rect = (center_x + button_spacing - button_size//2, pan_y - button_size//2, button_size, button_size)
        cv2.rectangle(panel, (pan_right_rect[0], pan_right_rect[1]), 
                     (pan_right_rect[0] + pan_right_rect[2], pan_right_rect[1] + pan_right_rect[3]), 
                     (100, 100, 255), -1)
        cv2.putText(panel, ">", (pan_right_rect[0] + 16, pan_right_rect[1] + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # Tilt buttons (vertical, more compact)
        tilt_y = y_pos + button_spacing + 15
        # Up button
        tilt_up_rect = (center_x - button_size//2, tilt_y - button_spacing//2 - button_size//2, button_size, button_size)
        cv2.rectangle(panel, (tilt_up_rect[0], tilt_up_rect[1]), 
                     (tilt_up_rect[0] + tilt_up_rect[2], tilt_up_rect[1] + tilt_up_rect[3]), 
                     (100, 255, 100), -1)
        cv2.putText(panel, "^", (tilt_up_rect[0] + 16, tilt_up_rect[1] + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(panel, "TILT", (tilt_up_rect[0] - 4, tilt_up_rect[1] - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
        
        # Down button
        tilt_down_rect = (center_x - button_size//2, tilt_y + button_spacing//2 - button_size//2, button_size, button_size)
        cv2.rectangle(panel, (tilt_down_rect[0], tilt_down_rect[1]), 
                     (tilt_down_rect[0] + tilt_down_rect[2], tilt_down_rect[1] + tilt_down_rect[3]), 
                     (100, 255, 100), -1)
        cv2.putText(panel, "v", (tilt_down_rect[0] + 16, tilt_down_rect[1] + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # Calculate Angle button (prominent, placed early to ensure visibility)
        calc_y = tilt_y + button_spacing + 20
        calc_rect = (10, calc_y, 280, 45)
        calc_color = (50, 150, 255)  # Orange/blue color
        cv2.rectangle(panel, (calc_rect[0], calc_rect[1]), 
                     (calc_rect[0] + calc_rect[2], calc_rect[1] + calc_rect[3]), 
                     calc_color, -1)
        cv2.rectangle(panel, (calc_rect[0], calc_rect[1]), 
                     (calc_rect[0] + calc_rect[2], calc_rect[1] + calc_rect[3]), 
                     (0, 0, 0), 2)
        cv2.putText(panel, "CALCULATE ANGLE", (calc_rect[0] + 45, calc_rect[1] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        
        # Typed input buttons (compact)
        input_y = calc_y + 55
        # Pan input button
        pan_input_rect = (10, input_y, 135, 30)
        cv2.rectangle(panel, (pan_input_rect[0], pan_input_rect[1]), 
                     (pan_input_rect[0] + pan_input_rect[2], pan_input_rect[1] + pan_input_rect[3]), 
                     (200, 200, 200), -1)
        cv2.rectangle(panel, (pan_input_rect[0], pan_input_rect[1]), 
                     (pan_input_rect[0] + pan_input_rect[2], pan_input_rect[1] + pan_input_rect[3]), 
                     (0, 0, 0), 2)
        cv2.putText(panel, "PAN Input", (pan_input_rect[0] + 5, pan_input_rect[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        
        # Tilt input button
        tilt_input_rect = (155, input_y, 135, 30)
        cv2.rectangle(panel, (tilt_input_rect[0], tilt_input_rect[1]), 
                     (tilt_input_rect[0] + tilt_input_rect[2], tilt_input_rect[1] + tilt_input_rect[3]), 
                     (200, 200, 200), -1)
        cv2.rectangle(panel, (tilt_input_rect[0], tilt_input_rect[1]), 
                     (tilt_input_rect[0] + tilt_input_rect[2], tilt_input_rect[1] + tilt_input_rect[3]), 
                     (0, 0, 0), 2)
        cv2.putText(panel, "TILT Input", (tilt_input_rect[0] + 5, tilt_input_rect[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        
        # Input text display (compact)
        input_display_y = input_y + 40
        cv2.putText(panel, "Type degrees:", (10, input_display_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        
        input_box_rect = (10, input_display_y + 8, 280, 32)
        cv2.rectangle(panel, (input_box_rect[0], input_box_rect[1]), 
                     (input_box_rect[0] + input_box_rect[2], input_box_rect[1] + input_box_rect[3]), 
                     (255, 255, 255), -1)
        cv2.rectangle(panel, (input_box_rect[0], input_box_rect[1]), 
                     (input_box_rect[0] + input_box_rect[2], input_box_rect[1] + input_box_rect[3]), 
                     (0, 0, 0), 2)
        
        # Display current input text
        if self.input_text:
            cv2.putText(panel, self.input_text, (input_box_rect[0] + 5, input_box_rect[1] + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        elif self.input_mode:
            cv2.putText(panel, f"Enter {self.input_mode.upper()}...", 
                       (input_box_rect[0] + 5, input_box_rect[1] + 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)
        
        # Enter button (compact)
        enter_y = input_display_y + 45
        enter_rect = (10, enter_y, 280, 30)
        enter_color = (100, 200, 100) if self.input_text else (150, 150, 150)
        cv2.rectangle(panel, (enter_rect[0], enter_rect[1]), 
                     (enter_rect[0] + enter_rect[2], enter_rect[1] + enter_rect[3]), 
                     enter_color, -1)
        cv2.putText(panel, "ENTER", (enter_rect[0] + 110, enter_rect[1] + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        
        # Store button rectangles for click detection
        self.button_rects = {
            'pan_left': pan_left_rect,
            'pan_right': pan_right_rect,
            'tilt_up': tilt_up_rect,
            'tilt_down': tilt_down_rect,
            'pan_input': pan_input_rect,
            'tilt_input': tilt_input_rect,
            'input_box': input_box_rect,
            'enter': enter_rect,
            'calculate_angle': calc_rect
        }
        
        return panel
    
    def handle_control_panel_click(self, x, y):
        """
        Handle mouse click on control panel.
        
        Args:
            x, y: Mouse click coordinates in control panel
        """
        if not self.control_panel_open:
            print(f"   ⚠️  Control panel not open")
            return
        
        if not hasattr(self, 'button_rects'):
            print(f"   ⚠️  Button rects not initialized")
            return
        
        print(f"   🔍 Checking click at ({x}, {y}) against {len(self.button_rects)} buttons")
        
        # Check which button was clicked
        for button_name, rect in self.button_rects.items():
            if (rect[0] <= x <= rect[0] + rect[2] and 
                rect[1] <= y <= rect[1] + rect[3]):
                
                print(f"   ✅ Button clicked: {button_name}")
                
                if button_name == 'pan_left':
                    print(f"   🎮 Moving PAN LEFT by {self.joystick_step_size}°")
                    self.handle_joystick_move('pan', -self.joystick_step_size)
                elif button_name == 'pan_right':
                    print(f"   🎮 Moving PAN RIGHT by {self.joystick_step_size}°")
                    self.handle_joystick_move('pan', self.joystick_step_size)
                elif button_name == 'tilt_up':
                    print(f"   🎮 Moving TILT UP by {self.joystick_step_size}°")
                    self.handle_joystick_move('tilt', -self.joystick_step_size)
                elif button_name == 'tilt_down':
                    print(f"   🎮 Moving TILT DOWN by {self.joystick_step_size}°")
                    self.handle_joystick_move('tilt', self.joystick_step_size)
                elif button_name == 'pan_input':
                    self.input_mode = 'pan'
                    self.input_text = ""
                    print(f"\n🎮 PAN Input mode activated - Type degrees and press Enter")
                elif button_name == 'tilt_input':
                    self.input_mode = 'tilt'
                    self.input_text = ""
                    print(f"\n🎮 TILT Input mode activated - Type degrees and press Enter")
                elif button_name == 'input_box':
                    # Focus on input box - activate input mode if not already active
                    if not self.input_mode:
                        print(f"   💡 Click 'PAN Input' or 'TILT Input' first to select servo")
                    else:
                        self.input_text = ""
                elif button_name == 'enter':
                    if self.input_text and self.input_mode:
                        # Execute typed movement
                        try:
                            delta = float(self.input_text)
                            print(f"   🎮 Executing {self.input_mode.upper()} movement: {delta}°")
                            self.handle_joystick_move(self.input_mode, delta)
                            self.input_text = ""
                            self.input_mode = None
                        except ValueError:
                            print(f"   ❌ Invalid input: '{self.input_text}'")
                            self.input_text = ""
                    else:
                        print(f"   ⚠️  No input to execute. Click 'PAN Input' or 'TILT Input' first, then type degrees.")
                elif button_name == 'calculate_angle':
                    print(f"   📊 Manual angle calculation requested...")
                    self.manual_angle_calculation()
                return
        
        print(f"   ⚠️  No button clicked at ({x}, {y})")
    
    def handle_control_panel_key(self, key):
        """
        Handle keyboard input for control panel.
        
        Args:
            key: Key code from cv2.waitKey
        """
        if not self.control_panel_open:
            return False
        
        # If in input mode, handle text input
        if self.input_mode:
            # Handle backspace
            if key == 8 or key == 127:  # Backspace
                if self.input_text:
                    self.input_text = self.input_text[:-1]
                return True
            
            # Handle Enter
            if key == 13 or key == 10:  # Enter
                if self.input_text:
                    try:
                        delta = float(self.input_text)
                        self.handle_joystick_move(self.input_mode, delta)
                        self.input_text = ""
                        self.input_mode = None
                        print(f"   ✅ Executed {self.input_mode} movement: {delta}°")
                        return True
                    except ValueError:
                        print(f"   ❌ Invalid input: '{self.input_text}'")
                        self.input_text = ""
                        return True
                return True
            
            # Handle number input
            if 48 <= key <= 57:  # 0-9
                self.input_text += chr(key)
                return True
            elif key == ord('+') or key == ord('='):
                self.input_text += '+'
                return True
            elif key == ord('-') or key == ord('_'):
                self.input_text += '-'
                return True
            elif key == ord('.') or key == ord(','):
                self.input_text += '.'
                return True
        
        return False
    
    def measure_and_log_angle_change(self, servo_type, delta_servo):
        """
        Measure current angle and log it (for manual mode).
        If there's a previous angle, calculate the change.
        """
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        processed = self.process_frame(frame)
        if processed is None:
            return None
        
        needle_line = self.detect_line(processed, 'yellow')
        if needle_line is None:
            print("   ⚠️  Cannot measure: Yellow needle not detected")
            return None
        
        # Use stable_angle for control if available, otherwise use display angle
        current_angle = needle_line.get('stable_angle', needle_line['angle'])
        
        # If we have a previous angle, log the change
        if len(self.angle_log) > 0:
            last_entry = self.angle_log[-1]
            angle_before = last_entry['angle_after']  # Use last measured angle as before
            self.log_angle_change(servo_type, delta_servo, angle_before, current_angle)
        else:
            # First measurement - just record it
            self.step_idx += 1
            self.angle_log.append({
                "step": self.step_idx,
                "servo": servo_type.upper()[:1],
                "delta_servo": float(delta_servo),
                "angle_before": float(current_angle),
                "angle_after": float(current_angle),
                "delta_angle": 0.0
            })
            if len(self.angle_log) > ANGLE_LOG_MAX_LEN:
                self.angle_log.pop(0)
        
        return current_angle
    
    def handle_manual_servo_move(self, servo_type):
        """
        Handle manual servo movement input from user.
        
        Args:
            servo_type: 'pan' or 'tilt'
        """
        print(f"\n🎮 Manual {servo_type.upper()} control")
        print("   Enter movement (e.g., +5, -3, 10, -2.5): ", end='', flush=True)
        
        # Get current angle before movement
        ret, frame_before = self.cap.read()
        if not ret:
            print("   ❌ Cannot read camera")
            return
        
        processed_before = self.process_frame(frame_before)
        if processed_before is None:
            print("   ❌ Cannot process frame")
            return
        
        needle_line_before = self.detect_line(processed_before, 'yellow')
        if needle_line_before is None:
            print("   ❌ Yellow needle not detected")
            return
        
        angle_before = needle_line_before['angle']
        current_pos = self.current_pan_angle if servo_type == 'pan' else self.current_tilt_angle
        
        print(f"   Current {servo_type}: {current_pos:.1f}°, Needle angle: {angle_before:.2f}°")
        print("   Type movement and press Enter (or 'c' to cancel): ", end='', flush=True)
        
        # Read input from terminal (non-blocking would be better, but this works)
        # For now, we'll use a simple approach: wait for user to type in terminal
        import sys
        try:
            user_input = input().strip()
            if user_input.lower() == 'c':
                print("   ❌ Cancelled")
                return
            
            # Parse input: +5, -3, 5, -2.5, etc.
            try:
                delta = float(user_input)
            except ValueError:
                print(f"   ❌ Invalid input: '{user_input}' - expected number like +5 or -3")
                return
            
            # Move servo
            new_pos = current_pos + delta
            new_pos = max(0, min(180, new_pos))
            
            print(f"   Moving {servo_type} from {current_pos:.1f}° to {new_pos:.1f}° (delta: {delta:+.1f}°)")
            
            if servo_type == 'pan':
                self.set_servo_angles(pan_angle=new_pos)
            else:
                self.set_servo_angles(tilt_angle=new_pos)
            
            # Wait for movement
            time.sleep(self.movement_delay)
            
            # Measure new angle
            ret, frame_after = self.cap.read()
            if not ret:
                print("   ❌ Cannot read camera after movement")
                return
            
            processed_after = self.process_frame(frame_after)
            if processed_after is None:
                print("   ❌ Cannot process frame after movement")
                return
            
            needle_line_after = self.detect_line(processed_after, 'yellow')
            if needle_line_after is None:
                print("   ⚠️  Yellow needle not detected after movement")
                return
            
            angle_after = needle_line_after['angle']
            
            # Log the change
            self.log_angle_change(servo_type, delta, angle_before, angle_after)
            
            # Calculate angle change
            angle_change = angle_after - angle_before
            if angle_change > 90:
                angle_change -= 180
            elif angle_change < -90:
                angle_change += 180
            
            print(f"   ✅ Movement complete!")
            print(f"   📊 Angle: {angle_before:.2f}° → {angle_after:.2f}° (change: {angle_change:+.2f}°)")
            if abs(delta) > 0.1:
                sensitivity = angle_change / delta
                print(f"   📊 Sensitivity: {sensitivity:.3f}° angle per ° servo")
            
            print("   ✅ Camera window will reopen - click on it to continue")
            
        except (EOFError, KeyboardInterrupt):
            print("\n   ❌ Input cancelled")
        except Exception as e:
            print(f"\n   ❌ Error: {e}")
            import traceback
            traceback.print_exc()
    
    def handle_typed_servo_move(self, servo_type):
        """
        Handle typed servo movement - user types exact degrees to move.
        This is an alias for handle_manual_servo_move for consistency.
        
        Args:
            servo_type: 'pan' or 'tilt'
        """
        self.handle_manual_servo_move(servo_type)
    
    def manual_angle_calculation(self):
        """
        Manually calculate and log the current angle.
        Compares with the last logged angle to show the change.
        """
        print(f"\n   📊 Manual Angle Calculation")
        print(f"   " + "="*50)
        
        # Measure current angle using stable measurement
        angle_current, midpoint_current = self.measure_stable_angle_and_position()
        if angle_current is None:
            print(f"   ❌ Could not measure angle - needle not detected")
            return
        
        print(f"   ✅ Current needle angle: {angle_current:.2f}°")
        
        # Also measure hair angle for comparison and show debug info
        ret, frame = self.cap.read()
        if ret:
            processed = self.process_frame(frame)
            if processed is not None:
                # Get needle line again for debug info
                needle_line_debug = self.detect_line(processed, 'yellow')
                hair_line = self.detect_line(processed, self.target_hair_color)
                if hair_line is not None and needle_line_debug is not None:
                    hair_angle = hair_line['angle']
                    print(f"   ✅ Current hair angle: {hair_angle:.2f}°")
                    print(f"   📊 Angle difference: {abs(angle_current - hair_angle):.2f}°")
                    
                    # Debug: show detailed angle information
                    print(f"\n   🔍 DEBUG INFORMATION:")
                    print(f"      Needle:")
                    if 'raw_angle' in needle_line_debug:
                        print(f"        Raw angle: {needle_line_debug['raw_angle']:.2f}°")
                        print(f"        Normalized: {needle_line_debug['normalized_angle']:.2f}°")
                        print(f"        Stabilized: {needle_line_debug['angle']:.2f}°")
                        print(f"        Direction vector: ({needle_line_debug.get('vx', 0):.3f}, {needle_line_debug.get('vy', 0):.3f})")
                    print(f"      Hair:")
                    if 'raw_angle' in hair_line:
                        print(f"        Raw angle: {hair_line['raw_angle']:.2f}°")
                        print(f"        Normalized: {hair_line['normalized_angle']:.2f}°")
                        print(f"        Stabilized: {hair_line['angle']:.2f}°")
                        print(f"        Direction vector: ({hair_line.get('vx', 0):.3f}, {hair_line.get('vy', 0):.3f})")
                    
                    if abs(angle_current - hair_angle) > 10:
                        print(f"\n   ⚠️  WARNING: Large angle difference detected!")
                        print(f"      If lines look parallel, angles should be similar.")
                        print(f"      Possible causes:")
                        print(f"      1. Lines are not actually parallel")
                        print(f"      2. Angle stabilization causing drift")
                        print(f"      3. Normalization edge case")
                        print(f"      4. Measurement noise")
        
        # Get the last logged angle (if any)
        if len(self.angle_log) > 0:
            last_entry = self.angle_log[-1]
            angle_before = last_entry['angle_after']
            print(f"   📋 Last logged angle: {angle_before:.2f}°")
            
            # Calculate change
            angle_change = angle_current - angle_before
            if angle_change > 90:
                angle_change -= 180
            elif angle_change < -90:
                angle_change += 180
            
            print(f"   📊 Angle change: {angle_change:+.2f}°")
            
            # Log this as a measurement (no servo movement, just measurement)
            self.step_idx += 1
            self.angle_log.append({
                "step": self.step_idx,
                "servo": "M",  # 'M' for Manual/Measurement
                "delta_servo": 0.0,  # No servo movement
                "angle_before": float(angle_before),
                "angle_after": float(angle_current),
                "delta_angle": float(angle_change)
            })
            
            # Keep list bounded
            if len(self.angle_log) > ANGLE_LOG_MAX_LEN:
                self.angle_log.pop(0)
            
            print(f"   📝 Logged: Step {self.step_idx}, Manual measurement, angle {angle_before:.2f}° → {angle_current:.2f}° (Δ{angle_change:+.2f}°)")
        else:
            # First measurement - just record it
            self.step_idx += 1
            self.angle_log.append({
                "step": self.step_idx,
                "servo": "M",
                "delta_servo": 0.0,
                "angle_before": float(angle_current),
                "angle_after": float(angle_current),
                "delta_angle": 0.0
            })
            print(f"   📝 Logged: Step {self.step_idx}, Initial measurement, angle {angle_current:.2f}°")
        
        print(f"   " + "="*50 + "\n")
    
    def handle_joystick_move(self, servo_type, delta):
        """
        Handle joystick movement (arrow keys or WASD).
        Moves servo by delta degrees and measures angle change.
        
        Args:
            servo_type: 'pan' or 'tilt'
            delta: movement in degrees (positive or negative)
        """
        current_time = time.time()
        
        # Throttle joystick movements to avoid too many measurements
        if current_time - self.last_joystick_move_time < self.joystick_repeat_delay:
            return
        
        self.last_joystick_move_time = current_time
        
        # Get current position
        current_pos = self.current_pan_angle if servo_type == 'pan' else self.current_tilt_angle
        
        # Calculate new position
        new_pos = current_pos + delta
        new_pos = max(0, min(180, new_pos))
        
        # If position didn't change (at limits), don't do anything
        if abs(new_pos - current_pos) < 0.1:
            return
        
        # Measure angle before movement (single frame for speed)
        ret, frame_before = self.cap.read()
        if not ret:
            print(f"   ⚠️  Cannot read frame before movement")
            return
        processed_before = self.process_frame(frame_before)
        if processed_before is None:
            print(f"   ⚠️  Cannot process frame before movement")
            return
        needle_line_before = self.detect_line(processed_before, 'yellow')
        if needle_line_before is None:
            print(f"   ⚠️  Cannot detect needle before movement")
            return
        angle_before = needle_line_before['angle']
        print(f"   📊 Angle before: {angle_before:.2f}°")
        
        # Move servo
        print(f"   🔧 Moving {servo_type.upper()} from {current_pos:.1f}° to {new_pos:.1f}° (Δ{delta:+.1f}°)")
        if servo_type == 'pan':
            self.set_servo_angles(pan_angle=new_pos)
            self.current_pan_angle = new_pos
        else:
            self.set_servo_angles(tilt_angle=new_pos)
            self.current_tilt_angle = new_pos
        
        # Wait for servo to move and camera to update (need enough time for physical movement)
        wait_time = max(self.movement_delay, 0.5)  # At least 0.5s for servo to move
        print(f"   ⏳ Waiting {wait_time:.1f}s for servo movement...")
        time.sleep(wait_time)
        
        # Measure angle after movement - take multiple samples and use median for stability
        print(f"   📊 Measuring angle after movement...")
        angles_after = []
        for i in range(3):  # Take 3 samples
            ret, frame_after = self.cap.read()
            if ret:
                processed_after = self.process_frame(frame_after)
                if processed_after is not None:
                    needle_line_after = self.detect_line(processed_after, 'yellow')
                    if needle_line_after is not None:
                        angles_after.append(needle_line_after['angle'])
            time.sleep(0.1)  # Small delay between samples
        
        if len(angles_after) > 0:
            angle_after = np.median(angles_after)  # Use median for stability
            print(f"   ✅ Angle after: {angle_after:.2f}° (from {len(angles_after)} samples)")
        else:
            print(f"   ⚠️  Could not measure angle after - using before value")
            angle_after = angle_before
        
        # Log the change (always log, even if measurement failed)
        self.log_angle_change(servo_type, delta, angle_before, angle_after)
        
        # Calculate and display the change
        angle_change = angle_after - angle_before
        if angle_change > 90:
            angle_change -= 180
        elif angle_change < -90:
            angle_change += 180
        print(f"   📝 Logged: Step {self.step_idx}, {servo_type.upper()}, Δservo={delta:.1f}°, angle {angle_before:.2f}° → {angle_after:.2f}° (Δ{angle_change:+.2f}°)")
        
        # Calculate and display sensitivity
        angle_change = angle_after - angle_before
        if angle_change > 90:
            angle_change -= 180
        elif angle_change < -90:
            angle_change += 180
        
        direction = "→" if delta > 0 else "←"
        print(f"   🎮 {servo_type.upper()} {direction} {abs(delta):.1f}°: {angle_before:.1f}° → {angle_after:.1f}° (Δ{angle_change:+.2f}°)")
    
    def start_alignment(self):
        """Start the alignment process"""
        print("\n" + "="*70)
        print("🚀 STARTING ALIGNMENT (Single Camera)")
        print("="*70)
        print(f"   Target hair: {self.target_hair_color}")
        print(f"   Angle tolerance: {self.angle_tolerance}°")
        print("="*70)
        
        # Reset state
        self.iterations = 0
        self.is_aligned = False
        self.prev_angle_error = None
        self.prev_needle_angle = None
        self.prev_pan_adjustment = 0.0
        self.prev_tilt_adjustment = 0.0
        self.pan_direction_corrected = False
        self.tilt_direction_corrected = False
        self.no_change_count = 0
        self.pan_no_change_count = 0
        self.tilt_no_change_count = 0
        self.use_pan_only = False
        self.use_tilt_only = False
        self.missing_count = 0
        self.recovery_search_step = 0
        self.last_needle_position = None
        self.initial_movements_count = 0
        self.stagnation_counter = 0
        self.angle_log = []  # Reset angle log
        self.step_idx = 0  # Reset step counter
        
        # Run calibration if not done
        if not self.calibration_done:
            self.calibrate_servos()  # Will set fallback values if calibration fails
        
        return True
    
    def step_alignment(self):
        """Execute one step of alignment"""
        current_time = time.time()
        
        if current_time - self.last_control_update < self.control_update_interval:
            return
        
        self.last_control_update = current_time
        
        ret, frame = self.cap.read()
        if not ret:
            print("❌ Failed to read from camera")
            return
        
        processed = self.process_frame(frame)
        if processed is None:
            return
        
        needle_line = self.detect_line(processed, 'yellow')
        hair_line = self.detect_line(processed, self.target_hair_color)
        
        if needle_line is None:
            self.missing_count += 1
            print(f"⚠️  Yellow needle not detected (out of view) - count: {self.missing_count}")
            
            if self.missing_count >= 2:
                self.recover_needle_to_view()
            
            if hair_line is None:
                print(f"⚠️  Also missing: {self.target_hair_color} hair")
            return
        
        if self.missing_count > 0:
            print(f"✅ Yellow needle found again!")
        self.missing_count = 0
        self.recovery_search_step = 0
        self.last_needle_position = needle_line['center']
        
        if hair_line is None:
            print(f"⚠️  Missing: {self.target_hair_color} hair")
            return
        
        angle_info = self.calculate_angle_difference(needle_line, hair_line)
        if angle_info is None:
            return
        
        self.iterations += 1
        
        print(f"\n[ALIGNMENT - Iteration {self.iterations}]")
        print(f"   Needle: {angle_info['needle_angle']:.2f}°, "
              f"Hair: {angle_info['hair_angle']:.2f}°, "
              f"Error: {angle_info['angle_diff']:+.2f}°")
        
        if self.calibration_done and self.primary_servo:
            primary_k = self.pan_sensitivity if self.primary_servo == 'pan' else self.tilt_sensitivity
            print(f"   Using: {self.primary_servo.upper()} (k={primary_k:.3f})")
        
        self.update_control(angle_info, needle_line)
        
        return angle_info, needle_line, hair_line
    
    def run(self):
        """Main execution loop"""
        if not self.initialize_camera():
            return
        
        arduino_connected = self.initialize_arduino()
        
        print("\n" + "="*70)
        print("🎯 SINGLE CAMERA ALIGNMENT SYSTEM")
        print("="*70)
        print("\n📋 Controls:")
        print("   'm' - Toggle MANUAL/AUTO mode")
        print("   's' - Start automatic alignment (AUTO mode only)")
        print("   'k' - Re-calibrate servos")
        print("   'r' - Reset alignment")
        print("   'b' - Switch target hair to BLUE")
        print("   'g' - Switch target hair to GREEN")
        print("   'c' - Calculate angle manually (or Center servos in AUTO mode)")
        print("   'x' - Reset angle stabilization (clear angle history buffers)")
        print("   'w' - Save angle history (AUTO mode) / Tilt UP (MANUAL mode)")
        print("   'l' - Load angle history from file")
        print("   'q' - Quit")
        print("\n📋 Manual Mode Controls:")
        print("   WASD = Quick 1° movements (W=Tilt↑, S=Tilt↓, A=Pan←, D=Pan→)")
        print("   P = Move PAN by typed amount (type degrees when prompted)")
        print("   T = Move TILT by typed amount (type degrees when prompted)")
        print("   Each movement measures angle change")
        print("\n📋 Current Settings:")
        print(f"   Target hair: {self.target_hair_color}")
        print(f"   Angle tolerance: {self.angle_tolerance}°")
        print(f"   Kp_pan: {self.Kp_pan}, Kp_tilt: {self.Kp_tilt}")
        print(f"   Arduino: {'Connected' if arduino_connected else 'Simulation'}")
        print(f"   Calibration: {'Done' if self.calibration_done else 'Will run on start'}")
        if self.calibration_done:
            if self.pan_sensitivity is not None:
                print(f"   Pan sensitivity: {self.pan_sensitivity:.3f}° angle/° servo")
            if self.tilt_sensitivity is not None:
                print(f"   Tilt sensitivity: {self.tilt_sensitivity:.3f}° angle/° servo")
            if self.primary_servo:
                print(f"   Primary servo: {self.primary_servo.upper()}")
        print("="*70)
        
        alignment_active = False
        last_frame_time = time.time()
        target_fps = 30  # Target frames per second
        frame_interval = 1.0 / target_fps
        
        while True:
            # Frame rate limiting to prevent glitching
            current_time = time.time()
            elapsed = current_time - last_frame_time
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
            last_frame_time = time.time()
            
            ret, frame = self.cap.read()
            if not ret or frame is None:
                print("❌ Failed to read from camera - trying to reconnect...")
                # Try to reinitialize camera
                self.cap.release()
                time.sleep(1)
                if not self.initialize_camera():
                    print("❌ Could not reconnect to camera")
                    break
                continue
            
            processed = self.process_frame(frame)
            if processed is None:
                continue
            
            needle_line = self.detect_line(processed, 'yellow')
            hair_line = self.detect_line(processed, self.target_hair_color)
            
            angle_info = None
            if needle_line and hair_line:
                angle_info = self.calculate_angle_difference(needle_line, hair_line)
            
            # Only run automatic alignment if not in manual mode
            if alignment_active and not self.manual_mode:
                result_info = self.step_alignment()
                if result_info:
                    angle_info, needle_line, hair_line = result_info
            
            result = self.draw_detection_results(processed, needle_line, hair_line, angle_info)
            
            # No angle history table - angles are displayed on the camera view
            H, W = result.shape[:2]
            self.camera_frame_width = W  # Store for click detection
            self.table_panel_width = 0  # No table panel
            
            # Add control panel if in manual mode
            if self.manual_mode:
                # Make control panel match the height of the camera frame
                control_panel = self.create_control_panel(target_height=H)
                # Stack: camera view | control panel
                combined = np.hstack([result, control_panel])
                self.control_panel_open = True
            else:
                combined = result  # Just the camera view
                self.control_panel_open = False
            
            window_name = "Single Camera Alignment + Angle Log"
            cv2.imshow(window_name, combined)
            # Move window to front (macOS)
            cv2.moveWindow(window_name, 100, 100)
            
            # Set mouse callback - use closure to capture current frame dimensions
            def mouse_callback(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    # Check if click is in control panel area
                    if self.manual_mode and self.control_panel_open:
                        # Use current frame dimensions from the loop
                        # No table panel anymore, so control panel starts right after camera frame
                        control_panel_start_x = W
                        if x >= control_panel_start_x:
                            panel_x = x - control_panel_start_x
                            print(f"🖱️  Click detected at window ({x}, {y}), panel ({panel_x}, {y})")
                            self.handle_control_panel_click(panel_x, y)
            
            cv2.setMouseCallback(window_name, mouse_callback)
            
            key = cv2.waitKey(1) & 0xFF  # Reduced wait time for smoother display
            
            # Always handle mode toggle and quit first, regardless of control panel
            if key == ord('q'):
                break
            elif key == ord('m'):
                # Toggle manual mode
                self.manual_mode = not self.manual_mode
                alignment_active = False  # Always stop automatic alignment when toggling
                if self.manual_mode:
                    print("\n" + "="*70)
                    print("🎮 MANUAL MODE ENABLED - Automatic movement DISABLED")
                    print("="*70)
                    print("   You have full control of servo movements")
                    print("\n   🎮 Manual Control Options:")
                    print("   📱 Visual Control Panel (appears on right side):")
                    print("      - Click colored buttons: PAN (← →) and TILT (↑ ↓) for 1° moves")
                    print("      - Click 'PAN Input' or 'TILT Input' to enter degree input mode")
                    print("      - Type numbers in the input box (e.g., 5, -3, +10)")
                    print("      - Press Enter button or keyboard Enter to execute")
                    print("   ⌨️  Keyboard Controls:")
                    print("      - WASD: W=Tilt↑, S=Tilt↓, A=Pan←, D=Pan→ (1° moves)")
                    print("      - P: Type PAN degrees, T: Type TILT degrees")
                    print("\n   Each movement measures angle change")
                    print("   Results shown in angle history table")
                    print("="*70)
                else:
                    print("\n🤖 AUTOMATIC MODE - Press 's' to start automatic alignment")
                continue  # Skip other key handling after mode toggle
            
            # Handle control panel keyboard input (but not for mode toggle or quit)
            if self.control_panel_open and self.handle_control_panel_key(key):
                continue  # Key was handled by control panel
            
            if key == ord('s'):
                if self.manual_mode:
                    print("\n⚠️  Manual mode is active - press 'm' to disable first")
                elif self.start_alignment():
                    alignment_active = True
                    print("\n🔄 Alignment started - system will automatically align")
            elif key == ord('r'):
                alignment_active = False
                self.is_aligned = False
                print("\n🔄 Alignment reset")
            elif key == ord('b'):
                self.target_hair_color = 'blue'
                print(f"\n🎯 Target hair changed to: {self.target_hair_color}")
            elif key == ord('g'):
                self.target_hair_color = 'green'
                print(f"\n🎯 Target hair changed to: {self.target_hair_color}")
            elif key == ord('c'):
                if self.manual_mode:
                    # In manual mode, 'c' calculates angle
                    print("\n   📊 Manual angle calculation triggered (keyboard)")
                    self.manual_angle_calculation()
                else:
                    # In auto mode, 'c' centers servos
                    print("\n🏠 Centering servos...")
                    self.set_servo_angles(pan_angle=90, tilt_angle=90)
                    self.current_pan_angle = 90.0
                    self.current_tilt_angle = 90.0
                    print("   ✅ Servos centered")
            elif key == ord('x'):
                # Reset angle stabilization (clear history buffers)
                print("\n🔄 Resetting angle stabilization...")
                for color_name in self.angle_history.keys():
                    self.angle_history[color_name].clear()
                    self.stable_angles[color_name] = None
                print("   ✅ Angle stabilization reset - angles will be recalculated fresh")
            elif key == ord('k'):
                # Re-calibrate servos
                print("\n🔧 Re-calibrating servos...")
                self.calibration_done = False
                if self.calibrate_servos():
                    print("✅ Re-calibration complete")
                else:
                    print("❌ Re-calibration failed")
            elif self.manual_mode and key == ord('w'):
                # W = Tilt UP (joystick control in manual mode)
                self.handle_joystick_move('tilt', -self.joystick_step_size)
            elif self.manual_mode and key == ord('s'):
                # S = Tilt DOWN (joystick control in manual mode)
                self.handle_joystick_move('tilt', self.joystick_step_size)
            elif self.manual_mode and key == ord('a'):
                # A = Pan LEFT (joystick control in manual mode)
                self.handle_joystick_move('pan', -self.joystick_step_size)
            elif self.manual_mode and key == ord('d'):
                # D = Pan RIGHT (joystick control in manual mode)
                self.handle_joystick_move('pan', self.joystick_step_size)
            elif self.manual_mode and key == ord('p'):
                # P = Move PAN servo by typed amount
                self.handle_typed_servo_move('pan')
            elif self.manual_mode and key == ord('t'):
                # T = Move TILT servo by typed amount
                self.handle_typed_servo_move('tilt')
            elif not self.manual_mode and key == ord('w'):
                # Save angle history to file (only when not in manual mode)
                if len(self.angle_log) > 0:
                    self.save_angle_log_to_file()
                else:
                    print("\n⚠️  No angle history to save")
            elif key == ord('l'):
                # Load angle history from file
                print("\n📂 Enter filename to load (or press Enter for default): ", end='', flush=True)
                try:
                    filename = input().strip()
                    if not filename:
                        # List available files
                        import glob
                        files = glob.glob("angle_history_*.json")
                        if files:
                            print(f"   Available files: {', '.join(files)}")
                            print("   Enter filename: ", end='', flush=True)
                            filename = input().strip()
                        else:
                            print("   No angle history files found")
                            continue
                    self.load_angle_log_from_file(filename)
                except (EOFError, KeyboardInterrupt):
                    print("\n   ❌ Cancelled")
            elif self.manual_mode and (key == ord('p') or key == ord('t')):
                # Manual servo control (only works in manual mode)
                servo_type = 'pan' if key == ord('p') else 'tilt'
                self.handle_manual_servo_move(servo_type)
        
        if self.arduino_conn:
            try:
                self.arduino_conn.write(b"STOP\n")
                time.sleep(0.2)
                print("   Servos disabled on exit")
            except:
                pass
            self.arduino_conn.close()
        
        if self.cap:
            self.cap.release()
        
        cv2.destroyAllWindows()
        print("\n✅ System shutdown complete")


def main():
    """Main entry point"""
    print("="*70)
    print("🎯 SINGLE CAMERA ALIGNMENT SYSTEM")
    print("="*70)
    print("\n📖 Simplified version using only side camera")
    print("   Perfect for testing before two-stage alignment")
    print("="*70)
    
    system = SingleCameraAlignment(
        camera_id=1,  # Side camera
        arduino_port=None  # None = auto-detect or simulation
    )
    
    system.run()


if __name__ == "__main__":
    main()

