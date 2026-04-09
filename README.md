Camera-Guided Needle Alignment (CV + Servos)
What this is:
This is a small prototype I built to test how a system can use a camera to guide a “needle” toward a target at the correct angle.

In my setup:
the yellow line = needle
the blue/green lines = targets (“hair”)
The idea is to align them in both angle and position, and then move the system using servos until they match.
This is part of a bigger exploration into camera-guided physical systems.

What it does:
Detects colored lines in real time (OpenCV)
Calculates angle + position of the needle vs target
Moves a pan/tilt system (servos) to align them
Can run:
manually (you control movement)
automatically (system tries to align itself)
without hardware (simulation mode)

Why I built this:
I wanted to understand how to go from:
“camera sees something” → “physical system moves correctly”
This turned out to be much harder than expected because:
camera view ≠ real-world geometry
small errors compound quickly
direction can be wrong even when logic seems correct
So this project is basically me working through those problems step by step.

How it works (simple version):
Camera detects objects by color
Extracts lines and calculates their angle
Compares needle vs target
Decides which way to move
Sends commands to Arduino → servos move
Repeats until aligned

Files (what’s where):
single_camera_alignment.py
→ main logic using one camera (side view)
two_camera_manual_alignment.py
→ both cameras + manual control
two_camera_auto_alignment.py
→ more automated version
dual_camera_viewer.py
→ just visualization / debugging
arduino_uno_pca9685_servo.ino
→ controls pan/tilt servos
*.json
→ color calibration (HSV ranges)

Hardware I used
Arduino Uno
PCA9685 servo driver
2x small servos (pan/tilt)
USB camera(s)

What’s still rough:
Color detection depends a lot on lighting
No depth estimation yet
Alignment is not perfectly stable
Some logic is still experimental
What I want to improve next:
Use 2 cameras properly for depth (parallax)
Make alignment more reliable
Clean up control logic (maybe PID)
Eventually connect this to a more complete robotic system
