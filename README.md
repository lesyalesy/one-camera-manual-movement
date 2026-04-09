Camera-Guided Needle Alignment (CV + Servos)
What this is

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
