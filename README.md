# Autonomous Asset Tracker

A custom perception-to-actuation pipeline for an autonomous mobile robot (AMR). This project uses the **Webots** physics simulator, **OpenCV**, and **Python** to enable a robot to autonomously detect, track, and intercept logistics assets (red and green targets) in its environment.

### Project Overview
Instead of relying on pre-built navigation stacks, this project implements a custom closed-loop control system from the ground up. The robot processes raw camera frames to isolate targets, calculates steering corrections based on visual error, and executes mission-based task switching using a finite state machine.

### Key Features
* **Perception:** Real-time HSV color-space thresholding to isolate target assets from environmental noise.
* **Closed-Loop Control:** A proportional (P) controller that translates 2D pixel error into differential steering velocities for smooth target interception.
* **State Machine:** Logic to handle mission cycling, allowing the robot to hunt for multiple targets sequentially.

### Technical Stack
* **Simulator:** Webots
* **Language:** Python
* **Vision:** OpenCV (cv2)
* **Hardware Interface:** Webots Robot Controller API

### Getting Started
1. **Prerequisites:** Ensure you have [Webots](https://cyberbotics.com/) installed.
2. **Setup:** Clone this repository into your Webots project folder.
3. **Execution:** Open the world file in Webots and ensure the robot controller is linked to this Python script.
4. **Controls:** The robot will automatically begin scanning and tracking once the simulation is started. Press 'q' in the live feed window to terminate the script.
