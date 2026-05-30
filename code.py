import cv2 
import numpy as np
from controller import Robot 

robot = Robot()
time_step = int(robot.getBasicTimeStep())

camera = robot.getDevice('camera')
camera.enable(time_step)
cam_width = camera.getWidth()

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

MAX_SPEED = 6.28  

missions = ["RED", "GREEN"]
mission_index = 0

cv2.namedWindow("Tracking View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Tracking View", 400, 200) # Directly forces a large layout canvas window

while robot.step(time_step) != -1:

    raw_image = camera.getImage()
    if raw_image is None:
        continue        
    frame = np.frombuffer(raw_image, dtype=np.uint8).reshape((camera.getHeight(), cam_width, 4))
    bgr_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2

    lower_green, upper_green = np.array([40, 100, 50]), np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    
    active_index = mission_index % len(missions)
    current_target = missions[active_index]
    
    if current_target == "RED":
        target_mask = red_mask
        text_color = (0, 0, 255) 
    elif current_target == "GREEN":
        target_mask = green_mask
        text_color = (0, 255, 0) 
        
    left_speed = 1.5
    right_speed = -1.5
    
    if target_mask is not None:
        contours, _ = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest_contour)
            
            if contour_area > 10:
                x, y, w, h = cv2.boundingRect(largest_contour)
                object_center_x = x + (w / 2)
                image_center_x = cam_width / 2
                
                cv2.rectangle(bgr_frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
                cv2.circle(bgr_frame, (int(object_center_x), int(y + h/2)), 1, (255, 0, 0), -1)
              
                error = object_center_x - image_center_x 
                steering_gain = 0.06
                correction = error * steering_gain
                
                base_forward_speed = 4.0
                left_speed = base_forward_speed + correction
                right_speed = base_forward_speed - correction

                if contour_area > 1900:
                    print(f"Target reached! Switching missions away from {current_target}")
                    mission_index += 1  
                    
                left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
                right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    big_width = int(cam_width * 10)
    big_height = int(camera.getHeight() * 10)
    large_frame = cv2.resize(bgr_frame, (big_width, big_height), interpolation=cv2.INTER_NEAREST)
    cv2.putText(large_frame, f"HUNTING: {current_target}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
    cv2.imshow("Tracking View", large_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
