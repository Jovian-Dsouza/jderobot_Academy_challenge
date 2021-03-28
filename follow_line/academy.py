from GUI import GUI
from HAL import HAL
import numpy as np
import cv2

lower = np.array([0, 0, 118], dtype = "uint8")
upper = np.array([50, 56, 255], dtype = "uint8")

linear_vel = 1
kp = 1/ 700.0
kd = 1 /150.0

prev_err = 0.0

while True:
    # Enter iterative code!
    
    image =  HAL.getImage()
    
    h, w, d = image.shape
    top = int((3/4.0) * h)
    bottom = top + 22
    
    crop_img = image[top:bottom,:]
    mask = cv2.inRange(crop_img, lower, upper)
    output = cv2.bitwise_and(crop_img, crop_img, mask = mask)
    
    
    
    M = cv2.moments(mask)
    if M['m00'] != 0 :
        x = int(M['m10']/M['m00'])
        y = int(M['m01']/M['m00'])
        cv2.circle(crop_img, (x, y), 21, (50, 168, 64), -1)
        
        e = -float(x - w/2)
        console.print(e)
        de = e - prev_err
        prev_err = e
        
        angular_vel = kp * e +kd *de 
        HAL.motors.sendV(linear_vel)
        HAL.motors.sendW(angular_vel)
        console.print("v = %0.3f, w = %0.3f" % (linear_vel, angular_vel))
    
    else: 
        HAL.motors.sendV(0)

         
    GUI.showImage(image)
        