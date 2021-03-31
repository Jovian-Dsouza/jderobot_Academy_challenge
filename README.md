# JdeRobot Academy Challenge 2021

## YouTube video

### Amazon Warehouse Robot
[![follow line video](https://img.youtube.com/vi/_B97Z-RKh-Q/0.jpg)](https://youtu.be/_B97Z-RKh-Q)

#### Follow Line 
[![follow line video](https://img.youtube.com/vi/D28bFF6xgWk/0.jpg)](https://youtu.be/D28bFF6xgWk)

## How to setup JdeRobot Academy

Install
```bash
docker pull jderobot/robotics-academy
git clone https://github.com/JdeRobot/RoboticsAcademy
```

Run 
```bash
docker run -it -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 jderobot/robotics-academy python3.8 manager.py
```

Then open `RoboticsAcademy/exercises/<exercise_name>/web-template/exercise.html` in web Browser

## Follow line

![follow_line_GIF](follow_line/GIF.gif)

[RoboticsAcademy Link](https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/)

## Amazon Warehouse Robot

![Amazon Warehouse Robot](https://media.giphy.com/media/uMk4669ilaGPX8G80r/giphy.gif)

[RoboticsAcademy](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/single_robot_amazon_warehouse/)