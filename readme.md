# USI Angry Turtle Project

This is a ROS2 project, that was completed for the USI 2025 robotics lecture. The assignment was to make a ros2 turtlesim turtle draw USI and meanwhile attack other turtles, that are messing up the "painting". I opted to up the challange, by remaking the USI logo, by using edge detection with `cv2.Canny()` and `cv2.findContours()`.

## Results
A nice gif of the artist turtle on a drawing rampage ;)

![Angry Turtle in Action](media/angry_turtle.gif)

## Installation

I used pixi as a package manager and assume it is already installed:
```bash
cd ~/usi_angry_turtle_project
pixi shell
```



## Usage

```bash
pixi run colcon build  
pixi run ros2 launch usi_angry_turtle launch_draw_usi.py
```
