# alexa-robocar
>Initially I wanted to make a LEGO Mindstorms EV3 robocar which can be controlled with Amazon Alexa voice commands, Alexa related stuff is still not implemented.

In this project I'm planning to implement in LEGO robocar all things I've learned:
  1. From [“Control of Mobile Robots”](https://www.coursera.org/learn/mobile-robot) using [PySimiam](http://pysimiam.sourceforge.net/coursera.html). See my [repository](https://github.com/CatUnderTheLeaf/pySimIAm)
     - [x] [Unicycle and differential drive dynamics](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Unicycle-and-differential-drive-dynamics), and transformation between them
     - [x] [Odometry](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Odometry)
     - [x] [PID Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/PID-Controller)
     - [FSM](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FSM) that switches to different behaviors depending on observed conditions:
       - [x] [GoToGoal Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/GoToGoal-Controller)
       - [x] [AvoidObstacles Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/AvoidObstacles-Controller)
       - [ ] [Blended (GoToGoal + AvoidObstacles) Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Blending-Controller-(GTG-and-AO))
       - [ ] [FollowWall Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FollowWall-Controller) cw and ccw
  2. From ["Artificial Intelligence for Robotics"](https://classroom.udacity.com/courses/cs373). See my [repository](https://github.com/CatUnderTheLeaf/udacity_Runaway-Robot)
     - [ ] Localization
     - [ ] Kalman or Particle Filters
     - [ ] Path planning
     - [ ] SLAM
  3. Other stuff
     - Alexa skills and commands
     - Machine Learning and Computer Vision
       - [ ] Lane Finding
       - [ ] Traffic Sign Classification
     - ROS

## Wiki links
   - [Building robocar and software setup](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Building-robocar-and-software-setup)
   - [Architecture](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Architecture)
   - [Unicycle and differential drive dynamics](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Unicycle-and-differential-drive-dynamics)
   - [Odometry](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Odometry)
   - [PID Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/PID-Controller)
   - [FSM](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FSM)
   - [GoToGoal Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/GoToGoal-Controller)
   - [AvoidObstacles Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/AvoidObstacles-Controller)
   - [Blended (GoToGoal + AvoidObstacles) Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Blending-Controller-(GTG-and-AO))
   - [FollowWall Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FollowWall-Controller) cw and ccw
