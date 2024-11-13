# alexa-robocar

In this project I'm planning to implement in LEGO robocar all things I've learned from Coursera “Control of Mobile Robots” using [PySimiam](http://pysimiam.sourceforge.net/coursera.html).

### Building robocar and software setup
1. You need [LEGO MINDSTORMS EV3 Set](https://www.lego.com/en-us/product/lego-mindstorms-ev3-31313)
2. I've built a two wheels differential drive model with IR Sensor installed on top of medium servo motor. You can make modifications to this model, the code will need some small changes:
   - measure distance between wheels and change `DIST_BTW_WHEELS` in `robocar.py`
   - if you use tires other than EV3Tire - change tires class in `__init__(self)` in `robocar.py`
   - measure distance from the center of medium motor to the middle point between two wheels and change `self.sensor_rotation_point` in `robots/legobot.py`
   - measure distance from the center of medium motor to the IR Sensor bulb and change `self.sensor_rotation_radius` in `robots/legobot.py`

![Roboduck model](/pics/IMG_20200120_141340.jpg)

### Software setup
1. Please follow all steps from [ev3dev instructions](https://www.ev3dev.org/docs/getting-started/)
2. Connect to the EV3 via SSH and install additional libraries. It takes some time.
   ```
   sudo apt update
   sudo apt-get install python-numpy
   ```
3. For development you can use [Visual Studio Code](https://code.visualstudio.com/download) with extension [ev3dev Browser](https://github.com/ev3dev/vscode-ev3dev-browser)
4. Download and send files from this repository to the device
5. For more convenient development I would also recommend locally install [python-ev3dev library](https://python-ev3dev.readthedocs.io/en/ev3dev-stretch/index.html)

### Data flow
![data flow](/pics/dataFlow.jpg)
I implemented [FSM](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FSM) that switches to different behaviors depending on observed conditions:
   - [x] [GoToGoal Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/GoToGoal-Controller)
   - [x] [AvoidObstacles Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/AvoidObstacles-Controller)
   - [ ] [Blended (GoToGoal + AvoidObstacles) Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Blending-Controller-(GTG-and-AO))
   - [ ] [FollowWall Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FollowWall-Controller) cw and ccw


### Class diagram

![class diagram](/pics/Architecture%20Diagram.jpg)

## Wiki links
   - [Unicycle and differential drive dynamics](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Unicycle-and-differential-drive-dynamics)
   - [Odometry](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Odometry)
   - [FSM](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FSM)
   - [PID Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/PID-Controller)
      - [GoToGoal Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/GoToGoal-Controller)
      - [AvoidObstacles Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/AvoidObstacles-Controller)
      - [Blended (GoToGoal + AvoidObstacles) Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/Blending-Controller-(GTG-and-AO))
      - [FollowWall Controller](https://github.com/CatUnderTheLeaf/alexa-robocar/wiki/FollowWall-Controller) cw and ccw
