# Programming a Real Self-Driving Car project

## Goal of the project

The goal was to implement ROS nodes with certain core functionalities of the autonomous vehicle system. This included Waypoint updation, traffic light detection and control. I used the workspace provided by udacity for my development.

[//]: # (Image References)

[image1]: ./examples/architecture.png "Architecture"
[image2]: ./examples/waypoint_updater.png "Waypoint Updater"
[image3]: ./examples/dbw_node.png "Drive by Wire"
[image4]: ./examples/tl_detection.png "Traffic Light detection"

## Architecture

Below is the architecture that was followed for the project. The architecture shows the nodes and the topics communicated between the nodes.
The main subsystems Perception, planning and control are handled with 6 nodes here.

![alt text][image1]

### Waypoint update partial

Since the vehicle position is available in the /current_pose node and the /base_waypoints which gives the waypoints of the path are also available from the waypoint loader, the planning of the next 200 waypoints was done using these subscribed topics.The Tree method from scipy library was used to create an lookup as suggested in walkthrough which can be queried to find the current location and to estimate the next locations. Initially, i started with 200 waypoints and at a frequency of 30Hz. After looking at the latency issue which ended in late updation of the waypoints and hence the vehicle moving out of the trajectory when starting camera, i reduced the waypoints to 100. Also, the frequency was modified to 10Hz. 

![alt text][image2]

### Drive by Wire node implementation

Next, i started with the implementation of the control part. I used the yaw controller class provided for the steering angle calculation.
PID controller class was used for throttle control. The values for the parameters were manually tuned to Kp = 0.3, Kd=0.1 , Ki =0 . Before using the PID, i used the low pass filter to filter out the velocity incoming from the simulator as suggested in the walkthrough.
Finally, the brake force is calculated based on the required deceleration, vehicle_mass and wheel radius.  These commands can then be published to the topics /vehicle/throttle_cmd, /vehicle/brake_cmd and /vehicle/steering_cmd .

![alt text][image3]

### Traffic light detection

Then, i started with traffic light detection. First, i use the vehicle's location and the (x, y) coordinates for traffic lights to find the nearest visible traffic light ahead of the vehicle using the get_closest_waypoint method. This takes place in the process_traffic_lights method of tl_detector.py. The other part is to find the status of the traffic signal. I tried using the image thresholding technique to detect the red traffic signal before going to use the neural network techniques. I used the BGR image and performed thresholding which did not work well. However when i tried with the HSV format as suggested in a stack overflow discussion [https://stackoverflow.com/questions/30331944/finding-red-color-in-image-using-python-opencv], it worked out fine. Now the status and location of the traffic signal is passed through the traffic waypoint into the waypoint updater. 

![alt text][image4]

### Waypoint update full

With the traffic light detection working, the traffic light data is then subscribed into the waypoint updater node. I then found out the distance from the traffic waypoint and start decelerating at 0.5m/s2 before reaching the traffic waypoint. If the status changes to green, then the next waypoint is updated.

### Final working

After implementing all the nodes mostly with the help of walkthrough, i was finally able to see the car driving autonomously around the highway track provided. It stopped at the red signal and accelerated in time after the signal shifted to green.


### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
