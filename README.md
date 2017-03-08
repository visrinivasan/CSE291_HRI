# CSE291_HRI





## Notes
Using OpenCV with cv_bridge:

sudo apt-get install ros-indigo-cv-bridge


Using pocketsphinx:

To install the package - 

  sudo apt-get install ros-indigo-pocketsphinx

To install the package dependencies - 

  sudo apt-get install gstreamer0.10-pocketsphinx

  sudo apt-get install python-gst0.10

  sudo apt-get install gstreamer0.10-gconf

To run - 
roslaunch pocketsphinx turtlebot_voice_cmd.launch


### Nodes
robot_soccer.py: controls state (which task robot is doing)


### Publishers
color_detect.py: detects specified color and returns centroid of detected color blob


