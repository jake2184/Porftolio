Dependencies
-------------

This is based on Ubuntu 14, with ROS Indigo + python installed


Core services
--------------
Python
core_node controls interaction flow
status collects and publishes status of the robot
android_link connects to tablet


Speech
-------
Python
git clone https://github.com/vijaravind/ros-gspeech (dependent on flac, PyAudio)
sudo apt-get install ros-indigo-pocketsphinx


Android
--------
Run android_link.py and connect to the IP address of the server/laptop from the Android tablet.
The default port is 8000 for the server.

User requests
--------------
This is a node that responds to user requests and provides navigational directions.

Pre-requisites:
`pip install -U googlemaps`
`pip install -U beautifulsoup4`
`pip install -U textblob`
`python -m textblob.download_corpora`

Obtain a Google Maps Web Service API key from https://developers.google.com/console and replace the dummy key provided in the software.

To run:
`rosrun user_request directionsHandler.py`

Movement
---------
Requires a Robot.

rosdep install rosaria-
Install the ARIA dependency

Source:
https://github.com/amor-ros-pkg/rosaria

rosrun rosaria RosAria -
Launch the node to enable connection between laptop and the robot, the default port is /dev/ttyUSB0. To specify the port, do rosrun rosaria RosAria _port:=/dev/ttyPORTNAME 

rosrun movement robotControl.py-
Launch the node for moving/rotating the robot

Vision
-------
Requires a Microsoft Kinect camera. Requires the following services to be running, in this order:

roslaunch openni_launch openni.launch - 
Launches all camera services.

rosrun openni_tracker openni_tracker - 
Launches the skeleton tracker application.

Source:
https://github.com/ros-drivers/openni_tracker

rosrun vision visionControl.py - 
Launches the node for the vision section of the robot.

Robot Beeping 
-------
If the PeopleBot is beeping, here are three possible causes:
1. p2os_driver is running but you do not send any commands
2. Battery voltage is low
3. The bot is in emergency mode, release the button