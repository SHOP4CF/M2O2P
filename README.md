# M2O2P

This repository for SHOP4CF Component M2O2P which stands for  
Multi-Modal Online and Offline Programming Solutions  

M2O2P enables natural human inputs by reading sensor data from CaptoGlove LLC  
sensor glove, and changing the raw sensor data to states and so on to gestures.  
These gestures are used to make commands which are POSTed to FIWARE and  
published to ROS2 topics, and additionally using Integration-Service  
as a bridge to ROS1 topics.  

Check the [introduction video](https://youtu.be/e0BxHBktAxE) for more information about the component.  

This repository stands as supportive repository for the component.  

The manual can be followed for setting up the component.  

- M2O2P_manual.pdf can be followed for setting up the component
- condiguration.json is used to configure docker images related to this component
- docker-compose.yml is used to launch the docker images
- original_limits.txt holds the user defined limits for bending and pressure of fingers
- ros2_ros1_command.yaml is used to configure Integration-Service
- captoglovesdk folder holds the .exe and configuration file for CaptoGloveSDK
- SHOP4CF.postman_collection.json has CRUD operations for testing the component
