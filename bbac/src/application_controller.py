#!/usr/bin/env python

import socket
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import rclpy
import time
import json
import copy
import psycopg2
import threading
from rclpy.node import Node
import sys

class ApplicationController(Node):
    def __init__(self):
        ### INITIALIZING ROS2 NODE ###
        super().__init__('application_controller')
        
        ### INITIALIZING PUBLISHERS ###
        self.pub_task_completion = self.create_publisher(String, 'task_completion', 10) # Sends message to bridge to complete task entity
        self.pub_task_info = self.create_publisher(String, 'task_info', 10) # Publishes task information to frontend
        self.pub_command_id = self.create_publisher(String, 'command_id', 10) # publishes the command id to bridge to update the device entity

        ### INITIALIZING SUBSCRIBERS ###
        self.subscription_taskname = self.create_subscription( # Subscribe new tasks from FIWARE
            String,
            'taskname',
            self.listener_callback_taskname,
            10)                  
        self.subscription_status_change = self.create_subscription( # Subscribe messages if frontend task completion button is used
            String,
            'status_change',
            self.listener_callback_status_change,
            10)
        self.subscription_completion_time = self.create_subscription( # Subscribe messages if some other component changes status
            String,
            'status_completion_time',
            self.listener_callback_completion_time,
            10)
        
        ### DEFINITION OF CLASS VARIABLES ###
        self.msg = String()
        self.publish_all_commands_mode = False
        self.last_task = ""
        self.timeStamp = ""
        self.task_identifier = ""
        self.command_id = 0
        self.command_id_to_be_sent = ""
        self.on_going_task = False
        self.task_name = ""
        self.taskId = ""
        self.reset = ""
        
    ######### DEFINITION OF SUBSCRIBER CALLBACKS #########
    
    def listener_callback_taskname(self, msg):
        line = msg.data
        # Check if filtering mode is on, if it is
        if not self.publish_all_commands_mode:
            # check that the same task is not given twice.
            if self.last_task != line:
                # If not, then get the task information out of the message (json message)
                self.last_task = line
                line_dict = json.loads(line)
                if line_dict["taskName"] != "": # Check if "empty" task is not given
                    self.taskId = line_dict["taskName"]
                    self.timeStamp = line_dict["timeStamp"]
                    self.task_identifier = line_dict["id"]
                    if self.operating_mode == 0:
                        self.get_task_info()
                    elif self.operating_mode == 1: # If the required task information is provided in the task entity
                        self.task_name = line_dict["taskDescription"]
                        self.command_id = line_dict["commandId"]
                    else:
                        print("Select operating mode to be 0 if the additional information is provided through PostgreSQL and 1 if all information is provided in Task entity")
                        print("Exiting...")
                        sys.exit(0)
                    self.logger("Received task with description: " + self.task_name)
                    time.sleep(0.2)
                    if self.command_id != 0:  
                        self.gesture_to_be_done = self.command_map[int(self.command_id)-1][1]
                    else:
                        self.gesture_to_be_done = "none"
                    self.command_id_to_be_sent = str(self.command_id)
                    # Message to frontend
                    message = {
                        "taskName":self.task_name,
                        "gesture":self.gesture_to_be_done,
                        "timeStamp":str(self.timeStamp)
                    }
                    json_msg = json.dumps(message)
                    self.msg.data = json_msg
                    self.pub_task_info.publish(self.msg)
                    time.sleep(0.1)
                    # Update the status of Task entity to "inProgress" now that the user
                    # is notified and we successfully got the task information.
                    message = {
                        "value":"inProgress",
                        "id":self.task_identifier
                    }
                    json_msg = json.dumps(message)
                    self.msg.data = json_msg
                    self.logger("Updating the 'status' of Task entity to: " + self.msg.data)
                    self.pub_task_completion.publish(self.msg)
                    self.on_going_task = True # Change on going task to true for later usage
            else: # Update frontend if empty task is given, aka some other component completed the task
                self.task_name = ""
                self.command_id = ""

    # Handles if status is changed by the webapp and notifies the bridge
    # about it
    def listener_callback_status_change(self, msg):
        line = msg.data
        if self.on_going_task:
            message = {
                "value":line,
                "id":self.task_identifier
            }
            json_msg = json.dumps(message)
            self.msg.data = json_msg
            self.logger("Updating the 'status' of Task entity with id '" + message["id"] + "' with value of '" + message["value"]+ "'.")
            self.pub_task_completion.publish(self.msg)
            if line == "paused":
                paused = True
            elif line == "inProgress":
                paused = False
            if line == "completed":
                self.logger("Updating the 'status' of Task entity with id '" + \
                    message["id"] + "' with value of '" + \
                    message["value"] + "' and 'value' of Device entity to '" + \
                    self.command_id_to_be_sent + "'.")
                message = {
                    "taskName":"",
                    "gesture":"",
                    "timeStamp":""
                }
                json_msg = json.dumps(message)
                self.msg.data = json_msg
                self.pub_task_info.publish(self.msg)
                self.msg.data = self.command_id_to_be_sent
                self.pub_to_ros1.publish(self.msg)
                self.pub_command_id.publish(self.msg)
                self.on_going_task = False
                self.paused = False
    ######################################################
    def status(self, fingers):
        # Update statuses of each finger to the dictionary 
        # regarding the thresholds.
        return fingers

    def recognize_gesture(self, fingers):
       
        # check if statuses corresponds a gesture
        print("recognize_gesture")

    # Publishes information to WebApp UI
    def logger(self, to_be_printed):
        #print(to_be_printed)
        msg_logger = String()
        msg_logger.data = to_be_printed
        #print("Publishing: " + msg.data + " to webapp.")
        self.pub_output.publish(msg_logger)

    # Retrieves the configuration from configuration.json file
    def retrieve_configuration(self, path):
        f = open(path)
        data_dict = json.load(f)
        
        self.query = data_dict["config_ac"]["query"]
            
        self.postgres_dict = copy.deepcopy(data_dict["config_ac"]["postgres"])

        self.operating_mode = data_dict["operating_mode"]
        
        f.close()
        #return glove_amount, command_map, postgres_dict, query

    # Saves sensor data to "fingers" dictionary
    def retrieve_fingers(self, fingers, left):
        
        # Here the fingers dictionary that is initialized at the beginning is 
        # populated with bending and pressure values.
        
        return fingers

    # Processes through the glove sensor data
    def extract_sensor_values(self):

        # Gets request from the tcp ip connection, extracts the sensor values from the request and 
        # changes the bool value of that hand to true if the message had right/left in the name.
        print("extract_sensor_values")
    
    def initialize_limits(self):

        # Read limits from original limits file, populate limits, original limits and backup limits
        # notify frontend about the limits
        print("initialize_limits")

    def create_socket_connection(self):
        # Create socket connection to one 
        # or two gloves depending on the configuration
        print("create_socket_connection")

    
    def init_postgres(self):
        # Connect to Postgres database and retrieve version information (test connection at the start so we can be sure the connection exists later on)
        # Populates the postgresql with needed data when testing the component
        print("init_postgres")
    
    
    def socket_listener(self):
        
        # Main loop that handles reading the glove data and acting on it     
        
        #Inifite loop reading the clients messages
        while True:
            self.extract_sensor_values()
            
            # To keep the loop running:
            self.text_left_bool = True

            # If the the right glove is connected, do this
            if self.text_right_bool:
                # Functions are universal for both fingers so with bool the function knows to which dictionary to save the changes
                left = False
                
                # Transform text to joint values and update right fingers
                self.right_fingers = self.retrieve_fingers(self.right_fingers, left)

                # Update the statuses of the right fingers
                self.right_fingers = self.status(self.right_fingers)
                
                # Determine if current joint value statuses corresponds a gesture
                self.recognize_gesture(self.right_fingers)

            # If the left glove is connected, do this
            elif self.text_left_bool:
                # We want to change left fingers
                left = True
            
                # Transform text to joint values and update left fingers
                self.left_fingers = self.retrieve_fingers(self.left_fingers, left)
                
                # Update the statuses of the left fingers
                self.left_fingers = self.status(self.left_fingers)
                
                # Determine if current joint value statuses corresponds a gesture
                self.recognize_gesture(self.left_fingers)

            if self.text_left_bool or self.text_right_bool:
                # Original functionality:
                # Wait 500ms if the gesture is held
                #   If the gesture is right, notify user to hold another 1000ms
                #   otherwise notify the user that the gesture is wrong or that there is no gesture to be done
                # After held for 1500ms total, update the task entity as completed
                print("main loop")
                time.sleep(10)

def start():
    if not rclpy.ok():
        rclpy.init()
    time.sleep(1)

    # Initialize the ApplicationController ROS2 node
    node = ApplicationController()
    
    # Initialize bending and pressure limits
    node.initialize_limits()
    
    # Create socket connection
    node.create_socket_connection()
    
    # Check that postgres can be reached and make TaskDef database if it is not made by other components
    node.init_postgres()
    
    # Create a thread that listens a port, where the sensorvalues are sent
    socket_listener_thread = threading.Thread(target=node.socket_listener,args=())
    
    # Start the thread
    socket_listener_thread.start()
    
    # Spin the node while the node
    while not node.stop_main:
        rclpy.spin_once(node)
    del node
      
def main(args=None):
    # Start the AC with start function
    start()


if __name__ == '__main__':
    main()
