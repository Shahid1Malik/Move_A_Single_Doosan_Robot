import rospy
import os
import csv
import threading, time
import sys
from time import sleep
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) )  # get import path : DSR_ROBOT.py

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0509"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

# Global variable for the CSV file path
csv_file_path = "/home/lab_user/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/robot_state_data.csv"

# Function to shutdown the robot
def shutdown():
    print("shutdown time!")
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

# Callback function to save robot state to CSV
def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if 0 == (msgRobotState_cb.count % 1): 
        rospy.loginfo("________ ROBOT STATUS ________")
        
        # Print information
        print("  robot_state           : %d" % (msg.robot_state))
        print("  current_posx          : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (
            msg.current_posx[0], msg.current_posx[1], msg.current_posx[2],
            msg.current_posx[3], msg.current_posx[4], msg.current_posx[5]
        ))
        
        # Check if the file exists and write header if it doesn't
        file_exists = os.path.isfile(csv_file_path)
        
        with open(csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            
            # Write header only if the file is being created
            if not file_exists:
                writer.writerow(['timestamp', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

            # Convert msg.current_posx to a list and concatenate with the timestamp
            current_time = rospy.Time.now()
            #timestamp_microseconds = int(secs * 1_000_000 + nsecs // 1000)
            timestamp_microseconds = int(current_time.to_sec() * 1e6)
            #rospy.loginfo(f"Secs: {secs}, Nsecs: {nsecs}, Timestamp: {timestamp_microseconds}")


 # Convert to microseconds
            writer.writerow([timestamp_microseconds] + list(msg.current_posx))

msgRobotState_cb.count = 0

# Thread function to subscribe to robot state topic
def thread_subscriber():
    rospy.Subscriber('/' + ROBOT_ID + ROBOT_MODEL + '/state', RobotState, msgRobotState_cb)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('single_robot_simple_py')
    rospy.on_shutdown(shutdown)
    
    # Initialize the robot mode
    set_robot_mode = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/system/set_robot_mode', SetRobotMode)
    t1 = threading.Thread(target=thread_subscriber)
    t1.daemon = True 
    t1.start()

    pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=10)

    # Set the robot mode to autonomous
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # Set global task speed and acceleration
    set_velx(30, 20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60, 40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    velx = [50, 50]
    accx = [100, 100]

    while not rospy.is_shutdown():
        posx_current = get_current_posx()
        
        # Move robot smoothly to next position
        next_posx = posx(490.476, 463.083, 557.459, 158.767, -107.682,  87.486)
        #next_posx = posx(810.518, -234.426, 396.316, 163.156, -102.192,  80.940)  #CURRENT POSE HOME POSIITON
        next_posx = posx(460.143, -230.300, 563.675, 161.317, -116.110,  82.826)
        duration = 3  # Move from p1 to p2 in 5 seconds
        #next_j = [245,-61,-19,279,-93,173]
        next_j = [170,-67,-19,328,24,122]
        #movej(next_j, v=10, t=5, a=20, r=10)
        movel(next_posx, v=20, a=5, r=10)

    print('good bye!')

