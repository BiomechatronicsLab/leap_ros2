#!/usr/bin/env python3

# ros packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np

import rtde_receive
import rtde_control
import time

# the only requirement for this demo is also running the ROBOTIQ driver
# run: ros2 launch robotiq_ros2 launch_robotiq.py
# also make sure you can connect to the robotiq by ping 192.168.1.50 and 
# the ur5e ping 192.168.1.60

class UWSS_RobustTest(Node):
    def __init__(self):

        super().__init__('test_skin_leap')

        # Leap hand controller
        # self.command_publisher = self.create_publisher(JointState, '/robotiq/command', 1)
        self.command_publisher = self.create_publisher(JointState, 'command_joint_states', 10)

        # Initialize joint angles
        self.joint_angles = [0.0] * 16
        self.grab_angles = np.array([0.0] * 16)         # Define angles for grabbing
        self.release_angles = np.array([0.0] * 16)      # Define angles for releasing


        # ur5e controller 
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.60")
        self.rtde_r = rtde_receive.RTDEReceiveInterface('192.168.1.60') # piece of code to add freq

    # Dont forget to /1000 (mm to m) if using tool pose!
    
    
    # Joint Poses (dont for get to convert rad!)
        # self.home_pose = [-40.49, -111.31, -96.17, -165.08, 23.69, 63.99]
        self.home_pose = [-53.25, -111.31, -96.17, -137.22, 23.69, 63.99]
        self.home_pose = np.array(self.home_pose) * np.pi / 180.0

        # self.start_pose = [-87.13, -136.67, -96.47, -102.65, 4.52, 34.31]
        self.start_pose = [-87.13, -137.78, -96.47, -93.61, 4.52, 58.63]
        self.start_pose = np.array(self.start_pose) * np.pi / 180.0
        
        # self.above_start_pose = [-87.13, -129.73, -96.47, -106.92, 5.93, 34.31]
        self.above_start_pose = [-87.13, -129.73, -96.47, -106.92, 5.93, 58.63]
        self.above_start_pose = np.array(self.above_start_pose) * np.pi / 180.0

        # self.raised_pose = [-87.13, -110.84, -95.45, -124.47, 5.94, 34.13]
        self.raised_pose = [-87.13, -110.84, -95.45, -124.47, 5.94, 58.63]
        self.raised_pose = np.array(self.raised_pose) * np.pi / 180.0
        
        # self.shake_down_pose = [-86.33, -120.52, -96.38, -121.36, 5.69, 33.75]
        self.shake_down_pose = [-86.33, -120.52, -96.38, -121.36, 5.69, 70.60]
        self.shake_down_pose = np.array(self.shake_down_pose) * np.pi / 180.0
        
        # self.shake_down_pose_1 = [-86.33, -120.52, -96.38, -121.36, 5.69, 65.13]
        # self.shake_down_pose_1 = [-86.15, -123.57, -91.11, -124.19, 5.84, 65.74]
        self.shake_down_pose_1 = [-86.15, -123.57, -91.11, -124.19, 5.84, 83.38]
        self.shake_down_pose_1 = np.array(self.shake_down_pose_1) * np.pi / 180.0
        
        # self.shake_down_pose_2 = [-86.33, -120.52, -96.38, -121.36, 5.69, 11.50]
        self.shake_down_pose_2 = [-86.33, -120.52, -96.38, -121.36, 5.69, 36.28]
        self.shake_down_pose_2 = np.array(self.shake_down_pose_2) * np.pi / 180.0
        
        # self.shake_up_pose = [-86.39, -105.44, -96.86, -128.40, 5.64, 23.26]
        # self.shake_up_pose = [-86.39, -105.44, -96.86, -128.40, 5.64, 70.60] #***
        self.shake_up_pose = [-86.33, -115.06, -92.00, -131.06, 5.69, 70.49]
        self.shake_up_pose = np.array(self.shake_up_pose) * np.pi / 180.0
        
        # self.shake_up_pose_1 = [-86.39, -105.44, -96.86, -128.40, 5.64, 65.13]
        self.shake_up_pose_1 = [-86.39, -105.44, -96.86, -128.40, 5.64, 83.38]
        self.shake_up_pose_1 = np.array(self.shake_up_pose_1) * np.pi / 180.0
        
        #self.shake_up_pose_2 = [-86.39, -105.44, -96.86, -128.40, 5.64, 11.50]
        # self.shake_up_pose_2 = [-85.73, -126.16, -73.00, -136.84, 6.18, 14.16]
        self.shake_up_pose_2 = [-85.73, -126.16, -73.00, -136.84, 6.18, 36.28]
        self.shake_up_pose_2 = np.array(self.shake_up_pose_2) * np.pi / 180.0

        # self.pull_start_pose = [-109.28, -137.41, -54.50, -166.39, -15.92, 56.39]
        self.pull_start_pose = [-109.28, -137.41, -54.50, -166.39, -15.92, 90.98]
        self.pull_start_pose = np.array(self.pull_start_pose) * np.pi / 180.0

        # self.pull_end_pose = [-95.86, -125.75, -75.18, -148.46, -2.47, 47.38]
        self.pull_end_pose = [-95.86, -125.75, -75.18, -148.46, -2.47, 82.20]
        self.pull_end_pose = np.array(self.pull_end_pose) * np.pi / 180.0
        
        self.inspect_pose = [-33.39, -100.84, -70.30, -196.69, 23.63, 145.21]
        self.inspect_pose = np.array(self.inspect_pose) * np.pi / 180.0


        self.home_position()
        self.grab_object()

        # self.start_position()
            
    
        # # Squeeze Test
        # self.get_logger().info("Squeeze Test...")
        # for _ in range(1):
        #     self.get_logger().info(f"Squeeze {_ +1}")
        #     self.run_routine_Squeeze()

        # self.home_position()
        # self.get_logger().info("Squeeze Test Completed. \nPress Enter to Continue.")
        # input()
        # self.get_logger().info("Continuing to next test...")
        

        # # Lift test
        # self.get_logger().info("Lift Test...")
        # for _ in range(1):
        #     self.get_logger().info(f"Lift {_ +1}")
        #     self.run_routine_Lift()
        # self.home_position()
        # self.get_logger().info("Lift Test Completed. \nPress Enter to Continue?")
        # input()
        # self.get_logger().info("Continuing to next test...")
        

        # # Shake Test
        # self.get_logger().info("Shake Test...")        
        # for _ in range(1):
        #     self.get_logger().info(f"Shake {_ +1}")
        #     self.run_routine_Shake()
        # self.home_position()
        # self.get_logger().info("Shake Test Completed.")


        # # Pull Test
        # self.get_logger().info("Pull Test...")        
        # for _ in range(1):
        #     self.get_logger().info(f"Pull {_ +1}")
        #     self.run_routine_PullBack()
        # self.home_position()
        # self.get_logger().info("Pull Test Completed.")

    
    def home_position(self): 

        # # Move robot to starting position
        # self.rtde_c.moveL_FK(self.start_pose, 0.2, 0.1) 

        self.release_object()

        # Move robot to home position
        self.rtde_c.moveL_FK(self.home_pose, 0.2, 0.1)  


    def grab_object(self):
        # self.grab_angles[[0, 4, 8]] = 0.0
        self.grab_angles[0] = 5.0
        self.grab_angles[4] = -3.0
        self.grab_angles[8] = -15.0
        self.grab_angles[[1, 5, 9]] = 90.0   #Fingers grip
        self.grab_angles[[2, 6, 10]] = 30.0
        self.grab_angles[[3, 7, 11]] = 15.0

        self.grab_angles[12] = 95.0
        self.grab_angles[13] = 3.0
        self.grab_angles[14] = -35.0     #Thumb grip
        self.grab_angles[15] = 60.0
        
        self.joint_angles = list(self.grab_angles)
        self.publish_joint_angles()

        time.sleep(1)


    def release_object(self):
        self.release_angles[[0, 4, 8]] = 0.0
        self.release_angles[[1, 5, 9]] = 30.0
        self.release_angles[[2, 6, 10]] = 30.0
        self.release_angles[[3, 7, 11]] = 0.0

        self.release_angles[12] = 100.0
        self.release_angles[13] = 0.0
        self.release_angles[14] = -50.0
        self.release_angles[15] = 30.0

        self.joint_angles = list(self.release_angles)
        self.publish_joint_angles()

        time.sleep(1)


    def publish_joint_angles(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = self.joint_angles
        self.command_publisher.publish(joint_state_msg)

        
        
    def start_position(self):	
        # Open Gripper  
        self.release_object()
             
        # Move robot to starting position
        self.rtde_c.moveL_FK(self.start_pose, 0.2, 0.1)  
        
    def squeeze_object(self):
        self.start_position()
        
        # Close Gripper  
        self.grab_object()	

        time.sleep(1)

        # Open Gripper
        self.release_object()
    
    
    def pick_up(self):		
        # Open Gripper and Move robot to starting position
        self.start_position()	
        
        # Close Gripper
        self.grab_object()
        
        # Move robot to raised position (to pick up object)
        self.rtde_c.moveL_FK(self.raised_pose, 0.2, 0.1)

        # time.sleep(4)
        
    
    def put_down(self):
        # Move robot to starting position
        self.rtde_c.moveL_FK(self.start_pose, 0.2, 0.1)
        
        # Open Gripper  
        self.release_object()	

    
    def shake_seq_0(self):
        # Move robot to shake object down(ward)
        self.rtde_c.moveL_FK(self.shake_down_pose, 0.2, 0.2)

        for _ in range(2):
            # Move robot to shake object up(ward)
            self.rtde_c.moveL_FK(self.shake_up_pose, 0.3, 0.4)
            # Move robot to shake object down(ward)
            self.rtde_c.moveL_FK(self.shake_down_pose, 0.3, 0.4)


    def shake_seq_1(self):
        # Move robot to shake object down(ward)
        self.rtde_c.moveL_FK(self.shake_down_pose_1, 0.2, 0.2)

        for _ in range(2):
            # Move robot to shake object up(ward)
            self.rtde_c.moveL_FK(self.shake_up_pose_1, 0.5, 0.7)
            # Move robot to shake object down(ward)
            self.rtde_c.moveL_FK(self.shake_down_pose_1, 0.5, 0.7)

    
    def shake_seq_2(self):
        # Move robot to shake object down(ward)
        self.rtde_c.moveL_FK(self.shake_down_pose_2, 0.2, 0.2)

        for _ in range(2):
            # Move robot to shake object up(ward)
            self.rtde_c.moveL_FK(self.shake_up_pose_2, 0.5, 0.7)
            # Move robot to shake object down(ward)
            self.rtde_c.moveL_FK(self.shake_down_pose_2, 0.5, 0.7)



    def shake_object(self):
            self.pick_up()

            time.sleep(2)
            self.get_logger().info("Beginning Shake...")
            time.sleep(1)

            # Shake seqence 0
            self.shake_seq_0()

            # Move robot to raised position (to pick up object)
            self.rtde_c.moveL_FK(self.raised_pose, 0.2, 0.1)

            # Shake seqence 1
            self.shake_seq_1()

            # Move robot to raised position (to pick up object)
            self.rtde_c.moveL_FK(self.raised_pose, 0.2, 0.1)

            # Shake seqence 2
            self.shake_seq_2()

            time.sleep(2)
            self.get_logger().info("Ending Shake...")
            time.sleep(1)

            self.put_down()

    def pull_back(self):
        # Move robot to start position for pulling back 
        self.rtde_c.moveL_FK(self.pull_start_pose, 0.2, 0.1)

        self.grab_object()

        # Move robot to end position for pulling back 
        self.rtde_c.moveL_FK(self.pull_end_pose, 0.2, 0.1)

        self.release_object()
        
         
    def inspect_UWSS(self):
        # Move robot to raised position (to clear object)
        self.rtde_c.moveL_FK(self.home_pose, 0.2, 0.1)
        
        
        # Move robot to inspect position
        self.rtde_c.moveL_FK(self.inspect_pose, 0.2, 0.1)
        
        self.get_logger().info("Waiting while inspecting...")
        self.get_logger().info("Press Enter to Continue...")
        input()
        
        self.get_logger().info("Please Confirm Again. Press Enter to Continue...")
        input()
        self.get_logger().info("Continuing...")

        
        # Move robot back to home position 
        self.rtde_c.moveL_FK(self.home_pose, 0.1, 0.1)
       
    
    def run_routine_Squeeze(self):
        for _ in range(5):
            # Squeeze
            self.squeeze_object()
        
        # Inspect
        self.inspect_UWSS()
        
        time.sleep(1)
         

    def run_routine_Lift(self):
        for _ in range(5):
            # Pick Up
            self.pick_up()

            time.sleep(4)
            
            # Put Down
            self.put_down()
        
        # Inspect
        self.inspect_UWSS()
        
        time.sleep(1)
        
        
    def run_routine_Shake(self):
        for _ in range(5):
            # Shake
            self.shake_object()
        
        # Inspect
        self.inspect_UWSS()
        
        time.sleep(5)
    

    def run_routine_PullBack(self):

        for _ in range(5):
            # Shake
            self.pull_back()
        
        # Inspect
        self.inspect_UWSS()
        
        time.sleep(5)


    def run_routine_Testing(self):

        for _ in range(2):
            # TESTING
            self.grab_object

            time.sleep(1)

            self.release_object()

            time.sleep(3)
        
        # # Inspect
        # self.inspect_UWSS()
        
        time.sleep(5)
   
        
def main(args=None):
    rclpy.init(args=args)
    UWSS_RT_node = UWSS_RobustTest()
    rclpy.spin(UWSS_RT_node)        #
    UWSS_RT_node.destroy_node() # this line is optional 
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
