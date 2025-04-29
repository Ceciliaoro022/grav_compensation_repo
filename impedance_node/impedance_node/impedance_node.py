#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class ImpedanceNode(Node):
    def __init__(self):
        super().__init__('impedance_node')

        self.K = np.array([[600.0, 0], [0, 600.0]])  # Stiffness
        self.D = np.array([[48.98, 0], [0, 48.98]])  # Damping (always recalculate when stiffness is altered)

        #Can I change only damping for joint 2?? Without respecting the "formula"
        #For joint 2
        # 400 5625              400  400        400  400                400  400                                400  400
        # 40  150               40   70         40   100                40   200                                40   300
        # Not very different    Not the worst   A little bit better     Not so different from the last one      Worst than last one
        #                                                                                                       (it does not respect the 
        #                                                                                                       desired position)

        # 400  400                      600     600
        # 40   40                       48.98   48.98
        # Okay, but not great fot j2    Not for j2 great but maybe better.. 



        # State variables
        self.q_pos_current = np.zeros(9) #Change the number depending on the number of joints, for 5DoF (5)
        self.q_vel_current = np.zeros(9)
        self.gravity_torque_data = np.zeros(9)
        self.q_desired = None
        self.received_q_desired = False
        self.received_gravity = False


        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Subscriber for desired positions
        self.desired_pos_sub = self.create_subscription(Float64MultiArray, '/desired_positions', self.desired_position_callback, 10)

        #Subscriber for gravity torque
        self.gravity_torque_sub = self.create_subscription(Float64MultiArray, '/gravity_torques', self.gravity_torque_callback, 10) #rbdl uses /gravity_comp_torque and uses JointStates

        # Publisher for torque commands
        self.torque_pub = self.create_publisher(Float64MultiArray, '/joint_effort_controller/commands', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.001, self.compute_and_send_torque) #Every 1 milisec so 0.001


        self.get_logger().info('Gravity Compensation Node Initialized!')


    def joint_state_callback(self, msg):
        target_joint_names = ["joint1", "joint2"]

        try:
            indices = [msg.name.index(joint_name) for joint_name in target_joint_names] #For each joint_name inside the list target_joint_names, find its index in the msg (/joint_states)
            self.q_pos_current = np.array([msg.position[i] for i in indices]) #use the indices index as i 
            self.q_vel_current = np.array([msg.velocity[i] for i in indices])

            if self.q_desired is None:
                self.q_desired = self.q_pos_current.copy() #If no q_desired, use current position as command
    
        except ValueError as error_:
            self.get_logger().error(f"One or more joint names not found in /joint_states: {error_}")
            return
    


    def desired_position_callback(self, msg):
        self.get_logger().info(f"Received desired positions: {msg.data}")
        self.q_desired = np.array(msg.data)
        self.received_q_desired = True

    def gravity_torque_callback (self, msg):
        if len(msg.data) >= 2: #For rbdl use "effort", for pinocchio use "data"
            self.gravity_torque_data = np.array(msg.data[-2:])
            self.received_gravity = True

    def compute_and_send_torque(self):
    # This is to wait until desired position is received and joint state arrays have the 2 elements
        if (not hasattr(self, 'received_q_desired') or not self.received_q_desired or 
            self.q_pos_current.size < 2 or self.q_vel_current.size < 2): 
            if not getattr(self, '_warned_waiting', False):
                self.get_logger().warn("Waiting for desired position before computing torques...")
                self._warned_waiting = True  # Mark that the warning have been logged
            return  # Do nothing until data is available

        # Reset the flag once the data is available so the warning can show again if needed
        self._warned_waiting = False

        tau_impedance = self.K @ (self.q_desired - self.q_pos_current) - self.D @ (self.q_vel_current)
        # tau_impedance = self.K @ (self.q_desired - self.q_pos_current[-2:]) - self.D @ (self.q_vel_current[-2:])

        if not self.received_gravity:
            self.get_logger().warn("Waiting for gravity torque...")
            return
        tau_gravity = self.gravity_torque_data
        # tau_gravity = self.gravity_torque_data[-2:]

        tau_final = tau_gravity + tau_impedance

        self.get_logger().info(f"Publishing efforts (torques): {tau_final}")
        #self.get_logger().info(f"q: {self.q_pos_current[-2:]}, qd: {self.q_desired}, τ: {tau_final}")


        torque_msg = Float64MultiArray()
        torque_msg.data = tau_final.tolist()
        self.torque_pub.publish(torque_msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceNode()
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()


if __name__ == '__main__':
    main()