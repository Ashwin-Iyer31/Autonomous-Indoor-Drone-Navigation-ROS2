#!/usr/bin/env python
#---------------------------------------------------
import threading
import rclpy
from .PID import PID
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler

global yawSetpoint, thrust, pitchSetpoint, rollSetpoint
yawSetpoint = 0
thrust = 1500
pitchSetpoint = 0
rollSetpoint = 0

class PoseSub(Node):
    def __init__(self):

        node = rclpy.create_node('Control')
        
        global err_rollPub, err_pitchPub, err_yawPub, velPub
        #initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>
        err_rollPub = node.create_publisher(Float32, 'err_roll', 1)
        err_pitchPub = node.create_publisher(Float32, 'err_pitch', 1)
        err_yawPub = node.create_publisher(Float32, 'err_yaw', 1)

        #initialte publisher velPub that will publish the velocities of individual BLDC motors
        velPub = node.create_publisher(Float64MultiArray, '/jmc/commands', 4)


        super().__init__('pose_sub')
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.listener_callback,
            1000)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global roll, pitch, yaw, err_roll, err_pitch, err_yaw

        f = Float64MultiArray()
        err_roll = Float32()
        err_pitch = Float32()
        err_yaw = Float32()

        index = 0
        try:
           index = msg.name.index('my_bot')
        except ValueError:
            pass
        orientationObj = msg.pose[index].orientation
        orientationList = [orientationObj.w, orientationObj.x, orientationObj.y, orientationObj.z]
        (roll, pitch, yaw) = (quat2euler(orientationList))

        #send roll, pitch, yaw data to PID() for attitude-stabilisation, along with 'f', to obtain 'fUpdated'
        #Alternatively, you can add your 'control-file' with other algorithms such as Reinforcement learning, and import the main function here instead of PID().
        (fUpdated, err_roll.data, err_pitch.data, err_yaw.data) = PID(roll, pitch, yaw, f, yawSetpoint, thrust, pitchSetpoint, rollSetpoint)

        velPub.publish(fUpdated)
        err_rollPub.publish(err_roll)
        err_pitchPub.publish(err_pitch)
        err_yawPub.publish(err_yaw)
        #self.get_logger().info('I heard: "%s"' % msg.data)

#--------------------------------------------------------------------------------------------------------------------------------------

class CmdVelSub(Node):
    def __init__(self):
        super().__init__('cmd_vel_sub')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        global yawSetpoint, thrust, pitchSetpoint, rollSetpoint
        yawSetpoint = (msg.angular.z*10.0)  #Left is positive
        thrust = (msg.linear.z*1000 + 1000) #Up is positive
        pitchSetpoint = (msg.linear.x*20.0) #Up is positive
        rollSetpoint = (msg.angular.x*10.0) #Left is positive

#--------------------------------------------------------------------------------------------------------------------------------------

def main(args=None):
    
    #Initiate the node that will control the gazebo model
    rclpy.init(args=args)
    
    #Subscribe to /gazebo/model_states to obtain the pose in quaternion form
    pose_sub = PoseSub()
    cmd_vel_sub = CmdVelSub()
    

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pose_sub)
    executor.add_node(cmd_vel_sub)
    
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        while rclpy.ok():
            None
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()
    
if __name__ == '__main__':
    main()