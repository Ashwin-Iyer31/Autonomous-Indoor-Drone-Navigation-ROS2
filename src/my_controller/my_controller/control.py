#!/usr/bin/env python
#---------------------------------------------------

import rclpy
from .PID import PID
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose
from transforms3d.euler import quat2euler
from sensor_msgs.msg import Joy

global yawSetpoint, thrust, pitchSetpoint, rollSetpoint, kp, ki, kd
yawSetpoint = 0
thrust = 1500
pitchSetpoint = 0
rollSetpoint = 0

kp = 10
ki = 0.0002
kd = 3.8


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
        orientationList = [orientationObj.x, orientationObj.y, orientationObj.z, orientationObj.w]
        (roll, pitch, yaw) = (quat2euler(orientationList))
    
        #send roll, pitch, yaw data to PID() for attitude-stabilisation, along with 'f', to obtain 'fUpdated'
        #Alternatively, you can add your 'control-file' with other algorithms such as Reinforcement learning, and import the main function here instead of PID().
        (fUpdated, err_roll.data, err_pitch.data, err_yaw.data) = PID(roll, pitch, yaw, f, yawSetpoint, thrust, pitchSetpoint, rollSetpoint, kp, ki, kd)

        velPub.publish(fUpdated)
        err_rollPub.publish(err_roll)
        err_pitchPub.publish(err_pitch)
        err_yawPub.publish(err_yaw)
        #self.get_logger().info('I heard: "%s"' % msg.data)

#--------------------------------------------------------------------------------------------------------------------------------------

#def JoyData(msg4):
#    global yawSetpoint, thrust, pitchSetpoint, rollSetpoint
#    yawSetpoint = (msg4.axes[0]*10.0)  #Left is positive
#    thrust = (msg4.axes[1]*500 + 1500) #Up is positive
#    pitchSetpoint = (msg4.axes[2]*10.0) #Up is positive
#    rollSetpoint = (msg4.axes[3]*10.0) #Left is positive

#--------------------------------------------------------------------------------------------------------------------------------------

def main(args=None):
    
    #Initiate the node that will control the gazebo model
    rclpy.init(args=args)
    

    #Subscribe to /gazebo/model_states to obtain the pose in quaternion form
    pose_sub = PoseSub()
    #joyData = node.create_subscription(Joy, '/joy', JoyData)

    rclpy.spin(pose_sub)
    #rclpy.spin(joyData)

if __name__ == '__main__':
    main()