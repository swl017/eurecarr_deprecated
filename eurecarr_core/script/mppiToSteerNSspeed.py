#!/usr/bin/env python

import rospy
from autorally_msgs.msg import chassisCommand, runstop
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np

class MppiToSteerNSpeed(object):
    def __init__(self):
        # self.sub_mppi = rospy.Subscriber("hello", chassisCommand, self.mppiSubCallback)
        self.sub_odom = rospy.Subscriber("pose_estimate", Odometry, self.odomSubCallback)
        self.sub_steer = rospy.Subscriber("mppi_controller/debug/steerSequence", Float64MultiArray, self.steerSquenceSubCallback)
        self.sub_speed = rospy.Subscriber("mppi_controller/debug/stateSequence", Float64MultiArray, self.speedSquenceSubCallback)
        self.sub_runstop = rospy.Subscriber("runstop", runstop, self.runstopSubCallback)
        self.pub_cmd   = rospy.Publisher("mppi_controller/chassisCommand", chassisCommand, queue_size=1)
        self.pub_debug = rospy.Publisher("debug/cmd_cur", Float64MultiArray, queue_size=1)
        self.pub_speed = rospy.Publisher("debug/speedSequence", Float64MultiArray, queue_size=1)

        self.steer_cmd    = 0
        self.throttle_cmd = 0
        self.speed_cmd    = 0
        self.speed_cur    = 0

        self.dt_mppi           = 0.01 # s
        self.t_steerAdvance    = 0.1  # s
        self.t_speedAdvance    = 0.1
        self.step_steerAdvance = int(self.t_steerAdvance/self.dt_mppi)
        self.step_speedAdvance = int(self.t_speedAdvance/self.dt_mppi)
        self.mppi_stateDim     = 7

        self.runstop     = False
        self.error       = 0
        self.error_accum = 0
        self.error_der   = 0
        self.error_last  = 0
        self.Kbat        = 1
        self.Kp          = 0.35
        self.Ki          = 0.036
        self.Kd          = 0.0001

        self.dt_speedPID     = 0
        self.speed_time      = 0
        self.last_speed_time = 0
        

    def mppiSubCallback(self, msg):
        self.steer_cmd = msg.steering
        self.throttle_cmd = msg.throttle

    def odomSubCallback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_cur = np.sqrt(vx**2 + vy**2)

    def steerSquenceSubCallback(self, msg):
        self.steer_cmd = msg.data[self.step_steerAdvance]

    def speedSquenceSubCallback(self, msg):
        self.speed_cmd = msg.data[self.mppi_stateDim*self.step_speedAdvance + 4]
        speedSequence = Float64MultiArray()
        speedSequence.data = [0]*(int(len(msg.data)/self.mppi_stateDim)-1)
        for i in range(0, len(speedSequence.data)):
            speedSequence.data[i] = msg.data[self.mppi_stateDim*i + 4]
        self.pub_speed.publish(speedSequence)

    def runstopSubCallback(self, msg):
        self.runstop = msg.motionEnabled

    def speedPIDcontroller(self, ref, ego):
        self.speed_time = rospy.get_time()
        self.dt_speedPID = np.clip(self.speed_time - self.last_speed_time, 0.005, 0.05)
        self.last_speed_time = self.speed_time

        self.error = ref - ego
        if self.runstop:
            self.error_accum = np.clip(self.error_accum + self.error, -1, 1)
        else:
            self.error_accum = 0
        self.error_der = np.clip((self.error - self.error_last) / self.dt_speedPID, -10, 10)

        control = np.clip(self.Kbat * (self.Kp*self.error + self.Ki*self.error_accum + self.Kd*self.error_der), -1, 1)

        msg = Float64MultiArray()
        msg.data    = [0] * 4 # steer cmd, speed cmd, cur, throttle cmd
        msg.data[0] = self.steer_cmd
        msg.data[1] = ref
        msg.data[2] = ego
        msg.data[3] = control

        self.pub_debug.publish(msg)

        return control

    def publishCommand(self):
        steer    = self.steer_cmd
        throttle = self.speedPIDcontroller(self.speed_cmd, self.speed_cur)

        msg = chassisCommand()
        msg.header.stamp = rospy.Time.now()
        msg.sender       = "mppi_controller"
        msg.steering     = steer
        msg.throttle     = throttle

        self.pub_cmd.publish(msg)



def main():
    rospy.init_node("mppiToSteerNSpeed", anonymous=True)
    # prefix = rospy.get_namespace()
    # if (len(prefix) > 2):
    #     prefix = prefix[1:-1] + "/"
    # print("prefix = ", prefix[0:-1])
    mppiSS = MppiToSteerNSpeed()
    # mppiSS.prefix = prefix
    rate = rospy.Rate(80) #  Hz
    while not rospy.is_shutdown():
        mppiSS.publishCommand()
        rate.sleep()

if __name__ == "__main__":
    main()    
