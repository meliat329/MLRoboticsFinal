#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl

'''
MotorIDstrings for Hexapod follow the convention legN_jM, where N can be 1,2,3,4,5,6 (for each of the 6 legs) and M can 1,2,3. M=1 is the joint closest to the body and M=3 is the joint farthest from the body.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['front', 'left', 'right'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time

Helper functions

degToRad() - Converts degrees to radians

Note that, this list of API functions could potentially grow. You will be notified via Canvas if anything is updated

'''

class HexapodControl(RobotControl):
    def __init__(self):
        super(HexapodControl, self).__init__(robot_type='hexapod')
        rospy.loginfo("hexapod setup complete")
        self.holdNeutral()
        time.sleep(2.0)




        #main control loop
        while not rospy.is_shutdown(): 
            # self.hold_neutral() #remove if not necessary
            # ---- add your code for a particular behavior here ----- #

            #if all sensors are triggered turn around
            #includes a 1cm margin of error for the right and left sensors
            #increase to move faster; decrease to be more precise
            if (( -1 < self.getSensorValue('front') < .188) and ( -1 < self.getSensorValue('right') < .33) and ( -1 < self.getSensorValue('left') < .33)):
                self.turnAround180()
                time.sleep(2)

            #if front and right sensor are triggered turn left
            #includes a 1cm margin of error for the right sensor
            #increase to move faster; decrease to be more precise
            elif (( -1 < self.getSensorValue('front') < .188) and ( -1 < self.getSensorValue('right') < .33)):
                self.turnLeft90()
                time.sleep(1)

            #if front and left sensor are triggered turn right
            #includes a 1cm margin of error for the left sensor
            #increase to move faster; decrease to be more precise
            elif ((-1 < self.getSensorValue('front') < .188) and (-1 < self.getSensorValue('left') < .33)):
                self.turnRight90()
                time.sleep(1)

            #if the hexapod is too close on the left side make a small turn to the right and move forward
            #includes a 1cm margin of error for the left sensor
            #increase to move faster; decrease to be more precise
            if -1 < self.getSensorValue('left') < 0.31:
                self.setMotorTargetJointPosition('leg1_j2', -0.55)
                self.setMotorTargetJointPosition('leg3_j2', -0.55)
                self.setMotorTargetJointPosition('leg5_j2', -0.55)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j1', -0.05)
                self.setMotorTargetJointPosition('leg3_j1', -0.05)
                self.setMotorTargetJointPosition('leg5_j1', -0.05)
                self.setMotorTargetJointPosition('leg1_j2', -0.5)
                self.setMotorTargetJointPosition('leg3_j2', -0.5)
                self.setMotorTargetJointPosition('leg5_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j2', -0.55)
                self.setMotorTargetJointPosition('leg4_j2', -0.55)
                self.setMotorTargetJointPosition('leg6_j2', -0.55)
                self.setMotorTargetJointPosition('leg1_j1', 0.0)
                self.setMotorTargetJointPosition('leg3_j1', 0.0)
                self.setMotorTargetJointPosition('leg5_j1', 0.0)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', -0.05)
                self.setMotorTargetJointPosition('leg4_j1', -0.05)
                self.setMotorTargetJointPosition('leg6_j1', -0.05)
                self.setMotorTargetJointPosition('leg2_j2', -0.5)
                self.setMotorTargetJointPosition('leg4_j2', -0.5)
                self.setMotorTargetJointPosition('leg6_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j2', -0.55)
                self.setMotorTargetJointPosition('leg3_j2', -0.55)
                self.setMotorTargetJointPosition('leg5_j2', -0.55)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', 0.0)
                self.setMotorTargetJointPosition('leg4_j1', 0.0)
                self.setMotorTargetJointPosition('leg6_j1', 0.0)
                self.walkForward()

            #if the hexapod is too far on the left side make a small turn to the left and move forward
            #includes a 1cm margin of error for the left sensor
            #increase to move faster; decrease to be more precise
            elif self.getSensorValue('left') > 0.33:
                self.setMotorTargetJointPosition('leg1_j2', -0.6)
                self.setMotorTargetJointPosition('leg3_j2', -0.6)
                self.setMotorTargetJointPosition('leg5_j2', -0.6)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j1', 0.05)
                self.setMotorTargetJointPosition('leg3_j1', 0.05)
                self.setMotorTargetJointPosition('leg5_j1', 0.05)
                self.setMotorTargetJointPosition('leg1_j2', -0.5)
                self.setMotorTargetJointPosition('leg3_j2', -0.5)
                self.setMotorTargetJointPosition('leg5_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j2', -0.6)
                self.setMotorTargetJointPosition('leg4_j2', -0.6)
                self.setMotorTargetJointPosition('leg6_j2', -0.6)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j1', 0.0)
                self.setMotorTargetJointPosition('leg3_j1', 0.0)
                self.setMotorTargetJointPosition('leg5_j1', 0.0)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', 0.05)
                self.setMotorTargetJointPosition('leg4_j1', 0.05)
                self.setMotorTargetJointPosition('leg6_j1', 0.05)
                self.setMotorTargetJointPosition('leg2_j2', -0.5)
                self.setMotorTargetJointPosition('leg4_j2', -0.5)
                self.setMotorTargetJointPosition('leg6_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j2', -0.6)
                self.setMotorTargetJointPosition('leg3_j2', -0.6)
                self.setMotorTargetJointPosition('leg5_j2', -0.6)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', 0.0)
                self.setMotorTargetJointPosition('leg4_j1', 0.0)
                self.setMotorTargetJointPosition('leg6_j1', 0.0)
                time.sleep(.15)
                self.walkForward()

            #if the hexapod is too close on the right side make a small turn to the left and move forward
            #includes a 1cm margin of error for the right sensor
            #increase to move faster; decrease to be more precise
            elif -1 < self.getSensorValue('right') < 0.31:
                self.setMotorTargetJointPosition('leg1_j2', -0.6)
                self.setMotorTargetJointPosition('leg3_j2', -0.6)
                self.setMotorTargetJointPosition('leg5_j2', -0.6)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j1', 0.05)
                self.setMotorTargetJointPosition('leg3_j1', 0.05)
                self.setMotorTargetJointPosition('leg5_j1', 0.05)
                self.setMotorTargetJointPosition('leg1_j2', -0.5)
                self.setMotorTargetJointPosition('leg3_j2', -0.5)
                self.setMotorTargetJointPosition('leg5_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j2', -0.6)
                self.setMotorTargetJointPosition('leg4_j2', -0.6)
                self.setMotorTargetJointPosition('leg6_j2', -0.6)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j1', 0.0)
                self.setMotorTargetJointPosition('leg3_j1', 0.0)
                self.setMotorTargetJointPosition('leg5_j1', 0.0)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', 0.05)
                self.setMotorTargetJointPosition('leg4_j1', 0.05)
                self.setMotorTargetJointPosition('leg6_j1', 0.05)
                self.setMotorTargetJointPosition('leg2_j2', -0.5)
                self.setMotorTargetJointPosition('leg4_j2', -0.5)
                self.setMotorTargetJointPosition('leg6_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j2', -0.6)
                self.setMotorTargetJointPosition('leg3_j2', -0.6)
                self.setMotorTargetJointPosition('leg5_j2', -0.6)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', 0.0)
                self.setMotorTargetJointPosition('leg4_j1', 0.0)
                self.setMotorTargetJointPosition('leg6_j1', 0.0)
                time.sleep(.15)
                self.walkForward()

            #if the hexapod is too far on the right side make a small turn to the right and move forward
            #includes a 1cm margin of error for the right sensor
            #increase to move faster; decrease to be more precise
            elif self.getSensorValue('right') > 0.33:
                self.setMotorTargetJointPosition('leg1_j2', -0.55)
                self.setMotorTargetJointPosition('leg3_j2', -0.55)
                self.setMotorTargetJointPosition('leg5_j2', -0.55)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j1', -0.05)
                self.setMotorTargetJointPosition('leg3_j1', -0.05)
                self.setMotorTargetJointPosition('leg5_j1', -0.05)
                self.setMotorTargetJointPosition('leg1_j2', -0.5)
                self.setMotorTargetJointPosition('leg3_j2', -0.5)
                self.setMotorTargetJointPosition('leg5_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j2', -0.55)
                self.setMotorTargetJointPosition('leg4_j2', -0.55)
                self.setMotorTargetJointPosition('leg6_j2', -0.55)
                self.setMotorTargetJointPosition('leg1_j1', 0.0)
                self.setMotorTargetJointPosition('leg3_j1', 0.0)
                self.setMotorTargetJointPosition('leg5_j1', 0.0)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', -0.05)
                self.setMotorTargetJointPosition('leg4_j1', -0.05)
                self.setMotorTargetJointPosition('leg6_j1', -0.05)
                self.setMotorTargetJointPosition('leg2_j2', -0.5)
                self.setMotorTargetJointPosition('leg4_j2', -0.5)
                self.setMotorTargetJointPosition('leg6_j2', -0.5)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg1_j2', -0.55)
                self.setMotorTargetJointPosition('leg3_j2', -0.55)
                self.setMotorTargetJointPosition('leg5_j2', -0.55)
                time.sleep(.15)
                self.setMotorTargetJointPosition('leg2_j1', 0.0)
                self.setMotorTargetJointPosition('leg4_j1', 0.0)
                self.setMotorTargetJointPosition('leg6_j1', 0.0)
                self.walkForward()
            
            #if all is well move forward
            else:
                self.walkForward()
            

    #forward gate
    def walkForward(self):
        self.setMotorTargetJointPosition('leg1_j3', 1.25)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.55)
        self.setMotorTargetJointPosition('leg6_j1', -0.55)
        self.setMotorTargetJointPosition('leg4_j2', -0.25)
        self.setMotorTargetJointPosition('leg4_j3', 1.25)
        time.sleep(.3)
        self.setMotorTargetJointPosition('leg3_j1', -0.55)
        self.setMotorTargetJointPosition('leg5_j1', 0.55)
        self.setMotorTargetJointPosition('leg1_j2', -0.25)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(.3)
        self.setMotorTargetJointPosition('leg2_j2', -0.6)
        self.setMotorTargetJointPosition('leg6_j2', -0.6)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j3', 2.09)
        time.sleep(.3)
        self.setMotorTargetJointPosition('leg1_j3', 2.09)
        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        time.sleep(.3)

    def holdNeutral(self):
        # --- simple example of a behavior ---- #
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg1_j3', 2.09)

        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg2_j3', 2.09)

        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j3', 2.09)

        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j3', 2.09)

        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        self.setMotorTargetJointPosition('hexa_leg5_j3', 2.09)

        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j3', 2.09)
        
    #22.5 degree turn to the left
    def turnLeft22(self):
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', 0.368)
        self.setMotorTargetJointPosition('leg3_j1', 0.368)
        self.setMotorTargetJointPosition('leg5_j1', 0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.6)
        self.setMotorTargetJointPosition('leg4_j2', -0.6)
        self.setMotorTargetJointPosition('leg6_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.368)
        self.setMotorTargetJointPosition('leg4_j1', 0.368)
        self.setMotorTargetJointPosition('leg6_j1', 0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        time.sleep(.25)

    #90 degree left turn
    def turnLeft90(self):
        self.turnLeft22()
        self.turnLeft22()
        self.turnLeft22()
        self.turnLeft22()

    #22.5 degree turn to the right
    def turnRight22(self):
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', -0.368)
        self.setMotorTargetJointPosition('leg3_j1', -0.368)
        self.setMotorTargetJointPosition('leg5_j1', -0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.5)
        self.setMotorTargetJointPosition('leg3_j2', -0.5)
        self.setMotorTargetJointPosition('leg5_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.6)
        self.setMotorTargetJointPosition('leg4_j2', -0.6)
        self.setMotorTargetJointPosition('leg6_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j1', 0.0)
        self.setMotorTargetJointPosition('leg3_j1', 0.0)
        self.setMotorTargetJointPosition('leg5_j1', 0.0)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', -0.368)
        self.setMotorTargetJointPosition('leg4_j1', -0.368)
        self.setMotorTargetJointPosition('leg6_j1', -0.368)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j2', -0.5)
        self.setMotorTargetJointPosition('leg4_j2', -0.5)
        self.setMotorTargetJointPosition('leg6_j2', -0.5)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg1_j2', -0.6)
        self.setMotorTargetJointPosition('leg3_j2', -0.6)
        self.setMotorTargetJointPosition('leg5_j2', -0.6)
        time.sleep(.25)
        self.setMotorTargetJointPosition('leg2_j1', 0.0)
        self.setMotorTargetJointPosition('leg4_j1', 0.0)
        self.setMotorTargetJointPosition('leg6_j1', 0.0)
        time.sleep(.25)

    #90 degree turn to the right
    def turnRight90(self):
        self.turnRight22()
        self.turnRight22()
        self.turnRight22()
        self.turnRight22()

    #180 degree turn to the left
    def turnAround180(self):
        self.turnLeft90()
        self.turnLeft90()



if __name__ == "__main__":
    q = HexapodControl()
    rospy.spin()