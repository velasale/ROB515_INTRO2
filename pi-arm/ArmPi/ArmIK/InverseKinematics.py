#!/usr/bin/env python3
# encoding: utf-8
# 4-DOF manipulator inverse kinematics: Given the corresponding coordinates (X, Y, Z) and pitch angle, calculate the angle of rotation of each joint
# 2020/07/20 Aiden
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # Servos count from bottom to top
    # Common parameters, that is, the connecting rod parameters of the 4-DOF manipulator
    l1 = 6.10    #The distance from the center of the robotic arm chassis to the central axis of the second servo is 6.10cm
    l2 = 10.16   #The distance from the second servo to the third servo is 10.16cm
    l3 = 9.64    #The distance from the third servo to the fourth servo is 9.64cm
    l4 = 0.00    #No specific assignment is made here, and reassignment is performed according to the selection during initialization

    # Specific parameters of the air pump
    l5 = 4.70  #The distance from the fourth servo to the top of the suction nozzle is 4.70cm
    l6 = 4.46  #The distance from the top of the nozzle to the nozzle is 4.46cm
    alpha = degrees(atan(l6 / l5)) # Calculate the angle between l5 and l4

    def __init__(self, arm_type):
        """ Adapt parameters according to different types of grippers"""
        self.arm_type = arm_type
        if self.arm_type == 'pump': #If it is an air pump robot arm
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))  #The fourth servo to the suction nozzle is used as the fourth connecting rod
        elif self.arm_type == 'arm':
            self.l4 = 16.65  #The distance from the fourth servo to the end of the robotic arm is 16.6cm, and the end of the robotic arm refers to when the claws are fully closed

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4, L5=l5, L6=l6):
        """ Change the length of the connecting rod of the robot arm, in order to adapt to the robot arm
         of different lengths with the same structure"""
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4
        self.l5 = L5
        self.l6 = L6
        if self.arm_type == 'pump':
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))
            self.alpha = degrees(atan(self.l6 / self.l5))

    def getLinkLength(self):
        # Get the currently set connecting rod length
        if self.arm_type == 'pump':
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4, "L5":self.l5, "L6":self.l6}
        else:
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        """ Given the specified coordinates and pitch angle, return the angle that each joint should rotate, if there is
        no solution, return False coordinate_data is the coordinates of the end of the gripper, the coordinate unit is
        cm, and it is passed in as a tuple, for example (0, 5, 10). Alpha is the angle between the gripper and the
        horizontal plane, in degrees"""

        """ Set the end of the gripper as P(X, Y, Z), the origin of the coordinates as O, the origin as the projection
        of the center of the gimbal on the ground, and the projection of point P on the ground as P. The intersection of
        l1 and l2 is A, the intersection of l2 and l3 is B, and the intersection of l3 and l4 is C. CD is perpendicular 
        to PD, CD is perpendicular to z axis, then the pitch angle Alpha is the angle between DC and PC, AE is 
        perpendicular to DP_, and E is on DP_, CF is perpendicular to AE, and F is on AE. 
        Angle representation: For example, the angle between AB and BC is expressed as ABC"""
        X, Y, Z = coordinate_data
        if self.arm_type == 'pump':
            Alpha -= self.alpha
        # Find the rotation angle of the base
        theta6 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) #P_to the origin O distance
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha)) #When the pitch angle is positive, PD is positive; when the pitch angle is negative, PD is negative
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        if round(CF, 4) < -self.l1:
            logger.debug('height below 0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # The sum of two sides is less than the third side
            logger.debug('cannot form a connecting rod structure, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        #seek theat4
        cos_ABC = round(-(pow(AC, 2)- pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*self.l3), 4) #cosine theorem
        if abs(cos_ABC) > 1:
            logger.debug('cannot form a connecting rod structure,abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) # Inverse trigonometric calculation of radians
        theta4 = 180.0 - degrees(ABC)

        # seek theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) #cosine theorem
        if abs(cos_BAC) > 1:
            logger.debug('cannot form a connecting rod structure, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        # seek theta3
        theta3 = Alpha - theta5 + theta4
        if self.arm_type == 'pump':
            theta3 += self.alpha

        # When there is a solution, return the angle dictionary
        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6}
            
if __name__ == '__main__':
    ik = IK('arm')
    ik.setLinkLength(L1=ik.l1 + 0.89, L4=ik.l4 - 0.3)
    print('Link length:', ik.getLinkLength())
    print(ik.getRotationAngle((0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4), 90))
