#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
    # Setting up Symbols and functions 
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

        r1, r2, r3 = symbols('r1:4') #Variables the roll, pitch and yaw will be put into to

        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')#The thetas
        # Modified DH params
        s = {alpha0:	 0,  a0:        0, d1:	0.75,
                 alpha1: -pi/2,  a1:     0.35, d2:     0,	q2: q2-pi/2,
                 alpha2:	 0,  a2:     1.25, d3:     0,
                 alpha3: -pi/2,  a3:   -0.054, d4:	1.50,
                 alpha4: -pi/2,  a4:		0, d5:     0,
                 alpha5: -pi/2,  a5:		0, d6:     0,
                 alpha6: 	 0,  a6:		0, d7:	0.303,	q7:0}
        print "Verables set up."

        # Define Modified DH Transformation matrix
        def TF_Matrix(alpha, a, d, q): 
                TF = Matrix([[             cos(q),            -sin(q),            0,    a],
	                           [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
	                           [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
	                           [                   0,                   0,            0,               1]])
                return TF 

        # Create individual transformation matrices
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(s)

        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE   


        # Extract rotation matrices from the transformation matrices
        #Find EE rotation 
        #RPY rotation matrices 

        r, p, y = symbols('r p y')
        ROT_x = Matrix([[1,		0,		0],
			            [0,cos(r),-sin(r)],
			            [0,sin(r),cos(r)]]) #ROLL

        ROT_y = Matrix([[cos(p),  0, sin(p)],
			            [0,		  1,      0],
			            [-sin(p), 0, cos(p)]]) #Pitch

        ROT_z = Matrix([[	cos(y),-sin(y),		0],
	                    [	sin(y),	cos(y),		0],
	                    [	0,			 0,		1]]) #Yaw

        ROT_EE = ROT_z * ROT_y  * ROT_x

       #Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            #Rotation Error correction 

            Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
            ROT_EE = ROT_EE * Rot_Error 
            ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y': yaw})

            EE = Matrix([[px],[py],[pz]])

            WC = EE - (.303) * ROT_EE[:,2]
            #Test 3: 
            #Calculate IK 
            theta1 = atan2(py,px)
            print("theta1 = ",theta1)

            j2z = 0.75 
            j2x = 0.35 * cos(theta1)
            j2y = 0.35 * sin(theta1)
            
            vector_z_length = sqrt( np.square(px - j2x) + np.square(py - j2y)+np.square(pz - j2z))
            
            
            vector_x_length = sqrt( np.square(2.53 - 0.35)+np.square(1.946 - 2.0))

            angle_alpha = asin((j2z - pz) / vector_z_length)
            
            angle_beta = acos( (np.square(1.25) + np.square(vector_z_length) - np.square(vector_x_length)) / (2 * 1.25 * vector_z_length))
            
            theta2 = np.pi/2 - angle_beta + angle_alpha
            print("theta2 = ",theta2)
            j3z = j2z +(1.25 * cos(theta2))

            j3z_jgz = j3z - 0.054 - pz

            theta3 = asin(j3z_jgz / vector_x_length) - theta2
            print("theta3 = ", theta3)


#Test 2: 
#            #Calculate IK 
#            theta1 = atan2(py,px)

#            #Calculate theta2 and theta3 while theta 4,5,6 = 0
#            side_a = 1.25
#            side_b = 1.80452
#            side_l = sqrt((px - .35)*(px - .35) + (pz - .75)*(pz - .75))
#            
#            #Once the triangle is built we cans uses the law of cosins to find the angles 
#            #SSS triangle 
#            angle_b = acos((side_a * side_a + side_l * side_l - side_b * side_b) / (2 * side_a * side_l))
#            angle_l = acos((side_a * side_a + side_b * side_b - side_l * side_l) / (2 * side_a * side_b))
#            
#            theta2 = pi/2 - angle_b - atan2(pz - 0.75, sqrt(px * px + py * py) - 0.35)
#            theta3 = pi - angle_l 

            theta4 = 0 
            theta5 = 0 
            theta6 = 0 


#            #Calculate joint angles using Geometrix IK method
#            theta1 = atan2(WC[1],WC[0])

#            # SSS triangle for theta2 and theta3 
#            side_a = 1.501 
#            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1]*WC[1]) - 0.35),2) + pow((WC[2]-0.75),2))
#            side_c = 1.250

#            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
#            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
#            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

#            theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
#            theta3 = pi/2 - (angle_b +0.036)

#            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
#            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

#            R3_6 = R0_3.inv("LU") * ROT_EE

#            # Euler angles from rotation matrix 
#            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
#            theta5 = np.clip(atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2]),-2,2)
#            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
#                    ###

                # Populate response for the IK request
                # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)

    

    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
