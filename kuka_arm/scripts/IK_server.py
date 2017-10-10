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
import numpy

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        roll_sym, pitch_sym, yaw_sym = symbols('roll_sym, pitch_sym, yaw_sym')
        
        # Create Modified DH parameters based on kr210.urdf.xacro
        s = { alpha0:      0, a0:      0, d1:  0.33 + 0.42, q1:          q1, # Joint 1
              alpha1:  -pi/2, a1:   0.35, d2:            0, q2: -pi/2. + q2, # Joint 2
              alpha2:      0, a2:   1.25, d3:            0, q3:          q3, # Joint 3
              alpha3:  -pi/2, a3: -0.054, d4:  0.96 + 0.54, q4:          q4, # Joint 4
              alpha4:   pi/2, a4:      0, d5:            0, q5:          q5, # Joint 5
              alpha5:  -pi/2, a5:      0, d6:            0, q6:          q6, # Joint 6
              alpha6:      0, a6:      0, d7: 0.193 + 0.11, q7:           0, # Gripper link
        }

        # Define Modified DH Transformation matrix
        #### Homogeneous Transforms
        def Generate_TF_Matrix(alpha, a, d, q):
            TF_Matrix = Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                  0,           0,             1]])
            return TF_Matrix

        T0_1 = Generate_TF_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = Generate_TF_Matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = Generate_TF_Matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = Generate_TF_Matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = Generate_TF_Matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = Generate_TF_Matrix(alpha5, a5, d6, q6).subs(s)
        T6_G = Generate_TF_Matrix(alpha6, a6, d7, q7).subs(s)
        
        #T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
        #               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
        #               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
        #               [               0,                       0,            0,               1]])
        #T0_1 = T0_1.subs(s)

        #T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
        #               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
        #               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
        #               [               0,                       0,            0,               1]])
        #T1_2 = T1_2.subs(s)

        #T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
        #               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
        #               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
        #               [                   0,                   0,            0,               1]])
        #T2_3 = T2_3.subs(s)

        #T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
        #               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
        #               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
        #               [                   0,                   0,            0,               1]])
        #T3_4 = T3_4.subs(s)

        #T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
        #               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
        #               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
        #               [                   0,                   0,            0,               1]])
        #T4_5 = T4_5.subs(s)

        #T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
        #               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
        #               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
        #               [                   0,                   0,            0,               1]])
        #T5_6 = T5_6.subs(s)

        #T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
        #               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
        #               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
        #               [                   0,                   0,            0,               1]])
        #T6_G = T6_G.subs(s)
        
        R_x  = Matrix([[                 1,                  0,                  0],
                       [                 0,      cos(roll_sym),     -sin(roll_sym)],
                       [                 0,      sin(roll_sym),      cos(roll_sym)]])

        R_y  = Matrix([[    cos(pitch_sym),              0,     sin(pitch_sym)],
                       [                 0,              1,                  0],
                       [   -sin(pitch_sym),              0,     cos(pitch_sym)]])

        R_z  = Matrix([[      cos(yaw_sym),      -sin(yaw_sym),              0],
                       [      sin(yaw_sym),       cos(yaw_sym),              0],
                       [                 0,                  0,              1]])
        
        R_corr_subs = R_z.subs({yaw_sym: pi}) * R_y.subs({pitch_sym: -pi/2.})

        # Create individual transformation matrices
        T_Total = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G     # base_link to link_G after coordination correction

        # Extract rotation matrices from the transformation matrices
        R_Total = T_Total.extract([0, 1, 2], [0, 1, 2])

        # Initialize service response
        joint_trajectory_list = []
        (theta1, theta2, theta3, theta4, theta5, theta6) = (0, 0, 0, 0, 0, 0)
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            EndEffector = Matrix([[px], [py], [pz]])
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here 
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            # Prepare rotation matrix

            Rrpy = R_z * R_y * R_x * R_corr_subs
            Rrpy_subs = Rrpy.subs({yaw_sym: yaw, pitch_sym: pitch, roll_sym: roll})
            N = Rrpy_subs[:, 2]
            WristCenter = EndEffector - 0.303 * N
            
            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WristCenter[1], WristCenter[0])

            # The distance from J2 to J3 is a2
            # The distance from J3 to WC is d4
            dist_a = 1.5
            dist_b = sqrt(pow(sqrt(WristCenter[0] * WristCenter[0] + WristCenter[1] * WristCenter[1]) - 0.35, 2) + pow(WristCenter[2] - 0.75, 2))
            dist_c = 1.25

            angle_a = acos((dist_b*dist_b + dist_c*dist_c - dist_a*dist_a) / (2 * dist_c * dist_b))
            angle_b = acos((dist_a*dist_a + dist_c*dist_c - dist_b*dist_b) / (2 * dist_a * dist_c))
            angle_c = acos((dist_a*dist_a + dist_b*dist_b - dist_c*dist_c) / (2 * dist_a * dist_b))

            theta2 = pi/2. - angle_a - atan2(WristCenter[2] - 0.75, sqrt(WristCenter[0] * WristCenter[0] + WristCenter[1] * WristCenter[1]) - 0.35)
            theta3 = pi/2. - (angle_b + 0.036)

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3_subs = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6_subs = R0_3_subs.transpose() * Rrpy_subs

            theta4 = atan2(R3_6_subs[2, 2], -R3_6_subs[0, 2])
            theta5 = atan2(sqrt(R3_6_subs[0, 2]*R3_6_subs[0, 2] + R3_6_subs[2, 2]*R3_6_subs[2, 2]), R3_6_subs[1, 2])
            theta6 = atan2(-R3_6_subs[1, 1], R3_6_subs[1, 0])

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
