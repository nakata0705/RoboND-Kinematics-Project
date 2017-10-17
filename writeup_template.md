## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/R0061823.JPG
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

DH Table

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
---   | ---   | ---    | ---   | ---
0->1  | 0     | 0      | 0.75  | q1
1->2  | -pi/2 | 0.35   | 0     | -pi/2 + q2
2->3  | 0     | 1.25   | 0     | q3
3->4  | -pi/2 | -0.054 | 1.5   | q4
4->5  | pi/2  | 0      | 0     | q5
5->6  | -pi/2 | 0      | 0     | q6
6->EE | 0     | 0      | 0.303 | 0

alpha(i-1) is the angle from Z(i-1) to z(i). As visualized in the picture below...
Z2 is 90 degree clockwise rotated Z1. So alpha1 is -pi/2.
Z4 is 90 degree clockwise rotated Z3. So alpha3 is -pi/2.
Z5 is 90 degree counter clockwise rotated Z4. So alpha4 is pi/2.
Z6 is 90 degree counter clockwise rotated Z5. So alpha5 is -pi/2.

d1 is 0.42 + 0.33. 0.33. It is the distance between ground and Joint 2. Since neither d2 nor a1 can describe the distance from Joint 1 to Joint 2 along with Z1, both 0.42 and 0.33 must be described as d1, which is the distance between the ground and Joint1.
d4 is 0.96 + 0.54. It is the distance from Joint 3 to Joint 5. Since neither d5 nor a4 can describe the distance from Joint 4 to Joint 5 along with Z4, both 0.96 and 0.54 must be described as d4, which is distance from Joint 3 to Joint 4.
d7 is 0.193 + 0.11. It is the distance from Joing 5 to the end effector. Since Z6 and Z7 are always in the same direction, we can safely add d6 to d7 and calculate the end effector position.
a1 is 0.35, Which is the distance from Joint 1 to Joint 2 along with X1.
a2 is 1.25, which is the distance from Joint 2 to Joint 3 along with X2.
a3 is -0.054 which is the distance rom Joint 3 to Joint 4 along with X3.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The individual transformation matrices are calculated as T0_1, T1_2, T2_3, T3_4, T4_5, T5_6 and T6_G in IK_server.py. The calculation is straight from the DH parameter.
The generalized homogeneous transform between base_link and gripper_link is calculated as T_Total in IK_server.py. The calculation is just multiplying all individual transformation matrices in the order of T0_1 to T6_G.
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


