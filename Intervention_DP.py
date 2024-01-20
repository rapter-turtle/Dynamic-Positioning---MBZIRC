
import sys
import os
import math
import rospy
from std_msgs.msg import Float32,Float64, Int16, Int8, Int32, Float32MultiArray
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3, PointStamped
from sensor_msgs.msg import Image, CompressedImage, Imu, NavSatFix
from core_msgs.msg import string_w_header
import tf
import numpy as np


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z



def normal_distance(relx, rely, relpsi):

    
    k = math.tan(0.5*math.pi - relpsi)

    y = (rely + k*relx)/(k*k+1)

    x = k*y

    e_x1 = (x)*math.cos(relpsi) + (y)*math.sin(relpsi)

    e_y1 = -(x)*math.sin(relpsi) + (y)*math.cos(relpsi)

    e_x0 = (relx)*math.cos(relpsi) + (rely)*math.sin(relpsi)

    e_y0 = -(relx)*math.sin(relpsi) + (rely)*math.cos(relpsi)

    e_y = e_y0

    e_x = e_x1


    return e_x


def bound_length(relx, rely, relpsi, boundary_angle,ds):

   
    k = math.tan(0.5*math.pi - relpsi)

    y = (rely + k*relx)/(k*k+1)

    x = k*y

    e_x1 = (x)*math.cos(relpsi) + (y)*math.sin(relpsi)

    e_y1 = -(x)*math.sin(relpsi) + (y)*math.cos(relpsi)

    e_x0 = (relx)*math.cos(relpsi) + (rely)*math.sin(relpsi)

    e_y0 = -(relx)*math.sin(relpsi) + (rely)*math.cos(relpsi)

    e_y = e_y0

    e_x = e_x1                               

    # distance = math.sqrt((x_0 - x)**2 + (y_0 - y)**2)
    bound_distance = abs(math.tan(boundary_angle) * (e_y + ds))

    return e_y, bound_distance

def angular_transform(Target_x, Target_y,Target_yaw, USV_x, USV_y ,USV_yaw):
    rel_psi = Target_yaw - USV_yaw
    # print(rel_psi)
    while rel_psi > 3.141592:
        rel_psi = rel_psi - 2*3.141592
    while rel_psi < -3.141592:
        rel_psi = rel_psi + 2*3.141592
    # print(rel_psi*180/math.pi)

    rel_x = (Target_x - USV_x)*math.cos(USV_yaw) + (Target_y - USV_y)*math.sin(USV_yaw)

    rel_y = (Target_x - USV_x)*math.sin(-USV_yaw) + (Target_y - USV_y)*math.cos(-USV_yaw)


    return rel_x, rel_y, rel_psi




class Control_Param():
    def __init__(self):

        self.switch = 1

        self.Target_vessel_pose_data = 0.0
        self.Target_vessel_pose_x = 0.0
        self.Target_vessel_pose_y = 0.0
        self.Target_vessel_pose_yaw = 180*math.pi/180
        
        self.USV_pose_data = 0.0
        self.USV_pose_x = 0.0
        self.USV_pose_y = 0.0
        self.USV_pose_yaw = 0.0

        self.U = 0.0
        self.U_const = 110.0
        self.U_delta = 0.0
        self.U_N = 0.0
        self.T_con = 0.0
        self.T_con_max = 600
        self.thrust_right = 0.0
        self.thrust_left = 0.0
        self.joint_right = 0.0
        self.joint_left = 0.0


        self.dtime = 0.0
        self.del_time = 0.0
        self.before_time = 0.0
        self.con_time = 0.0
        self.dock_con_time = 0.0
        self.publish_time = 0.0
        self.publish_before_time= 0.0
        self.publish_del_time = 0.0
        self.publish_sec = 0.0
        self.publish_dtime = 0.0

        self.Y_Kp = 30.0
        self.Y_Kd = 0.0
        self.N_Kp = 1000.0
        self.N_Kd = 0.0 
        self.X_Kp = 50.0
        self.X_Kd = 1.0     
        self.X_Kp_positive = 50.0
        self.X_Kp_negative = 50.0        
        self.U_default = 150
        self.K_beta = 0.001
        self.lamda = 10

        self.ds = 7
        self.ddp = 50
        self.rel_x = 0.01
        self.rel_y = 0.01
        self.rel_yaw = 0.01

        self.yaw_e = 0.0
        self.yaw_e_before = 0.0

        self.e_x = 0.0
        self.e_y = 0.0
        self.e_x_before = 0.0
        self.e_y_before = 0.0
        self.threshold_length = 0.0

        self.delta = 0.0
        self.delta_sp = 0.1
        self.delta_default = math.pi*10/180
        self.delta_default_n = math.pi*10/180

        self.desired_yaw = 0

        self.dock_switch = 2
        self.before_dock_switch = 0

        self.dock_distance = 20.0
        self.avoid_distance = 35.0

        self.wpt_distance = 30.0 #############

        self.angle_e = 0.0

        self.wpt_Kp = 100
        self.wpt_Kd = 100

        self.delta_min = math.pi*10/180

        self.real_thrust_right = 0.0
        self.real_thrust_left = 0.0
        self.before_thrust_right = 0.0
        self.before_thrust_left = 0.0
        self.thrust_bound = 50.0


        self.rel_vx = 0.0
        self.rel_vy = 0.0
        self.rel_x_before = 0.0
        self.rel_y_before = 0.0

        self.heading = 0.0
        self.heading_before = 0.0

        self.K_rel = 50.0
        self.search_time = 0.0

        self.found_heading = 0

        self.filtering_time = 0.0
        self.filtering_thres = 4000 
        self.filtering_swithch = 0      

        self.global_vel_x = 0.0
        self.global_vel_y = 0.0

        self.real_rel_x = 0.0
        self.real_rel_y = 0.0

        self.rotate_thrust = 400

        self.target_dtime = 0.0
        self.heading_search_time = 0.0

        self.before_rel_yaw = 0.0

        self.lidar_heading = 0.0


        self.wpt_bound = 20
        self.wpt_x_avoidance = 0.0
        self.wpt_y_avoidance = 0.0

        self.ID = 0
        self.delta_max = 70*math.pi/180
        self.steer = 4.5*math.pi/180

        self.y_bias = 0

        self.button = []
        self.before_button = []
        self.before_button_sub = []
        self.pub_button = 0
        self.converge_button = 1    

        self.target_vessel_length = 0    

        self.converge_t = 3
        self.bound = 30.0

        self.raw_ey = 0.0
        self.raw_ey_before = 0.0
        self.s = 0.0

        self.Y_Ki = 0.01
        self.T_con_I = 0.0
        self.max_T_con_I = 50

        self.T_con_s2 = 0.0
        self.start = 0

        self.raw_queue = []
        self.sec = 0.0

        self.avoidance = 0.0
        self.raw_queue = []
        self.max_T_con_I = 100.0
        self.ey_pub = rospy.Publisher('/ey', Float64, queue_size=1)
        self.raw_ey_pub = rospy.Publisher('/raw_ey', Float64, queue_size=1)
        self.rel_psi_pub = rospy.Publisher('/usv_heading', Float64, queue_size=1)
        self.T_con_pub = rospy.Publisher('/tcon', Float64, queue_size=1)
        self.U_N_pub = rospy.Publisher('/un', Float64, queue_size=1)       
        self.l_heading_pub = rospy.Publisher('/lidar_heading', Float64, queue_size=1)
        self.usv_heading_pub = rospy.Publisher('/rel_yaw', Float64, queue_size=1)
        self.error_heading_pub = rospy.Publisher('/error_heading', Float64, queue_size=1)    
        self.desired_heading_pub = rospy.Publisher('/desired_heading', Float64, queue_size=1)    


    def Imu_pose_sub(self,data):
        quat = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w])
        self.USV_pose_yaw= quat[2] #+ 3.141592*0.2

        if self.USV_pose_yaw < -math.pi:
            self.USV_pose_yaw += 2 * 3.141592
        if self.USV_pose_yaw > math.pi:
            self.USV_pose_yaw -= 2 * 3.141592

      

    def sub_local_xy(self,msg):
        if self.dock_switch == 8:
            if len(msg.data) > 3:

                N = int(msg.data[0])
                check = N
                # print(msg.data)

                for i in range(N):
                    if self.ID == int(msg.data[1+9*i]):
                        self.real_rel_x = msg.data[1+9*i+1]
                        self.real_rel_y = msg.data[1+9*i+2]
                        # global_lidar_heading = -msg.data[1+9*i+5]  ############# definitly change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        global_lidar_heading = msg.data[1+9*i+5]

                        self.target_vessel_length = msg.data[1+9*i+6]

                        self.lidar_heading = global_lidar_heading - self.USV_pose_yaw

                        while self.lidar_heading < -math.pi:
                            self.lidar_heading += 2 * 3.141592
                        while self.lidar_heading > math.pi:
                            self.lidar_heading -= 2 * 3.141592
                    else:
                        check = check - 1

                if check == 0:
                    self.filtering_swithch = 1
                    
                    if self.filtering_time < self.filtering_thres:
                        self.filtering_time += self.dtime 
                        print("########## No TRG 1 ################")
                        print(self.filtering_time)
                        print(self.dtime)
                        print("##################################")
                    else:
                        self.filtering_swithch = 0
                        self.filtering_time = 0.0                        
                else:
                    self.filtering_swithch = 0
                    self.filtering_time = 0.0 
            
            else:
                if self.filtering_time < self.filtering_thres:
                    self.filtering_swithch = 1
                    self.filtering_time += self.dtime
                    print("########## No TRG 2 ################")
                    print(self.filtering_time)
                    print(self.dtime)
                    print("##################################")                    
                else: 
                    self.filtering_swithch = 0
                    self.filtering_time = 0.0    
        else:
            self.filtering_time = 0.0
            self.filtering_swithch = 0


    def sub_wp_ID(self, msg):
        self.ID = msg.data


    def dock(self,msg):
        self.dock_switch = msg.data
        if self.dock_switch < 4 and self.before_dock_switch >= 4:
            self.found_heading = 0

        self.before_dock_switch = self.dock_switch


    def WPT(self,data):
        self.wpt_Kp = data.data
    def sub_U_con(self,data):
        self.U_const = data.data
        # print(self.U_const)
    def sub_nkp(self,data):
        self.N_Kp = data.data
    def sub_nkd(self,data):
        self.N_Kd = data.data
    def sub_ykp(self,data):
        self.Y_Kp = data.data
    def sub_ykd(self,data):
        self.Y_Kd = data.data
    def sub_xkp(self,data):
        self.X_Kp = data.data
    def sub_xkd(self,data):
        self.X_Kd = data.data
    def sub_Tmax(self,data):
        self.T_con_max = data.data
    def sub_Target_heading(self, data):
        self.Target_vessel_pose_yaw = data.data
    def sub_K_beta(self,data):
        self.K_beta = data.data
    def sub_lamda(self,data):
        self.lamda = data.data        
    def sub_y_bias(self,data):
        self.y_bias = data.data  
    def dp_sub(self,data):
        self.before_button = self.button
        self.button = data.data
        if len(self.button) == 2:
            if self.button[0] == 1:
                self.pub_button = 0
                self.y_bias = self.button[1]
                self.converge_button = 0
                self.con_time = 0.0

            self.before_button_sub = self.button
    def sub_con_range(self, data):
        self.converge_t = data.data
    def sub_wpt_distance(self, data):
        self.wpt_distance = data.data
    def sub_bound(self, data):
        self.bound = data.data



            
    def control_time(self,msg):
        # self.nanosec = msg.header.stamp.nsecs * 0.000000001
        # self.sec = msg.header.stamp.secs


        # if self.before_time == 0.0:
        #     self.before_time = self.sec + self.nanosec

        # self.del_time = abs(self.before_time - (self.sec + self.nanosec))
        # self.dtime = self.dtime + self.del_time
        # self.before_time = self.sec + self.nanosec
        # self.sec = rospy.get_time()

        # if self.before_time == 0.0:
        #     self.before_time = self.sec
        
        # self.del_time = abs(self.before_time - self.sec)
        # self.dtime = self.dtime + self.del_time
        # self.before_time = self.sec 

        
        if self.dtime > 0.1:
            # print("a")
            if self.dock_switch == 8:
                # print(self.dtime)
                        
                if self.found_heading >= 1:
                
                    rel_yaw_raw = self.lidar_heading
                
                    # h = Float64()
                    # h.data = self.lidar_heading
                    # self.l_heading_pub.publish(self.lidar_heading)                
                    ################################################################## -- heading_on ------ heading choose
                    rel_error = rel_yaw_raw - self.before_rel_yaw

                    while rel_error > math.pi:
                        rel_error -= 2*math.pi
                    while rel_error < -math.pi:
                        rel_error += 2*math.pi

                    # print(rel_error)

                    if rel_error > 0.5*math.pi:
                        rel_yaw_raw += math.pi
                    if rel_error < -0.5*math.pi:
                        rel_yaw_raw += math.pi
                         
                    
                    while rel_yaw_raw > math.pi:
                        rel_yaw_raw -= 2*math.pi
                    while rel_yaw_raw < -math.pi:
                        rel_yaw_raw += 2*math.pi
                    ##################################################################
                    # eh = Float64()
                    # eh.data = rel_error
                    # self.error_heading_pub.publish(eh)                        


                    # rel_yaw_raw = 
                    
                    # filtering_error = rel_yaw_raw - self.before_rel_yaw
                    
                    # while filtering_error > math.pi:
                    #     filtering_error -= 2*math.pi
                    # while filtering_error< -math.pi:
                    #     filtering_error += 2*math.pi
                    a = 0.7


                    self.raw_queue.append(rel_yaw_raw)
                    if len(self.raw_queue)>20:
                        self.raw_queue = self.raw_queue[1:]

                    diff = self.raw_queue.copy()
                    for i in range(len(self.raw_queue)):
                            diff[i] -= rel_yaw_raw

                    for i in range(len(self.raw_queue)):
                        while diff[i] > math.pi:
                            diff[i] -= 2*math.pi
                        while diff[i] < -math.pi:
                            diff[i] += 2*math.pi  

                    w = []
                    for i in range(len(self.raw_queue)):
                        w.append(a**i)
                    w.reverse()
                    weighted_sum = 0
                    w_sum = (1-a**len(self.raw_queue))/(1-a)
                    for i in range(len(self.raw_queue)):
                        weighted_sum += diff[i] * w[i]
                    weighted_sum /= w_sum

                    self.rel_yaw = weighted_sum + rel_yaw_raw


                    while self.rel_yaw > math.pi:
                        self.rel_yaw -= 2*math.pi
                    while self.rel_yaw < -math.pi:
                        self.rel_yaw += 2*math.pi


                    # self.rel_yaw = a*rel_yaw_raw + (1 - a)*self.rel_yaw

                    # track = Float64()
                    # track.data = self.rel_yaw
                    # self.usv_heading_pub.publish(track)
                    self.yaw_e = self.rel_yaw

                    self.before_rel_yaw = rel_yaw_raw

                    # print(self.yaw_e)
                    # print("aaaaaaaaaaaaaaaaaaaaaaa")

        ########################################################################### N control ###########################################################################


                self.U_N = ( self.N_Kp*self.desired_yaw + self.N_Kd * (self.desired_yaw - self.yaw_e_before)*10)
                

                self.yaw_e_before = self.desired_yaw


        ######################################################################## Axis calculation #########################################################################

                # self.rel_x, self.rel_y, self.rel_yaw = angular_transform(self.rel_x, self.rel_y,self.Target_vessel_pose_yaw, 0.0, 0.0 ,self.USV_pose_yaw)
                rx =self.real_rel_x
                ry =self.real_rel_y

                self.rel_x = rx*math.cos(self.USV_pose_yaw) + ry*math.sin(self.USV_pose_yaw)

                self.rel_y = rx*math.sin(-self.USV_pose_yaw) + ry*math.cos(-self.USV_pose_yaw)
                
                self.raw_ey = normal_distance(self.rel_x, self.rel_y, self.rel_yaw)
                self.raw_ey += self.y_bias

                a = 0.3
                self.e_y = a*self.raw_ey + (1 - a)*self.e_y

                ey = Float64()
                raw_ry = Float64()

                ey.data = self.e_y
                raw_ry.data = self.raw_ey

                self.ey_pub.publish(ey)
                self.raw_ey_pub.publish(raw_ry)
                
                # self.e_y = normal_distance(self.rel_x, self.rel_y, self.rel_yaw)
                self.e_x , self.threshold_length = bound_length(self.rel_x, self.rel_y, self.rel_yaw, math.pi*self.bound/180, self.ds)
                # self.e_y = (self.e_y + self.y_bias)
                self.e_x = self.e_x + self.ds
                # print("not inside boundary")

        ######################################################################## heading search #########################################################################
            
                if self.found_heading == 0:
                    ###### ooo #####
                    self.U_N = 0.0
                    self.T_con = 0.0


                    self.heading_search_time += self.dtime
                    self.target_dtime += self.dtime 

                    if self.heading_search_time <= 0.2:
                        print("------------- Search Heading -------------")
                        if self.target_dtime > 1.0:


                                if self.lidar_heading > 0.5*math.pi:
                                    self.lidar_heading += math.pi
                                if  self.lidar_heading < -0.5*math.pi:
                                    self.lidar_heading += math.pi


                                while self.lidar_heading > math.pi:
                                    self.lidar_heading -= 2*math.pi
                                while self.lidar_heading < -math.pi:
                                    self.lidar_heading += 2*math.pi

                                

                                print("#####################################################################################")
                                print("Lidar : ", self.lidar_heading*180/math.pi)
                                # print("Error : ", new_heading_error*180/math.pi)
                                print(self.heading_search_time)
                                print("#####################################################################################")

                                self.rel_yaw = self.lidar_heading
                                self.yaw_e = self.rel_yaw
                                self.before_rel_yaw = self.rel_yaw

                                self.target_dtime = 0.0
                    else:
                        self.found_heading = 1
                        self.heading_search_time = 0.0
                        self.target_dtime = 0.0
                    ###### ooo #####

        #################################################################### Avoidance control ###########################################################################

                elif self.found_heading == 1:

                    if self.rel_yaw < 0.0 and self.rel_yaw > -160*math.pi/180:

                        self.wpt_x_avoidance = (self.rel_x + (self.avoid_distance-1)*math.cos(self.rel_yaw - math.pi))
                        self.wpt_y_avoidance = (self.rel_y + (self.avoid_distance)*math.sin(self.rel_yaw - math.pi))

                        self.angle_e = math.atan2(self.wpt_y_avoidance, self.wpt_x_avoidance)
                        
                        while self.angle_e < -math.pi or self.angle_e > math.pi:
                            if self.angle_e > math.pi:
                                self.angle_e = self.angle_e - 2*math.pi
                            elif self.angle_e < -math.pi:
                                self.angle_e = self.angle_e + 2*math.pi

                        U_avoid = self.wpt_Kp*self.angle_e
                        T_avoid = self.U_const

                        if T_avoid < 0.0:
                            self.T_con = -math.sqrt(((-T_avoid)**1.351)/0.0013)
                        else:
                            self.T_con = math.sqrt(((T_avoid)**1.351)/0.0013)

                        if U_avoid < 0.0:
                            self.U_N = -math.sqrt(((-U_avoid)**1.351)/0.0013)
                        else:
                            self.U_N = math.sqrt(((U_avoid)**1.351)/0.0013)

                        self.joint_right = self.steer
                        self.joint_left = self.steer

                        print("################## Avoidance ##################")
                        print('Ship ID : ', self.ID)
                        print("Found_heading : ", self.found_heading)
                        print("wpt_x" , self.wpt_x_avoidance)
                        print("wpt_y" , self.wpt_y_avoidance)
                        print("angle : ", self.angle_e*180/math.pi)
                        print("rel : ", self.rel_yaw)
                        print("U_N : ", self.U_N,", T_con : ", self.T_con)
                        # print(" thrust right : ", self.real_thrust_right, ", thrust left : ", self.real_thrust_left)

                        if math.sqrt(self.wpt_x_avoidance*self.wpt_x_avoidance + self.wpt_y_avoidance*self.wpt_y_avoidance) < self.wpt_bound:
                            self.found_heading = 2
                    else:
                        self.found_heading = 2


        ########################################################################## wpt control ##########################################################################
                elif math.sqrt(self.e_y*self.e_y + self.e_x*self.e_x) > self.wpt_distance and self.found_heading == 2:

                    wpt_x = (self.rel_x + (self.dock_distance-1)*math.cos(self.rel_yaw - math.pi*1.5))
                    wpt_y = (self.rel_y + (self.dock_distance)*math.sin(self.rel_yaw - math.pi*1.5))

                    self.angle_e = math.atan2(wpt_y, wpt_x)
                    
                    while self.angle_e < -math.pi or self.angle_e > math.pi:
                        if self.angle_e > math.pi:
                            self.angle_e = self.angle_e - 2*math.pi
                        elif self.angle_e < -math.pi:
                            self.angle_e = self.angle_e + 2*math.pi

                    U_wpt = self.wpt_Kp*self.angle_e
                    T_wpt = self.U_const

                    if T_wpt < 0.0:
                        self.T_con = -math.sqrt(((-T_wpt)**1.351)/0.0013)
                    else:
                        self.T_con = math.sqrt(((T_wpt)**1.351)/0.0013)

                    if U_wpt < 0.0:
                        self.U_N = -math.sqrt(((-U_wpt)**1.351)/0.0013)
                    else:
                        self.U_N = math.sqrt(((U_wpt)**1.351)/0.0013)

                    self.joint_right = self.steer
                    self.joint_left = self.steer

                    print("################## WPT ##################")
                    print('Ship ID : ', self.ID)
                    print("Found_heading : ", self.found_heading)
                    print("wpt_x" , wpt_x)
                    print("wpt_y" , wpt_y)
                    print("angle : ", self.angle_e*180/math.pi)
                    print("rel yaw : ", self.rel_yaw*180/math.pi)
                    print("U_N : ", self.U_N,", T_con : ", self.T_con)
                    # print(" thrust right : ", self.real_thrust_right, ", thrust left : ", self.real_thrust_left)

            
        #################################################################################################################################################################


        ########################################################################## 1st control ##########################################################################
                elif math.sqrt(self.e_y*self.e_y + self.e_x*self.e_x) < self.wpt_distance and self.found_heading ==2:
                    if self.switch == 1:
                        # if self.e_x > 0:


                        if self.e_y < -abs(self.threshold_length) or self.e_y > abs(self.threshold_length):
                            self.switch = 2
                    
                        if self.e_x < 0:
                            change_e = (self.delta_sp*abs(self.e_x) + self.delta_default_n)
                            if change_e > self.delta_max:
                                change_e = self.delta_max
                            elif change_e < -self.delta_max:
                                change_e = -self.delta_max
                            if self.T_con_s2 > 0:    
                                self.desired_yaw = self.yaw_e - change_e
                            else:
                                self.desired_yaw = self.yaw_e + change_e
                            
                        if self.e_x >= 0:
                            change_e = (self.delta_sp*abs(self.e_x) + self.delta_default_n)
                            if change_e > self.delta_max:
                                change_e = self.delta_max
                            elif change_e < -self.delta_max:
                                change_e = -self.delta_max                            
                            if self.T_con_s2 > 0:    
                                self.desired_yaw = self.yaw_e + change_e
                            else:
                                self.desired_yaw = self.yaw_e - change_e

                        
                        if self.e_x < 2 and self.e_x > -2:
                            if self.desired_yaw - self.yaw_e > self.delta_min:
                                self.desired_yaw = self.delta_min + self.yaw_e

                            elif self.desired_yaw - self.yaw_e < -self.delta_min:
                                self.desired_yaw = -self.delta_min + self.yaw_e


                        if self.T_con_s2 > 0:
                            self.T_con = abs(self.X_Kp*self.e_x + self.X_Kd*(self.e_x - self.e_x_before)*10 + self.U_default)
                        else:
                            self.T_con = -abs(self.X_Kp*self.e_x + self.X_Kd*(self.e_x - self.e_x_before)*10 + self.U_default)


                        if self.T_con > self.T_con_max:
                            self.T_con = self.T_con_max
                        elif self.T_con < -self.T_con_max:
                            self.T_con = -self.T_con_max


                        self.e_x_before = self.e_x

                        self.joint_right = self.steer
                        self.joint_left = self.steer


        #################################################################################################################################################################


        ########################################################################## 2nd control ##########################################################################
                    elif self.switch == 2:

                        self.desired_yaw = self.yaw_e
                        
                        if self.e_y > -3.0 and self.e_y < 3.0:
                            self.con_time = self.con_time + self.dtime
                        else:
                            self.con_time = 0.0

                        if self.con_time > 1:
                            # self.switch = 1
                            # self.con_time = 0.0
                            if self.e_x > 2.5 or self.e_x < -2.5:
                                self.switch = 1
                                self.con_time = 0.0
                            if self.con_time > self.converge_t and self.converge_button == 0:
                                pub_dp = rospy.Publisher('/dp_ref',Float32MultiArray, queue_size=1)
                                dp = Float32MultiArray()
                                dp.data = [1]
                                pub_dp.publish(dp)
                                self.converge_button = 1
                            


                        if abs(self.desired_yaw) > math.pi*180/180:
                            self.T_con = 0
                        else:    
                            self.T_con_I += self.e_y*0.1
                            if self.T_con_I > self.max_T_con_I/self.Y_Ki:
                                self.T_con_I = self.max_T_con_I/self.Y_Ki
                            if self.T_con_I < -self.max_T_con_I/self.Y_Ki:
                                self.T_con_I = -self.max_T_con_I/self.Y_Ki      
                            self.T_con = (self.Y_Kp*self.e_y + self.Y_Kd*(self.e_y - self.e_y_before)*10) #+ self.Y_Ki*self.T_con_I
                            self.delta = 0.0

                            if self.T_con > self.T_con_max:
                                self.T_con = self.T_con_max
                            elif self.T_con < -self.T_con_max:
                                self.T_con = -self.T_con_max

                        self.T_con_s2 = self.T_con            


                        self.e_y_before = self.e_y


                        self.joint_right = self.steer
                        self.joint_left = self.steer

                        
        #################################################################################################################################################################

                    print("#########################################################################################")                    
                    print("Switch : ", self.switch)
                    print("Boundary : ", self.threshold_length, "con_time : ", self.con_time)
                    print("e_x : ", self.e_x, ", e_y : ", self.e_y)
                    print("rel_x : ", self.rel_x, ", rel_y : ", self.rel_y, "rel_psi : ",self.rel_yaw*180/math.pi)
                    print("U_N : ", self.U_N,", T_con : ", self.T_con)
                    print("filter time : ", self.filtering_time, ", filtering switch : ", self.filtering_swithch)
                    print("Cont time : ", self.con_time)
                    print("USV H : ", self.USV_pose_yaw*180/math.pi)
                    print("Lidar heading : ", self.lidar_heading*180/math.pi)
                    print("desired_yaw : ", self.desired_yaw*180/math.pi)
                    print("Converge button : ",self.converge_button)
                    


                self.thrust_right = self.T_con + self.U_N
                self.thrust_left = self.T_con - self.U_N


                tc = Float64()
                tc.data = self.T_con
                self.T_con_pub.publish(tc)

                un = Float64()
                un.data = self.U_N
                self.U_N_pub.publish(un)




                print("U_N : ", self.U_N,", T_con : ", self.T_con)
                print(" thrust right : ", self.real_thrust_right, ", thrust left : ", self.real_thrust_left)



            self.dtime = 0.0

            





def main():
    # settings = saveTerminalSettings()
    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    param = Control_Param()

    pub_lt = rospy.Publisher('/workshop_setup/pods/left', Int16, queue_size=1)
    pub_rt = rospy.Publisher('/workshop_setup/pods/right', Int16, queue_size=1)
    pub_la = rospy.Publisher('/workshop_setup/pod_steer/left_steer', Float32, queue_size=1)
    pub_ra = rospy.Publisher('/workshop_setup/pod_steer/right_steer', Float32, queue_size=1)
    pub_dp = rospy.Publisher('/dp_ref',Float32MultiArray, queue_size=1)
    pub_target_length = rospy.Publisher('/vessel_length',Float32, queue_size=1)


    rospy.Subscriber('/imu/data', Imu, param.Imu_pose_sub)
    rospy.Subscriber('/usv_flag', Int32, param.dock)
    rospy.Subscriber('/dp_ref', Float32MultiArray, param.dp_sub)

    rospy.Subscriber('/imu/data', Imu, param.control_time)

    rospy.Subscriber('/Y_Kp' , Float64, param.sub_ykp)
    rospy.Subscriber('/Y_Kd' , Float64, param.sub_ykd)
    rospy.Subscriber('/X_Kp' , Float64, param.sub_xkp)
    rospy.Subscriber('/X_Kd' , Float64, param.sub_xkd)
    rospy.Subscriber('/N_Kp' , Float64, param.sub_nkp)
    rospy.Subscriber('/N_Kd' , Float64, param.sub_nkd)
    rospy.Subscriber('/T_max' , Float64, param.sub_Tmax)
    rospy.Subscriber('/wpt' , Float64, param.WPT)
    rospy.Subscriber('/U_con' , Float64, param.sub_U_con)
    rospy.Subscriber('/K_beta' , Float64, param.sub_K_beta)
    rospy.Subscriber('/lamda' , Float64, param.sub_lamda) 
    rospy.Subscriber('/y_bias' , Float64, param.sub_y_bias) 
    rospy.Subscriber('/con_range' , Float64, param.sub_con_range)
    rospy.Subscriber('/wpt_distance' , Float64, param.sub_wpt_distance)
    rospy.Subscriber('/bound' , Float64, param.sub_bound)


    rospy.Subscriber('/lidar_track', Float32MultiArray, param.sub_local_xy, queue_size = 1)
    rospy.Subscriber('/wp_id', Float32, param.sub_wp_ID)
    rospy.Subscriber('/target_heading', Float64, param.sub_Target_heading)




    rate = rospy.Rate(200) # 10 Hz



    while not rospy.is_shutdown():
        param.publish_sec = rospy.get_time()

        if param.publish_before_time == 0.0:
            param.publish_before_time = param.publish_sec
        
        param.publish_del_time = abs(param.publish_before_time - param.publish_sec)
        param.publish_dtime = param.publish_dtime + param.publish_del_time
        param.publish_before_time = param.publish_sec 


        param.sec = rospy.get_time()

        if param.before_time == 0.0:
            param.before_time = param.sec
        
        param.del_time = abs(param.before_time - param.sec)
        param.dtime = param.dtime + param.del_time
        param.before_time = param.sec 




        Thrust_right = Int16()
        Angle_right = Float32()
        Thrust_left = Int16()
        Angle_left = Float32()

        rotate_Thrust_right = Int16()
        rotate_Angle_right = Float32()
        rotate_Thrust_left = Int16()
        rotate_Angle_left = Float32()

        rotate_Thrust_right.data = 400
        rotate_Angle_right.data = 0.0
        rotate_Thrust_left.data = -400
        rotate_Angle_left.data = 0.0

        stop_Thrust_right = Int16()
        stop_Angle_right = Float32()
        stop_Thrust_left = Int16()
        stop_Angle_left = Float32()

        stop_Thrust_right.data = 0
        stop_Angle_right.data = 0.0
        stop_Thrust_left.data = 0
        stop_Angle_left.data = 0.0
   

        if param.publish_dtime > 0.1:


        
            if param.thrust_left - param.real_thrust_left > param.thrust_bound:
                param.real_thrust_left = param.real_thrust_left + param.thrust_bound
            elif param.thrust_left - param.real_thrust_left < -param.thrust_bound:
                param.real_thrust_left = param.real_thrust_left - param.thrust_bound
            else:
                param.real_thrust_left = param.thrust_left


            if param.thrust_right - param.real_thrust_right > param.thrust_bound:
                param.real_thrust_right = param.real_thrust_right + param.thrust_bound
            elif param.thrust_right - param.real_thrust_right < -param.thrust_bound:
                param.real_thrust_right = param.real_thrust_right - param.thrust_bound
            else:
                param.real_thrust_right = param.thrust_right

            if param.real_thrust_left > 900:
                param.real_thrust_left = 900
            if param.real_thrust_left < -900:
                param.real_thrust_left = -900
            if param.real_thrust_right > 900:
                param.real_thrust_right = 900
            if param.real_thrust_right < -900:
                param.real_thrust_right = -900


            Thrust_right.data = math.floor(param.real_thrust_right)
            Angle_right.data = param.joint_right*180/math.pi
            Thrust_left.data = math.floor(param.real_thrust_left)
            Angle_left.data = param.joint_left*180/math.pi



            dp = Float32MultiArray()
            dp.data = [0]

            length = Float32()
            length.data = param.target_vessel_length

     
            if param.dock_switch == 8:

                pub_target_length.publish(length)
                
                if len(param.before_button_sub) == 2 and param.before_button_sub[0] == 0:
                    pub_rt.publish(stop_Thrust_right)
                    pub_ra.publish(stop_Angle_right)
                    pub_lt.publish(stop_Thrust_left)
                    pub_la.publish(stop_Angle_left)  
                    if param.pub_button == 0:
                        pub_dp.publish(dp)
                        param.pub_button = 1   
                else:            
                    if param.filtering_swithch == 0:
                        pub_rt.publish(Thrust_right)
                        pub_ra.publish(Angle_right)
                        pub_lt.publish(Thrust_left)
                        pub_la.publish(Angle_left)

                    if param.filtering_swithch == 1:
                        pub_rt.publish(rotate_Thrust_right)
                        pub_ra.publish(rotate_Angle_right)
                        pub_lt.publish(rotate_Thrust_left)
                        pub_la.publish(rotate_Angle_left)             
            param.publish_dtime = 0.0   
 

        rate.sleep()




if __name__ == '__main__':
    main()







