#!/usr/bin/env python3

import os
import numpy as np
import tf
import rospy
import rospkg
import threading
import random
import ctypes
import raisimpy as raisim
import time
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from quadruped_ctrl.srv import QuadrupedCmd, QuadrupedCmdResponse


class StructPointer(ctypes.Structure):
    _fields_ = [("eff", ctypes.c_double * 12)]


class WalkingSimulation(object):
    def __init__(self):

        self.prev_body_velocity_lin = [0] * 3
        rospy.logwarn("node start")

        self.__init_ros()
        rospy.logwarn("ros init")

        self.__load_controller()
        rospy.logwarn("controller loaded")

        self.__init_simulator()
        rospy.logwarn("simulator start")

        add_thread = threading.Thread(target=self.__thread_job)
        add_thread.start()

         

    #это можно оставить, параметры не будут использоваться
    def __init_ros(self):

        #коффеценты ПИД для МПС (хз где они применяются)
        self.freq = float(500)
        self.stand_kp = float(100)
        self.stand_kd = float(1)
        self.joint_kp = float(0)
        self.joint_kd = float(0.05)

        rospy.loginfo(" freq = " + str(self.freq) + " PID = " +
                      str([self.stand_kp, self.stand_kd, self.joint_kp, self.joint_kd]))
        
        self.rate = rospy.Rate(self.freq)  # Hz

        self.s0 = rospy.Service('gait_type', QuadrupedCmd, self.__callback_gait)
        self.s1 = rospy.Service('robot_mode', QuadrupedCmd, self.__callback_mode)
        self.s2 = rospy.Subscriber("cmd_vel", Twist, self.__callback_body_vel, buff_size=30)


    #это вообще не трогать
    def __load_controller(self):
        # rospy.logwarn("start load contrller")
        self.path = rospkg.RosPack().get_path('quadruped_ctrl')
        so_file = self.path.replace('src/quadruped_ctrl', 'devel/lib/libquadruped_ctrl.so')

        if(not os.path.exists(so_file)):
            so_file = self.path.replace('src/quadruped_ctrl', 'build/lib/libquadruped_ctrl.so')

        if(not os.path.exists(so_file)):
            rospy.logerr("cannot find cpp.so file")

        self.cpp_gait_ctrller = ctypes.cdll.LoadLibrary(so_file)
        self.cpp_gait_ctrller.torque_calculator.restype = ctypes.POINTER(StructPointer)


    #инициализируем раисим
    def __init_simulator(self):
        raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/rsc/activation.raisim") #license
        self.cheetah_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../urdf/mini_cheetah.urdf" #path to urdf (((make to general)))

        #world and server init
        self.world = raisim.World()
        self.world.setTimeStep(0.001)
        self.server = raisim.RaisimServer(self.world)
        self.ground = self.world.addGround()


    #инициализация робота
    def __reset_robot(self):
        self.cheetah = self.world.addArticulatedSystem(self.cheetah_urdf_file)
        self.cheetah.setName("cheetah")

        self.cheetah_joint_position = np.array([0, 0, 0.54, 1, 0.0, 0.0, 0.0, 
                                                 0.0, -0.8, 1.6, 
                                                 0, -0.8, 1.6,
                                                 0.00, -0.8, 1.6, 
                                                 0, -0.8, 1.6,])
        self.cheetah_joint_body_velocity = np.zeros([18])

        #PD for laod time, after it turn off
        self.cheetah.setGeneralizedCoordinate(self.cheetah_joint_position)
        self.cheetah.setPdGains(70*np.ones([18]), np.ones([18]))
        self.cheetah.setPdTarget(self.cheetah_joint_position, self.cheetah_joint_body_velocity)

        self.tau = np.zeros(18)
        self.cheetah.setGeneralizedForce(self.tau)

        #server setup
        self.server.launchServer(8080)
        self.server.focusOn(self.cheetah)
        time.sleep(2)
        self.world.integrate1()

        self.cpp_gait_ctrller.init_controller(
            self.__convert_type(self.freq),
            self.__convert_type([self.stand_kp, self.stand_kd, self.joint_kp, self.joint_kd]))

        #это типо калибровки
        for _ in range(10):
            self.world.integrate1()
            imu_data, leg_data = self.__get_data_from_sim()

            self.cpp_gait_ctrller.pre_work(self.__convert_type(
                imu_data), self.__convert_type(leg_data["state"]))


        self.cpp_gait_ctrller.set_robot_mode(self.__convert_type(1))


    #основной loop
    def run(self):
        rospy.logwarn("run function start")

        self.__reset_robot()
        rospy.logwarn("robot reseted")

        self.cheetah.setControlMode(raisim.ControlMode.FORCE_AND_TORQUE)
        self.cpp_gait_ctrller.set_robot_mode(self.__convert_type(1)) # low energy mod in quadruped_ctrl , ((may be reduce))
        
        while not rospy.is_shutdown():
            self.__simulation_step()
            self.rate.sleep()

    #один цикл симуляции
    def __simulation_step(self):
        # get data from simulator
        imu_data, leg_data = self.__get_data_from_sim()

        # pub msg (don't work)
        # self.__pub_nav_msg(base_pos, imu_data)
        # self.__pub_imu_msg(imu_data)
        # self.__pub_joint_states(leg_data)
        #self.__pub_whole_body_state(imu_data, leg_data, base_pos, contact_points)

        # call cpp function to calculate tau from mpc
        tau = self.cpp_gait_ctrller.torque_calculator(self.__convert_type(
            imu_data), self.__convert_type(leg_data["state"]))

        generalized_tau = np.concatenate((np.zeros(6), tau.contents.eff))
        self.cheetah.setGeneralizedForce(generalized_tau)

        self.server.integrateWorldThreadSafe()

    
    #какой-то конвертор типов, лучше не трогать
    def __convert_type(self, input):
        ctypes_map = {
            int: ctypes.c_int,
            float: ctypes.c_double,
            str: ctypes.c_char_p,
                     }
        input_type = type(input)
        if input_type is list:
            length = len(input)
            if length == 0:
                rospy.logerr("convert type failed...input is " + input)
                return 0
            else:
                arr = (ctypes_map[type(input[0])] * length)()
                for i in range(length):
                    arr[i] = bytes(
                        input[i], encoding="utf-8") if (type(input[0]) is str) else input[i]
                return arr
        else:
            if input_type in ctypes_map:
                return ctypes_map[input_type](bytes(input, encoding="utf-8") if type(input) is str else input)
            else:
                rospy.logerr("convert type failed...input is "+input)
                return 0

    def __thread_job(self):
        rospy.spin()

    def __callback_gait(self, req):
        self.cpp_gait_ctrller.set_gait_type(self.__convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the gait")

    #мб выпилить
    def __callback_mode(self, req):
        self.cpp_gait_ctrller.set_robot_mode(self.__convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the mode")

    def __callback_body_vel(self, msg):
        vel = [msg.linear.x, msg.linear.y, msg.angular.x]
        self.cpp_gait_ctrller.set_robot_vel(self.__convert_type(vel))

    def __fill_tf_message(self, parent_frame, child_frame, translation, rotation):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        return t


    def __pub_nav_msg(self, base_pos, imu_data):
        pub_odom = rospy.Publisher("/robot_odom", Odometry, queue_size=30)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = base_pos[0]
        odom.pose.pose.position.y = base_pos[1]
        odom.pose.pose.position.z = base_pos[2]
        odom.pose.pose.orientation.x = imu_data[3]
        odom.pose.pose.orientation.y = imu_data[4]
        odom.pose.pose.orientation.z = imu_data[5]
        odom.pose.pose.orientation.w = imu_data[6]

        pub_odom.publish(odom)

        t = self.__fill_tf_message(
            odom.header.frame_id, odom.child_frame_id, base_pos[0:3], imu_data[3:7])
        self.robot_tf.sendTransform(t)

    def __pub_imu_msg(self, imu_data):
        pub_imu = rospy.Publisher("/imu0", Imu, queue_size=30)
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = imu_data[0]
        imu_msg.linear_acceleration.y = imu_data[1]
        imu_msg.linear_acceleration.z = imu_data[2]
        imu_msg.angular_body_velocity.x = imu_data[7]
        imu_msg.angular_body_velocity.y = imu_data[8]
        imu_msg.angular_body_velocity.z = imu_data[9]
        imu_msg.orientation.x = imu_data[3]
        imu_msg.orientation.y = imu_data[4]
        imu_msg.orientation.z = imu_data[5]
        imu_msg.orientation.w = imu_data[6]
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "body"
        pub_imu.publish(imu_msg)

    def __pub_joint_states(self, joint_states):
        pub_js = rospy.Publisher("joint_states", JointState, queue_size=30)
        js_msg = JointState()
        js_msg.name = []
        js_msg.position = []
        js_msg.body_velocity = []
        # TODO: Use joints length
        i = 0
        for _ in joint_states["name"]:
            js_msg.name.append(joint_states["name"][i].decode('utf-8'))
            js_msg.position.append(joint_states["state"][i])
            js_msg.body_velocity.append(joint_states["state"][12+i])
            i += 1
        js_msg.header.stamp = rospy.Time.now()
        js_msg.header.frame_id = "body"
        pub_js.publish(js_msg)


    def __get_motor_joint_states(self):

        # joint_number_range = self.cheetah.getDOF()
        joint_positions = self.cheetah.getGeneralizedCoordinate()
        joint_velocities = self.cheetah.getGeneralizedVelocity()
        joint_torques = self.cheetah.getGeneralizedForce()
        joint_name = 0

        # joint_pos(0-6) - xyz coord + body quaternion
        # joint_vel(0-5) - andgular vel + rotat vel
        return joint_positions[7:19], joint_velocities[6:18], joint_torques, joint_name


    def __get_data_from_sim(self):
        #make all data float clearly with float()
        rotation_mat = []
        get_body_velocity = []
        get_invert = []
        imu_data = [0] * 10
        leg_data = {}
        leg_data["state"] = [0] * 24
        leg_data["name"] = [""] * 12

        position = self.cheetah.getGeneralizedCoordinate()
        body_pos = position[0:3]
        body_quaternion = position[3:7]

        body_velocity = self.cheetah.getGeneralizedVelocity()
        body_velocity_lin = body_velocity[0:3]
        body_velocity_ang = body_velocity[3:6]
  

        # # IMU data (тут поменял порядок с 0-3 на 1-3,0)
        buffer = body_quaternion[0]
        body_quaternion[0:3] = body_quaternion[1:4]
        body_quaternion[3] = buffer

        imu_data[3] = float(body_quaternion[0])
        imu_data[4] = float(body_quaternion[1])
        imu_data[5] = float(body_quaternion[2])
        imu_data[6] = float(body_quaternion[3])

        #rotation matrix 4x4(use only 3x3) from body_quaternion
        rotation_mat = tf.transformations.quaternion_matrix(body_quaternion)

        #linear acceleration x,y,z
        acceleration = [0, 0, 0]
        acceleration[0] = (body_velocity_lin[0] - self.prev_body_velocity_lin[0]) * self.freq
        acceleration[1] = (body_velocity_lin[1] - self.prev_body_velocity_lin[1]) * self.freq
        acceleration[2] = (body_velocity_lin[2] - self.prev_body_velocity_lin[2]) * self.freq + 9.8

        imu_data[0] = float(rotation_mat[0][0] * acceleration[0] + rotation_mat[1][0]*acceleration[1] + rotation_mat[2][0] * acceleration[2])
        imu_data[1] = float(rotation_mat[0][1] * acceleration[0] + rotation_mat[1][1]*acceleration[1] + rotation_mat[2][1] * acceleration[2])
        imu_data[2] = float(rotation_mat[0][2] * acceleration[0] + rotation_mat[1][2]*acceleration[1] + rotation_mat[2][2] * acceleration[2])

        imu_data[7] = float(rotation_mat[0][0] * body_velocity_ang[0] + rotation_mat[1][0]*body_velocity_ang[1]+ rotation_mat[2][0] * body_velocity_ang[2])
        imu_data[8] = float(rotation_mat[0][1] * body_velocity_ang[0] + rotation_mat[1][1]*body_velocity_ang[1]+ rotation_mat[2][1] * body_velocity_ang[2])
        imu_data[9] = float(rotation_mat[0][2] * body_velocity_ang[0] + rotation_mat[1][2]*body_velocity_ang[1]+ rotation_mat[2][2] * body_velocity_ang[2])

        # joint data from motors
        joint_positions, joint_velocities, joint_torques, joint_names = self.__get_motor_joint_states()

        for i in range(12):
            leg_data["state"][i] = float(joint_positions[i])

        for i in range(12):
            leg_data["state"][i+12] = float(joint_velocities[i])

        for i in range(12):
            leg_data["name"][i] = str(i)

        #update prev ang vel
        self.prev_body_velocity_lin = [body_velocity_lin[0], body_velocity_lin[1], body_velocity_lin[2]]

        return imu_data, leg_data


if __name__ == '__main__':
    rospy.init_node('quadruped_simulator', anonymous=True)
    rospy.logwarn("node init")

    walking_simulation = WalkingSimulation()
    rospy.logwarn("WalkingSimulation class init")

    walking_simulation.run()
