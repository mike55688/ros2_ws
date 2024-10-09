#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from geometry_msgs.msg import Twist
from forklift_driver.msg import Meteorcar
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import qos_profile_sensor_data
import math
import rclpy.time
import tf_transformations
import rclpy
import rclpy.logging
from rclpy.node import Node
from enum import Enum
import time
import statistics
def fnCalcDistPoints(x1, x2, y1, y2):
    return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)



class Action():
    def __init__(self, TestAction):
        
        # cmd_vel
        self.TestAction = TestAction
        self.cmd_vel = cmd_vel(TestAction)
        # NearbySequence
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking ')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        # pallet variable
        self.pallet_2d_pose_x = 0.0
        self.pallet_2d_pose_y = 0.0
        self.pallet_2d_theta = 0.0
        self.pallet_2d_pose_z = 0.0  # 新增的z轴属性
        # Fork_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.005
        # other
        self.check_wait_time = 0
        self.is_triggered = False

   
   

    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, 
        self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta,
        self.pallet_2d_pose_x, self.pallet_2d_pose_y, self.pallet_2d_pose_z) = self.TestAction.SpinOnce()   #新增fruits的xyz軸属性

    
    def SpinOnce_fork(self):
        self.updownposition = self.TestAction.SpinOnce_fork()

    def fnseqDeadReckoning(self, dead_reckoning_dist):#(使用里程紀計算)移動到離現在位置dead_reckoning_dist公尺的地方, 1.0 = 朝向marker前進1公尺, -1.0 = 朝向marker後退1公尺
        self.SpinOnce()
        Kp = 0.2
        threshold = 0.015
        if self.is_triggered == False:
            self.is_triggered = True
            self.initial_robot_pose_x = self.robot_2d_pose_x
            self.initial_robot_pose_y = self.robot_2d_pose_y
        dist = math.copysign(1, dead_reckoning_dist) * fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
        if math.copysign(1, dead_reckoning_dist) > 0.0:
            if  (dead_reckoning_dist - dist - threshold) < 0.0:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            else:
                self.cmd_vel.fnGoStraight(Kp, -(dead_reckoning_dist - dist))
                return False
        elif math.copysign(1, dead_reckoning_dist) < 0.0:
            if  dead_reckoning_dist - dist > 0.0:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            else:
                self.cmd_vel.fnGoStraight(Kp, -(dead_reckoning_dist - dist))
                return False

    def fnseqMoveToMarkerDist(self, marker_dist): #(使用marker計算) 移動到距離marker_dist公尺的位置
        self.SpinOnce()
        Kp = 0.2
        if(marker_dist < 2.0):
            threshold = 0.015
        else:
            threshold = 0.03

        dist = math.sqrt(self.marker_2d_pose_x**2 + self.marker_2d_pose_y**2)
        
        if dist < (marker_dist-threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        elif dist > (marker_dist+threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        else:
            self.cmd_vel.fnStop()
            return True
        
    def fnSeqChangingtheta(self, threshod): #旋轉到marker的theta值為0, threshod為角度誤差值
        self.SpinOnce()
        Kp = 0.1
        # self.marker_2d_theta= self.TrustworthyMarker2DTheta(1)
        # print("desired_angle_turn", self.marker_2d_theta)
        # print("threshod", threshod)
        if abs(self.marker_2d_theta) < threshod  :
            self.cmd_vel.fnStop()
            if self.check_wait_time > 5 :
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
                return False
        else:
            self.cmd_vel.fnTurn(Kp, self.Actionmarker_2d_theta)
            self.check_wait_time =0
            return False
        
    def TrustworthyMarker2DTheta(self, duration): #用於計算marker的theta值再duration(取樣時間)中的平均值，避免在抖動的marker的theta值不穩定, 並且去除一個標準差外的極端值
        marker_2d_theta_list = [0.0]
        initial_time = self.TestAction.get_clock().now().nanoseconds / 1e9
        
        while(abs(initial_time - (self.TestAction.get_clock().now().nanoseconds / 1e9)) < duration):
            self.SpinOnce()
            marker_2d_theta_list.append(self.marker_2d_theta)
            # print("self.marker_2d_theta", self.marker_2d_theta)
            time.sleep(0.05)
        # print("marker_2d_theta_list", marker_2d_theta_list)
        threshold = 0.5
        mean = statistics.mean(marker_2d_theta_list)
        stdev = statistics.stdev(marker_2d_theta_list)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        clean_list = []
        for i in marker_2d_theta_list:
            if(i > downcutoff and i < upcutoff):
                clean_list.append(i)
        # print("clean_list", clean_list)
        # print("mean", statistics.mean(clean_list))
        return statistics.median(clean_list) 
    
    
    def fnSeqMovingNearbyParkingLot(self, desired_dist_threshold): #如果desired_dist的值小於desired_dist_threshold, 則不執行此動作
        self.SpinOnce()
        Kp = 0.2
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(3)
                self.initial_marker_pose_x = self.marker_2d_pose_x
                # print("initial_marker_pose_theta ", self.initial_marker_pose_theta)
                # decide doing fnSeqMovingNearbyParkingLot or not
                desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
                if abs(desired_dist) < desired_dist_threshold:
                    self.is_triggered = False
                    return True
            
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            
            desired_angle_turn = desired_angle_turn
            self.cmd_vel.fnTurn(Kp, desired_angle_turn)

            if abs(desired_angle_turn) < 0.03:
                self.cmd_vel.fnStop()
                if self.check_wait_time >10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            else:
                self.check_wait_time =0    

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

            dist_from_start = fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
            desired_dist = self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
            
            remained_dist = desired_dist + dist_from_start
            self.cmd_vel.fnGoStraight(Kp, desired_dist)

            if abs(remained_dist) < 0.02:
                self.cmd_vel.fnStop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value
                self.is_triggered = False


        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            # print("desired_angle_turn", desired_angle_turn)
            desired_angle_turn = -1. * desired_angle_turn
            self.cmd_vel.fnTurn(Kp, desired_angle_turn)
            if abs(desired_angle_turn) < 0.01:
                self.cmd_vel.fnStop()
                self.is_triggered = False
                return True
            # if abs(desired_angle_turn) < 0.01:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.initial_turn.value
            #         self.is_triggered = False
            #         return True                
            #     else:
            #         self.check_wait_time =self.check_wait_time  +1
            # elif abs(desired_angle_turn) < 0.02 and self.check_wait_time:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 20:
            #         self.check_wait_time = 0
            #         self.current_nearby_sequence = self.NearbySequence.initial_turn.value
            #         self.is_triggered = False
            #         return True                
            #     else:
            #         self.check_wait_time =self.check_wait_time  +1
            # else:
            #     self.check_wait_time =0    
        return False
    
    def fnSeqParking(self, parking_dist, kp):
        # 打印初始状态信息
        self.TestAction.get_logger().info(f"Initial marker_2d_pose_x: {self.pallet_2d_pose_x}, marker_2d_pose_y: {self.pallet_2d_pose_y}")
        self.TestAction.get_logger().info(f"Initial parking_dist: {parking_dist}, kp: {kp}")

        self.SpinOnce()
        desired_angle_turn = math.atan2(self.pallet_2d_pose_y - 0, self.pallet_2d_pose_x - 0)

        # 打印计算出的 desired_angle_turn
        self.TestAction.get_logger().info(f"Calculated desired_angle_turn before adjustment: {desired_angle_turn}")

        if desired_angle_turn < 0:
            desired_angle_turn = desired_angle_turn + math.pi
        else:
            desired_angle_turn = desired_angle_turn - math.pi

        # 打印调整后的 desired_angle_turn
        self.TestAction.get_logger().info(f"Adjusted desired_angle_turn: {desired_angle_turn}")

        self.cmd_vel.fnTrackMarker(desired_angle_turn, kp)
        
        # 打印当前 marker_2d_pose_x 和 check_wait_time 的状态
        self.TestAction.get_logger().info(f"Current marker_2d_pose_x: {self.pallet_2d_pose_x}, check_wait_time: {self.check_wait_time}")

        if abs(self.pallet_2d_pose_x) < parking_dist:
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10:
                self.TestAction.get_logger().info("Condition met: marker is within parking distance, stopping vehicle.")
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time += 1
                self.TestAction.get_logger().info(f"Waiting: incremented check_wait_time to {self.check_wait_time}")
        else:
            self.check_wait_time = 0
            self.TestAction.get_logger().info("Marker not yet within parking distance, resetting check_wait_time to 0.")
            return False


    
    def fnForkFruit(self, z_pose_threshold):#0~2.7  #透過marker的z軸位置來控制牙叉的上下
        self.SpinOnce_fork()
        self.SpinOnce()

        self.TestAction.get_logger().info(f"Marker 2D Pose: x={self.pallet_2d_pose_x}, y={self.pallet_2d_pose_y}, z={self.pallet_2d_pose_z}")
        if( self.pallet_2d_pose_z < -z_pose_threshold):
            self.cmd_vel.fnfork(2000.0)
            self.TestAction.get_logger().info("Fork up")
            return False

        elif (self.pallet_2d_pose_z > z_pose_threshold):
            self.cmd_vel.fnfork(-2000.0)
            self.TestAction.get_logger().info("Fork down")
            return False
        else :
            self.cmd_vel.fnfork(0.0)
            self.TestAction.get_logger().info("Fork stop")
            return True
        
    def fnForkFruit_approach(self, x_pose_threshold):#0~2.7  #透過marker的x軸位置來控制叉車前進
        self.SpinOnce_fork()
        self.SpinOnce()

        self.TestAction.get_logger().info(f"Marker 2D Pose: x={self.pallet_2d_pose_x}, y={self.pallet_2d_pose_y}, z={self.pallet_2d_pose_z}")
        if( self.pallet_2d_pose_x < x_pose_threshold):
            self.cmd_vel.fnGoStraight_fruit()
            self.TestAction.get_logger().info("GoStraight")
            return False
        else:
            self.cmd_vel.fnStop()
            self.TestAction.get_logger().info("Stop")
            return True

    def fnSeqdecide(self, decide_dist):#decide_dist偏離多少公分要後退
        self.SpinOnce()
        dist = self.marker_2d_pose_y
        if  abs(dist) < abs(decide_dist):
            return True
        else:
            return False

    def fnForkUpdown(self, desired_updownposition):#0~2.7
        self.SpinOnce_fork()
        fork_threshold = 0.001
        if(desired_updownposition < 0):
            return True

        if self.updownposition < desired_updownposition - fork_threshold:
            self.cmd_vel.fnfork(2000.0)
            return False
        dist = self.marker_2d_pose_y
        if self.updownposition > desired_updownposition + fork_threshold:
            self.cmd_vel.fnfork(-2000.0)
            return False
        else:
            self.cmd_vel.fnfork(0.0)
            return True


        
class cmd_vel():
    def __init__(self, TestAction):
        self.pub_cmd_vel = TestAction.cmd_vel_pub
        self.pub_fork = TestAction.fork_pub

    def cmd_pub(self, twist): #限制速度的範圍
        if twist.linear.x > 0.2:
            twist.linear.x =0.2
        elif twist.linear.x < -0.2:
            twist.linear.x =-0.2
        if twist.linear.x > 0 and twist.linear.x < 0.02:
            twist.linear.x =0.02
        elif twist.linear.x < 0 and twist.linear.x > -0.02:
            twist.linear.x =-0.02

        if twist.angular.z > 0.2:
            twist.angular.z =0.2
        elif twist.angular.z < -0.2:
            twist.angular.z =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.05:
            twist.angular.z =0.05
        elif twist.angular.z < 0 and twist.angular.z > -0.05:
            twist.angular.z =-0.05
        
        self.pub_cmd_vel.publish(twist)

    def fnStop(self):
        twist = Twist()
        self.cmd_pub(twist)

    def fnTurn(self, Kp=0.2, theta=0.):
        twist = Twist()
        twist.angular.z = Kp * theta
        self.cmd_pub(twist)

    def fnGoStraight(self, Kp=0.2, v=0.):
        twist = Twist()
        twist.linear.x = Kp * v
        self.cmd_pub(twist)

    def fnfork(self,velocity):
        fork_msg = Meteorcar()
        fork_msg.fork_velocity = velocity
        self.pub_fork.publish(fork_msg)

    def fnTrackMarker(self, theta, Kp):
        # Kp = 2.0 #6.5
        twist = Twist()
        twist.linear.x = -0.025
        twist.angular.z = Kp * theta
        self.cmd_pub(twist)

        
    def SpinOnce_fork(self):
        rclpy.spin_once(self)
        return self.updownposition

    def fnGoStraight_fruit(self):      #控制叉車前進
        twist = Twist()
        twist.linear.x = -0.05
        self.cmd_pub(twist)

def main(args=None):
    rclpy.init(args=args)

    test_action = TestAction()
    try:
        rclpy.spin(test_action)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

'''    def SpinOnce_fork(self):
        rclpy.spin_once(self)
        return self.updownposition
    
def main(args=None):
    rclpy.init(args=args)

    test_action = TestAction()
    try:
        rclpy.spin(test_action)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

'''