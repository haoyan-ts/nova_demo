#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy                                                                      # ROS2 Python接口库
from rclpy.node   import Node                                                     # ROS2 节点类
from dobot_msgs_v3.srv import *   # 自定义的服务接口      
from dobot_msgs_v3.msg import *
from rclpy.qos import qos_profile_system_default
import time

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                             # ROS2节点父类初始化

        self.key_sub = self.create_subscription(ToolVectorActual, 'key_vec', self.key_callback, qos_profile_system_default)

        self.EnableRobot_l = self.create_client(EnableRobot,'/dobot_bringup_v3/srv/EnableRobot')
        self.relmovL_l= self.create_client(RelMovL, '/dobot_bringup_v3/srv/RelMovL')
        self.SpeedFactor_l = self.create_client(SpeedFactor,'/dobot_bringup_v3/srv/SpeedFactor')

        # self.MovJ_l = self.create_client(MovJ,'/dobot_bringup_v3/srv/MovJ')
        # self.MovL_l = self.create_client(MovL,'/dobot_bringup_v3/srv/srv/MovL')
            # self.DO_l = self.create_client(DO,'/dobot_bringup_v3/srv/srv/DO' )
    
        
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):                  # 循环等待服务器端成功启动
            self.get_logger().info('service not available, waiting again...')

        self.initialization()
                    
    def initialization(self):  # 初始化：速度、坐标系、负载、工具偏心等
        response = self.EnableRobot_l.call_async()
        print(response)
        spe = SpeedFactor.Request()
        spe.ratio = 10
        response = self.SpeedFactor_l.call_async(spe)
        print(response)

    def key_callback(self, msg):
        if msg.x != 0 or msg.y != 0 or msg.z != 0 or msg.rx != 0 or msg.ry != 0 or msg.rz != 0:
            # Handle the non-zero message here
            rel_p1 = RelMovL.Request()
            rel_p1.offset1 = float(msg.x)
            rel_p1.offset2 = float(msg.y)
            rel_p1.offset3 = float(msg.z)
            rel_p1.offset4 = float(msg.rx)
            rel_p1.offset5 = float(msg.ry)
            rel_p1.offset6 = float(msg.rz)
            self.get_logger().info('Handling non-zero key vector: "%s"' % str(msg))
            response = self.relmovL_l.call_async(rel_p1)
            print(response)


        # self.get_logger().info('Received key vector: "%s"' % str(msg))

    # def point(self, Move, X_j1, Y_j2, Z_j3, RX_j4, RY_j5, RZ_j6):  # 运动指令
    #     if Move == "MovJ":
    #         P1 = MovJ.Request()
    #         P1.x = float(X_j1)
    #         P1.y = float(Y_j2)
    #         P1.z = float(Z_j3)
    #         P1.rx = float(RX_j4)
    #         P1.ry = float(RY_j5)
    #         P1.rz = float(RZ_j6)
    #         response = self.MovJ_l.call_async(P1)
    #         print(response)
    #     elif Move == "MovL":
    #         P1 = MovL.Request()
    #         P1.x = float(X_j1)
    #         P1.y = float(Y_j2)
    #         P1.z = float(Z_j3)
    #         P1.rx = float(RX_j4)
    #         P1.ry = float(RY_j5)
    #         P1.rz = float(RZ_j6)
    #         response = self.MovL_l.call_async(P1)
    #         print(response)
    #     else:
    #         print("无该指令")


    # def DO(self, index, status):  # IO 控制夹爪/气泵
    #     DO_V = DO.Request()
    #     DO_V.index = index
    #     DO_V.status = status
    #     response = self.DO_l.call_async(DO_V)
    #     print(response)


def main(args=None):
    rclpy.init(args=args)                                                         # ROS2 Python接口初始化
    node = adderClient("service_adder_client")                                    # 创建ROS2节点对象并进行初始化
    #node.send_request()                                                           
    
    # 发送服务请求
    # node.point("MovJ", 50, -8, 0, 0, 0, 0)
    # node.point("MovJ", 0, -8, 0, 0, 0, 0)  
    
    rclpy.spin(node)

    node.destroy_node()                                                           # 销毁节点对象
    rclpy.shutdown()                                                              # 关闭ROS2 Python接口
