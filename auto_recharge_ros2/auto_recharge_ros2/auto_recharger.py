#!/usr/bin/env python3 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作
#2:Python.源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

#引用ros库
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from rclpy.duration import Duration

# 用到的变量定义
from std_msgs.msg import Bool 
from std_msgs.msg import Int8 
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from turtlesim.srv import Spawn

# 用于记录充电桩位置、发布导航点
from geometry_msgs.msg import PoseStamped

# rviz可视化相关
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# cmd_vel话题数据
from geometry_msgs.msg import Twist

# 里程计话题相关
from nav_msgs.msg import Odometry

# 获取导航结果
# from move_base_msgs.msg import MoveBaseActionResult # ROS1

# 键盘控制相关
import sys, select, termios, tty

# 延迟相关
import time

# 读写充电桩位置文件
import json
import yaml

import math
import os

#存放充电桩位置的文件位置
json_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json'
yaml_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/robot_info.yaml'
#print_and_fixRetract相关，用于打印带颜色的信息
RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

#圆周率
PI=3.1415926535897

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty
	
settings = None
if os.name != 'nt' and sys.stdin.isatty():
	settings = list(termios.tcgetattr(sys.stdin))

def get_key(settings):
	if os.name == 'nt':
		return msvcrt.getch().decode('utf-8')
	if sys.stdin.isatty():
		tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	if sys.stdin.isatty():
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def print_and_fixRetract(str):
	global settings
	'''键盘控制会导致回调函数内使用print()出现自动缩进的问题，此函数可以解决该现象'''
	if sys.stdin.isatty():
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	print(str)

class AutoRecharger(Node):
	def __init__(self):
		
        #创建节点
		super().__init__("auto_recharger")

		print_and_fixRetract('Automatic charging node start!')

        #机器人状态变量：类型、电池容量、电池电压、充电状态、充电电流、红外信号状态、记录机器人姿态
		self.robot = {
        'Type':'Plus', 
        'BatteryCapacity':5000, 
        'Voltage':25, 
        'Charging':0, 
        'Charging_current':0, 
        'RED':0, 
        'Rotation_Z':0,
        'car_mode':'mini_mec'
        }

        #用于记录导航结束是的机器人Z轴姿态
		self.nav_end_z=0
		self.start_turn = 0
		self.find_redsignal = 0

		#红外信号的数量
		self.red_count=0
		#机器人自动回充模式标志位，0：关闭回充，1：导航回充，2：回充装备控制回充
		self.chargeflag=0
		#机器人时间戳记录变量
		self.last_time= self.get_clock().now()
		#机器人红外信号丢失的时间滤波2
		self.lost_red_flag=self.get_clock().now()
		#机器人电量过低(<12.5或者<25)计数
		self.power_lost_count=0
		#机器人低电量检测1次标志位
		self.lost_power_once=1
		#机器人充电完成标志位
		self.charge_complete=0
		#机器人充电完成标志位
		self.last_charge_complete=0
		#最新充电桩位置数据
		self.json_data=0
		#是否监听导航结果标志位
		self.star_getNav_Feedback_Flag=0

		# 用户标记点和实际导航点直线偏移的距离，单位m
		self.diff_point = 1.2

		# 偏移的角度
		self.diff_angle = -15

		self.nav_controller =  BasicNavigator()

		#读取json文件内保存的充电桩位置信息
		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)

		self.robot_security_off_pub = self.create_publisher(Int8,'/chassis_security',   10) 

		#创建充电桩位置标记话题发布者
		self.Charger_marker_pub   = self.create_publisher(MarkerArray,'/goal_marker',   10) 

		#创建自动回充任务是否开启标志位话题发布者
		self.Recharger_Flag_pub = self.create_publisher(Int8,"robot_recharge_flag",  5)

		#速度话题用于不开启导航时，向底盘发送开启自动回充任务命令
		self.Cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",  5)

		#创建机器人电量话题订阅者
		self.Voltage_sub = self.create_subscription(Float32, "PowerVoltage", self.Voltage_callback,10)

		#创建机器人充电状态话题订阅者
		self.Charging_Flag_sub = self.create_subscription(Bool, "robot_charging_flag",self.Charging_Flag_callback,10)

		#创建机器人充电电流话题订阅者
		self.Charging_Current_sub = self.create_subscription(Float32,"robot_charging_current",  self.Charging_Current_callback,10)

		#创建机器人已发现红外信号话题订阅者
		self.RED_Flag_sub = self.create_subscription(UInt8,"robot_red_flag",  self.RED_Flag_callback,10)

		#创建充电桩位置更新话题订阅者
		self.Charger_Position_Update_sub = self.create_subscription( PoseStamped,"/charger_position_update", self.Position_Update_callback,10)

		#创建里程计话题订阅者
		self.Odom_sub = self.create_subscription(Odometry, '/odom', self.Odom_callback,10)

		# 创建服务调用者
		self.set_charge = self.create_client(Spawn,'/set_charge')

		self.server_set_state = None
		self.wait_server_done = None
		#按键控制说明
		self.tips = """
使用下面按键使用自动回充功能.       Press below Key to AutoRecharger.
Q/q:开启自动回充.                   Q/q:Start Navigation to find charger.
E/e:停止自动回充.                   E/e:Stop find charger.
Ctrl+C/c:关闭自动回充功能并退出.    Ctrl+C/c:Quit the program.
可使用话题"charger_position_update"更新充电桩的位置.
		"""
	def wait_server_callback(self,res):
		self.wait_server_done = 1
		response = res.result()
		self.server_set_state = response.name

	# 设置自动回充的状态
	def set_charge_mode(self,value,max_callcount=10):
		# 注:不可在回调函数调用服务,否则卡死
		try:
			if not self.set_charge.wait_for_service(2):
				raise TimeoutError("Service call time out")
	
		except TimeoutError as e:
			print_and_fixRetract(RED+'自动回充状态设置失败.原因:等待服务超时,请确认底盘节点是否被开启.'+RESET)
			return

		state = None  # 调用结果
		call_time = 0 # 调用失败后尝试调用的次数记录
		if round(value)==1 or round(value)==2:
			print("正在开启自动回充功能,等待响应,请确保底盘节点已开启...")
		else:
			print("正在关闭自动回充功能,等待响应,请确保底盘节点已开启...")
		while True:
			try:
				req = Spawn.Request()
				req.x = float(value)
				self.set_charge.call_async(req).add_done_callback(self.wait_server_callback)

				# 死循环等待响应结果
				while True:
					rclpy.spin_once(self)
					if self.wait_server_done==1:
						self.wait_server_done = 0
						break
				# 输出结果
				state = self.server_set_state
				
			except Exception  as e:
				print(e)
				state =  "false"

			if state=="true":
				self.server_set_state = None
				print("回充状态设置成功.")
				# 调用成功,跳出循环
				break
			else:
				# 记录失败的次数
				call_time = call_time + 1
				if call_time>max_callcount:
					print_and_fixRetract(RED+'尝试与底盘通信多次失败,无法开启自动回充功能,请检查底层设备是否正确.'+RESET)
					self.server_set_state = None
					break
			
			time.sleep(0.5)
				

	def Pub_Charger_Position(self):
		'''使用最新充电桩位置发布导航目标点话题'''
		# 开始监听导航结果
		self.star_getNav_Feedback_Flag=1

		nav_goal=PoseStamped()
		nav_goal.header.frame_id = 'map'
		nav_goal.header.stamp = self.get_clock().now().to_msg()
		nav_goal.pose.position.x = self.json_data['p_x']
		nav_goal.pose.position.y = self.json_data['p_y']
		nav_goal.pose.orientation.z = self.json_data['orien_z']
		nav_goal.pose.orientation.w = self.json_data['orien_w']

		# 发布充电桩位置的可视化
		self.Pub_Charger_marker(
			self.json_data['p_x'], 
			self.json_data['p_y'], 
			self.json_data['orien_z'], 
			self.json_data['orien_w'])
		
		# 导航到充电桩
		self.nav_controller.goToPose(nav_goal)

	def Pub_NavGoal_Cancel(self):
		'''取消导航'''
		# 导航被取消,停止监听导航结果事件
		self.star_getNav_Feedback_Flag = 0
		self.nav_controller.cancelTask()

	def Pub_Charger_marker(self, p_x, p_y, o_z, o_w):
		'''发布目标点可视化话题'''
		
		# 实际充电桩位置已经被偏移过，这里可视化转换成用户在rviz上标定的坐标	
		# 角度偏移
		tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
		tmp_angle = math.radians(-self.diff_angle)
		new_yaw = (tmp_yaw+tmp_angle)/2
		o_z = math.sin(new_yaw)
		o_w = math.cos(new_yaw)

		tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
		diff_x = math.cos(tmp_yaw)
		diff_y = math.sin(tmp_yaw)
		p_x = p_x - diff_x*self.diff_point
		p_y = p_y - diff_y*self.diff_point

		markerArray = MarkerArray()

		marker_shape  = Marker() #创建marker对象
		marker_shape.id = 0 #必须赋值id
		marker_shape.header.frame_id = 'map' #以哪一个TF坐标为原点
		marker_shape.type = Marker.ARROW #TEXT_VIEW_FACING #一直面向屏幕的字符格式
		marker_shape.action = Marker.ADD #添加marker
		marker_shape.scale.x = 0.5 #marker大小
		marker_shape.scale.y = 0.05 #marker大小
		marker_shape.scale.z = 0.05 #marker大小，对于字符只有z起作用
		marker_shape.pose.position.x = p_x#字符位置
		marker_shape.pose.position.y = p_y #字符位置
		marker_shape.pose.position.z = 0.1 #msg.position.z #字符位置
		marker_shape.pose.orientation.z = o_z #字符位置
		marker_shape.pose.orientation.w = o_w #字符位置
		marker_shape.color.r = 1.0 #字符颜色R(红色)通道
		marker_shape.color.g = 0.0 #字符颜色G(绿色)通道
		marker_shape.color.b = 0.0 #字符颜色B(蓝色)通道
		marker_shape.color.a = 1.0 #字符透明度
		markerArray.markers.append(marker_shape) #添加元素进数组
		
		marker_string = Marker() #创建marker对象
		marker_string.id = 1 #必须赋值id
		marker_string.header.frame_id = 'map' #以哪一个TF坐标为原点
		marker_string.type = Marker.TEXT_VIEW_FACING #一直面向屏幕的字符格式
		marker_string.action = Marker.ADD #添加marker
		marker_string.scale.x = 0.5 #marker大小
		marker_string.scale.y = 0.5 #marker大小
		marker_string.scale.z = 0.5 #marker大小，对于字符只有z起作用
		marker_string.color.a = 1.0 #字符透明度
		marker_string.color.r = 1.0 #字符颜色R(红色)通道
		marker_string.color.g = 0.0 #字符颜色G(绿色)通道
		marker_string.color.b = 0.0 #字符颜色B(蓝色)通道
		marker_string.pose.position.x = p_x #字符位置
		marker_string.pose.position.y = p_y #字符位置
		marker_string.pose.position.z = 0.1 #msg.position.z #字符位置
		marker_string.pose.orientation.z = o_z #字符位置
		marker_string.pose.orientation.w = o_w #字符位置
		marker_string.text = 'Charger' #字符内容
		markerArray.markers.append(marker_string) #添加元素进数组
		self.Charger_marker_pub.publish(markerArray) #发布markerArray，rviz订阅并进行可视化

	# def Pub_Recharger_Flag(self):
	# 	'''发布自动回充任务是否开启标志位话题'''
	# 	# topic=Int8()
	# 	# topic.data=self.chargeflag
	# 	# for i in range(10):
	# 	# 	self.Recharger_Flag_pub.publish(topic)
						
	# 	topic = Twist()
	# 	for i in range(5):
	# 		self.Cmd_vel_pub.publish(topic)

	def Pub_Recharger_Flag(self,set_velflag=0):
		'''发布自动回充任务是否开启标志位话题'''

        # 先开回充，再开导航的情况
		if set_velflag==1:
			topic=Int8()
			topic.data=self.chargeflag
			for i in range(10):
				self.Recharger_Flag_pub.publish(topic)
		
		self.set_charge_mode(self.chargeflag)

	def Voltage_callback(self, topic):
		'''更新机器人电池电量'''
		self.robot['Voltage']=topic.data

	def Charging_Flag_callback(self, topic):
		'''更新机器人充电状态'''
		if(self.robot['Charging']==0 and topic.data==1):
			print_and_fixRetract(GREEN+"Charging started!"+RESET)
		if(self.robot['Charging']==1 and topic.data==0):
			print_and_fixRetract(YELLOW+"Charging disconnected!"+RESET)
		self.robot['Charging']=topic.data

	def Charging_Current_callback(self, topic):
		'''更新机器人充电电流数据'''
		self.robot['Charging_current']=topic.data
		
	def RED_Flag_callback(self, topic):
		self.red_count = topic.data
		'''更新是否寻找到红外信号(充电桩)状态'''
		if self.robot['Charging']==0:
			#如果是导航寻找充电桩模式，红外信号消失时
			if topic.data==0 and self.robot['RED']==1:
				if((self.get_clock().now()-self.lost_red_flag).to_msg()).sec>=2:
					print_and_fixRetract(YELLOW+"Infrared signal lost."+RESET)
				self.lost_red_flag = self.get_clock().now()
	
			#红外信号出现
			if topic.data==1 and self.robot['RED']==0:
				print_and_fixRetract(GREEN+"Infrared signal founded."+RESET) 

		if topic.data>0:
			self.robot['RED']=1
		else:
			self.robot['RED']=0

		# 自转寻找红外时处理逻辑
		if self.start_turn==1:
			if self.robot['RED']==1:
				self.find_redsignal = self.find_redsignal + 1 
				# print(self.find_redsignal)
				# if self.find_redsignal>=3: # 稳定识别一段时间
				# 	self.find_redsignal = 0 
				# 	self.start_turn=0
				# 	print_and_fixRetract(GREEN+'已通过自转发现红外信号,开始对接充电.(Infrared signals have been detected by rotation. Docking and charging has begun.)'+RESET)
				# 	vel_topic=Twist()
				# 	self.Cmd_vel_pub.publish(vel_topic) # 停止运动
				# 	self.chargeflag=1 # 开启自动回充
				# 	self.Pub_Recharger_Flag()
			else:
				self.find_redsignal = 0

	def Position_Update_callback(self, topic):
		'''更新json文件中的充电桩位置'''
		position_dic={'p_x':0, 'p_y':0, 'orien_z':0, 'orien_w':0 }
		position_dic['p_x']=topic.pose.position.x
		position_dic['p_y']=topic.pose.position.y
		position_dic['orien_z']=topic.pose.orientation.z
		position_dic['orien_w']=topic.pose.orientation.w

		# 以用户标定的充电桩位置为基础，直线前移1.2米作为真实导航点
		tmp_yaw = math.atan2(2*(position_dic['orien_w']*position_dic['orien_z']),1-2*(position_dic['orien_z']**2))
		diff_x = math.cos(tmp_yaw)
		diff_y = math.sin(tmp_yaw)
		position_dic['p_x'] = position_dic['p_x'] + diff_x*self.diff_point
		position_dic['p_y'] = position_dic['p_y'] + diff_y*self.diff_point

		# 角度偏移
		tmp_angle = math.radians(self.diff_angle)
		new_yaw = (tmp_yaw+tmp_angle)/2
		position_dic['orien_z'] = math.sin(new_yaw)
		position_dic['orien_w'] = math.cos(new_yaw)

		#保存最新的充电桩位置到json文件
		with open(json_file, 'w') as fp:
			json.dump(position_dic, fp, ensure_ascii=False)
			print_and_fixRetract("New charging pile position saved.")
		#更新最新的充电桩位置数据
		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)

		#发布最新的充电桩位置话题
		# self.Pub_Charger_marker(position_dic['p_x'], position_dic['p_y'], position_dic['orien_z'], position_dic['orien_w'])

	# # 导航结果订阅函数
	# def Nav_Result_callback(self, topic):
		
	# 	# 目标点取消，一般是多点导航
	# 	if 'canceled' in topic.status.text:
	# 		return

	# 	# 导航点是充电桩位置
	# 	if self.nav_end_rc_flag == 1:
	# 		self.nav_end_rc_flag = 0
	# 		if 'Failed' in topic.status.text:
	# 			print_and_fixRetract('无法导航到充电桩位置,请检查充电桩附近是否存在障碍物.(Cannot navigate to the charging station, please check if there are any obstacles near the charging station.)')
	# 			return
			
	# 		if 'Aborting' in topic.status.text:
	# 			print_and_fixRetract('充电桩位置数据异常,请尝试重新标定.(Charging post position data is abnormal, please try to re-calibrate.)')
	# 			return
			
	# 		if 'oscillating' in  topic.status.text:
	# 			print_and_fixRetract(YELLOW+'WARNING:请清除充电桩附近障碍物.(Please remove obstacles near the charging post.)'+RESET)
	# 			return

	# 		# 导航到达充电桩处或者无法完成充电桩处的导航但是存在红外信号，则开启自动回充
	# 		if self.robot['RED']==1:
	# 			print_and_fixRetract(GREEN+'已到达充电桩位置,开始对接充电.(Arrived at the charging station and started charging.)'+RESET)
	# 			# 开启自动回充
	# 			self.chargeflag=1
	# 			self.Pub_Recharger_Flag()

	# 		else:
	# 			self.lost_power_once=1 # 未找到充电桩时恢复允许低电量导航
	# 			if 'akm' in self.robot['car_mode']:
	# 				print_and_fixRetract(YELLOW+'未找到充电桩,自动回充功能已停止.(Charging station not found.)'+RESET)		
	# 			else:
	# 				# 开启自动回充功能并开始旋转一圈来寻找充电桩
	# 				print_and_fixRetract(YELLOW+'未找到充电桩,开始自转寻找红外信号.(Charging station not found. Starting to rotate in search of infrared signal.)'+RESET)
	# 				time.sleep(1)
	# 				topic=Twist()
	# 				topic.angular.z = 0.2
	# 				self.Cmd_vel_pub.publish(topic) 
	# 				self.start_turn = 1
	# 				self.nav_end_z = self.robot['Rotation_Z']



	def Odom_callback(self, topic):
		'''更新的机器人实时位姿'''
		self.robot['Rotation_Z']=topic.pose.pose.position.z	 

	def Stop_Charge(self):
		#如果在导航回充模式下，关闭导航
		self.Pub_NavGoal_Cancel() 

		# 停止监听导航结果
		self.star_getNav_Feedback_Flag = 0

		self.lost_power_once=1
		
		#切换为停止回充模式
		self.chargeflag=0
		self.Pub_Recharger_Flag()
		#发布速度为0的话题停止机器人运动
		topic=Twist()
		self.Cmd_vel_pub.publish(topic)
		#如果机器人在充电，控制机器人离开充电桩
		if self.robot['Charging']==1:
			topic=Twist()
			topic.linear.x = 0.1
			self.Cmd_vel_pub.publish(topic)

			# 不同小车电机速度响应时间不同
			if 'mini' in self.robot['car_mode'] or 'akm' in self.robot['car_mode']: 
				time.sleep(3)
			else:
				time.sleep(1)
			topic.linear.x = 0.0
			self.Cmd_vel_pub.publish(topic)

	def autoRecharger(self, key):
		'''键盘控制开始自动回充:1-导航控制寻找充电桩,2-纯回充装备控制寻找充电桩
		'''
		# 如果机器人在充电中,则检测充电是否已经完成.
		if self.robot['Charging']==1:
			if (self.robot['Type']=='Plus'and self.robot['Voltage']>25) or (self.robot['Type']=='Mini' and self.robot['Voltage']>12.5):
				self.charge_complete=self.charge_complete+1
			else:
				self.charge_complete=0

		#导航控制寻找充电桩
		if key=='q' or key=='Q':
			# 存在3路以上的红外信号,小车姿态接近于对准充电桩,无需导航	
			if self.red_count>=3:
				self.Pub_NavGoal_Cancel()
				self.chargeflag=1
				self.Pub_Recharger_Flag()
				print_and_fixRetract('已捕获到高强度红外信号,使用红外信号对接.(High-intensity infrared signals have been captured and are docked using infrared signals.)')
			else:
				self.Pub_Charger_Position() 
				print_and_fixRetract('开始导航到充电桩位置.(Start navigating to the charging post location.)')

		#关闭自动回充 
		elif key=='e' or key=='E':
			print_and_fixRetract('停止寻找充电桩或停止充电.(Stop finding charging pile or charging.)')
			self.Stop_Charge()

		# 测试用
		elif key=='t' or key=='T':
			self.set_charge_mode(1)
		elif key=='y' or key=='Y':
			self.set_charge_mode(0)

		#电压过低时开启导航自动回充
		if self.robot['Charging']==0:
			if (self.robot['Type']=='Plus'and self.robot['Voltage']<20) or (self.robot['Type']=='Mini' and self.robot['Voltage']<10):
				time.sleep(1)
				self.power_lost_count=self.power_lost_count+1 # 低电量滤波

				# 低电量状态超过5次
				if self.power_lost_count>5 and self.lost_power_once==1:
					self.power_lost_count=0

					# 电量低且小车不在回充模式,开启导航充电
					if self.chargeflag==0:
						self.Pub_NavGoal_Cancel() # 取消导航

						if 'akm' in self.robot['car_mode']:
							self.chargeflag=2
						else:
							self.chargeflag=1
						self.Pub_Recharger_Flag(1) # 出现要优先开启自动回充然后再导航的情况,需要进行标志位传递
						self.Pub_Charger_Position()
						print_and_fixRetract(YELLOW+'检测到电池电量低,即将导航到充电桩进行充电.(Detects low battery level and will navigate to a charging station for charging.)'+RESET)
						self.lost_power_once=0
	
			else:
				self.power_lost_count=0		

		# #频率1hz的循环任务
		if ((self.get_clock().now()-self.last_time).to_msg()).sec>=1:
			#发布充电桩位置话题
			self.Pub_Charger_marker(
				self.json_data['p_x'], 
				self.json_data['p_y'], 
				self.json_data['orien_z'], 
				self.json_data['orien_w'])
			self.last_time=self.get_clock().now()


			# 需要监听导航结果
			res = None
			nav_feedback = None
			if self.star_getNav_Feedback_Flag==1:
				# 等待导航结束
				if self.nav_controller.isTaskComplete()==True:
					self.star_getNav_Feedback_Flag = 0 # 导航任务结束,结束监听
					res = self.nav_controller.getResult()
					if res==TaskResult.SUCCEEDED:
						print("已到达目标点.")
						if self.robot['RED']==1:
							# 成功到达目标点,开启自动回充
							print("发现红外信号,开启对接功能.")
							self.chargeflag=1
							self.Pub_Recharger_Flag()
						else:
							print('未发现红外信号,开始自转寻找.')
							self.nav_end_z = self.robot['Rotation_Z']
							self.start_turn=1 # 没有红外信号,让小车自转
							topic=Twist()
							topic.angular.z = 0.2
							self.Cmd_vel_pub.publish(topic) 
					
					# 导航被取消了
					elif res==TaskResult.CANCELED:
						print_and_fixRetract('nav was canceled.') 

					elif res==TaskResult.FAILED:
						# 导航失败,可能是用户使用rviz新建了目标点,也可能是无法规划到目的
						print_and_fixRetract('goal failed.')
				else:
					# 获取反馈
					nav_feedback = self.nav_controller.getFeedback()
					if nav_feedback!=None:
						if nav_feedback.distance_remaining < 0.2 and Duration.from_msg(nav_feedback.navigation_time) > Duration(seconds=120.0):
							print_and_fixRetract('长时间无法到达目标点,导航已取消')
							self.Pub_NavGoal_Cancel()
							if self.robot['RED']==1:
								self.chargeflag=1
								self.Pub_Recharger_Flag()
					else:
						pass

			#充电期间打印电池电压、充电时间
			if self.robot['Charging']==1:
				self.lost_power_once=1
				percent=0
				percen_form=0
				if self.robot['Type']=='Plus':
					percent= (self.robot['Voltage']-20)/5 
					percent_form=format(percent, '.0%')
				if self.robot['Type']=='Mini':
					percent= (self.robot['Voltage']-10)/2.5
					percent_form=format(percent, '.0%')
				print_and_fixRetract("Robot is charging.")
				print_and_fixRetract("Robot battery: "+str(round(self.robot['Voltage'], 2))+"V = "+str(percent_form)+
									 ", Charging current: "+str(round(self.robot['Charging_current'], 2))+"A.")
				mAh_time=0
				try:
					mAh_time=1/self.robot['Charging_current']/1000
				except ZeroDivisionError:
					pass
				left_battery=round(self.robot['BatteryCapacity']*percent, 2)
				if percent<1:
					need_charge_battery=self.robot['BatteryCapacity']-left_battery
					need_percent_form=format(1-percent, '.0%')		
					print_and_fixRetract(str(self.robot['BatteryCapacity'])+"mAh*"+str(need_percent_form)+"="+str(need_charge_battery)+"mAh need to be charge, "+
										 "cost "+str(round(need_charge_battery*mAh_time, 2))+" hours.")
				else:	
					print_and_fixRetract(GREEN+"Robot battery is full."+RESET)
				print_and_fixRetract("\n")

		 # 自转寻找红外执行判断
		if self.find_redsignal>=3:
			self.find_redsignal = 0 
			self.start_turn=0
			print_and_fixRetract(GREEN+'已通过自转发现红外信号,开始对接充电.(Infrared signals have been detected by rotation. Docking and charging has begun.)'+RESET)
			vel_topic=Twist()
			self.Cmd_vel_pub.publish(vel_topic) # 停止运动
			self.chargeflag=1 # 开启自动回充
			self.Pub_Recharger_Flag()

        # 非阿克曼车型如果导航到终点没有红外信号,则自传一圈寻找
		if self.start_turn == 1:
			if abs(self.robot['Rotation_Z']-self.nav_end_z)>2*PI:
				self.start_turn=0
				self.Stop_Charge()
				print_and_fixRetract(RED+'自转已完成,无法找到充电桩位置,已停止自动回充.(Rotation completed, unable to locate charging station, automatic recharging has been stopped.)'+RESET)

		#机器人充电完成判断
		if self.charge_complete>10:
			self.charge_complete=0
			if self.last_charge_complete!=0:
				self.last_charge_complete=0
				self.Stop_Charge()			
			print_and_fixRetract(GREEN+'充电已完成.(Chrge complete.)'+RESET)#Charging complete
		self.last_charge_complete=self.charge_complete

	
def main():
	rclpy.init()
	try:
		autorecharger=AutoRecharger() #创建自动回充类

		print_and_fixRetract("请开启导航功能,正在等待导航激活...(Wait for navigation now...)")
		autorecharger.nav_controller.waitUntilNav2Active()
		print_and_fixRetract(autorecharger.tips)
		
		tmp_sec = Int8()
		tmp_sec.data = 1
		tmp_vel = Twist()
		autorecharger.robot_security_off_pub.publish(tmp_sec)
		autorecharger.Cmd_vel_pub.publish(tmp_vel)
		
		# 初始化参数
		autorecharger.declare_parameter('robot_BatteryCapacity',5000)
		autorecharger.declare_parameter('car_mode',"mini_mec")
		autorecharger.declare_parameter('diff_point',1.2)
		autorecharger.declare_parameter('diff_angle',-15)

		# 获取参数
		with open(yaml_file,'r') as file:
			params = yaml.safe_load(file)

		# 参数赋值
		autorecharger.robot['BatteryCapacity'] = params['robot_info']['BatteryCapacity']
		autorecharger.robot['car_mode']        = params['robot_info']['car_mode']
		autorecharger.diff_point = params['robot_info']['diff_point']
		autorecharger.diff_angle = params['robot_info']['diff_angle']

		if autorecharger.robot['car_mode'][0:4]!='mini':
			autorecharger.robot['Type'] = 'Plus'
		else:
			autorecharger.robot['Type'] = 'Mini'
		
		while rclpy.ok():
			key = get_key(settings) #获取键值，会导致终端打印自动缩进
			autorecharger.autoRecharger(key) #开始自动回充功能
			rclpy.spin_once(autorecharger)
			if (key == '\x03'):
				topic=Twist()
				autorecharger.Cmd_vel_pub.publish(topic) #发布速度0话题
				autorecharger.chargeflag=0
				autorecharger.Pub_Recharger_Flag()# 关闭自动回充
				print_and_fixRetract('自动回充功能已关闭.(Quit AutoRecharger.)')#Auto charging quit
				break #Ctrl+C退出自动回充功能
		
	except Exception as e:
		print_and_fixRetract(e)
	
	finally:
		topic=Twist()
		autorecharger.Cmd_vel_pub.publish(topic) #发布速度0话题
		autorecharger.chargeflag=0
		autorecharger.Pub_Recharger_Flag()# 关闭自动回充,并传递标志位到cmd_vel的callback函数
		# 恢复终端属性
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	
	print('over.')
