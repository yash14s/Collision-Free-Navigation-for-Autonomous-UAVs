#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from dronekit.mavlink import MAVConnection
import math
import time

def connect_to_SITL_vehicle():
	global vehicle
	vehicle = connect('tcp:192.168.29.189:5762', wait_ready=True)
	print("Connected")

def connect_to_vehicle():
	global vehicle
	vehicle = connect('/dev/ttyUSB0', wait_ready=False, baud=57600)
	gcs_tcp = MAVConnection('tcpin:0.0.0.0:49152', source_system=1)
	vehicle._handler.pipe(gcs_tcp)
	gcs_tcp.master.mav.srcComponent = 1
	gcs_tcp.start()
	print("Connected")

def disconnect_to_vehicle():
	vehicle.close()
	print("Disconnected")

def arm():
	while vehicle.is_armable!=True:
		print("Undergoing pre-arm checks")
		time.sleep(1)
	print('Drone is now armable')
	vehicle.mode=VehicleMode('GUIDED')
	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Drone is now in GUIDED MODE.")
	vehicle.armed=True
	while vehicle.armed==False:
		print("Waiting for drone to become armed")
		time.sleep(1)
	print("Caution! Drone is ARMED!")

def disarm():
	print("Proceeding to disarm")
	vehicle.armed = False
	while(vehicle.armed==True):
		time.sleep(1)
	print("Drone is now DISARMED")

def takeoff(targetHeight):
	vehicle.simple_takeoff(targetHeight)#meters
	while True:
		print("Current Altitude: %f"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.9*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")

def land():
	vehicle.mode=VehicleMode("LAND")
	while vehicle.mode != 'LAND':
		print("Waiting for drone to enter LAND mode")
		time.sleep(1)
	print("Vehicle in LAND mode")

def get_distance_meters(waypoint,currentLocation):
	dLat = waypoint.lat - currentLocation.lat
	dLon = waypoint.lon - currentLocation.lon
	dAlt = waypoint.alt - currentLocation.alt
	distance = math.sqrt((dLon*dLon)+(dLat*dLat)+(dAlt*dAlt))
	print("Distance to wp : %f"%distance)
	return distance

def condition_yaw(degrees, relative, clk):
	if relative:
		is_relative=1
	else:
		is_relative=0
	if clk:
		is_clk=1
	else:
		is_clk=-1
	msg=vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,degrees,0,is_clk,is_relative,0,0,0)
	vehicle.send_mavlink(msg)
	print("Yawing")
	time.sleep(6)
	print("Yaw complete")

def send_local_ned_velocity(vx,vy,vz):
	msg=vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def send_global_ned_velocity(vx,vy,vz):
	msg=vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_LOCAL_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def stop():
	send_global_ned_velocity(0,0,0)

def nav_velocity(waypoint,speed):
	x_goal = waypoint.lat
	y_goal = waypoint.lon
	z_goal = waypoint.alt
	x_init = vehicle.location.global_relative_frame.lat
	y_init = vehicle.location.global_relative_frame.lon
	z_init = vehicle.location.global_relative_frame.alt
	theta = math.atan2(y_goal-y_init,x_goal-x_init)
	vx = speed * math.cos(theta)
	vy = speed * math.sin(theta)
	vz = 0.5 * (z_init-z_goal)
	if theta > 0:
		theta_deg = theta*180.0/math.pi
	else:
		theta_deg = 360 + theta*180.0/math.pi
	return vx,vy,vz,theta_deg

#Getting velocity output from the obstacle avoidance algorithm
def avoidance_callback(velocity_msg):
	global avoid_vx, avoid_vy, avoid_vz
	avoid_vx = velocity_msg.linear.x
	avoid_vy = velocity_msg.linear.y

rospy.init_node("safe_navigation_node", anonymous=False)
rospy.Subscriber("avoidance_velocity_topic", Twist, avoidance_callback)
rospy.on_shutdown(land)

def travel(waypoint,speed):
	print("Travelling to waypoint")
	distance_to_waypoint = get_distance_meters(waypoint,vehicle.location.global_relative_frame)
	vx,vy,vz,theta_deg = nav_velocity(waypoint,speed)
	condition_yaw(theta_deg,0,1)
	while True:
		if (not math.isnan(avoid_vx)) or (not math.isnan(avoid_vy)):
			send_local_ned_velocity(avoid_vx, avoid_vy, 0)
			rospy.loginfo("avoid_vx = %f , avoid_vy = %f", avoid_vx, avoid_vy)
		vx,vy,vz,theta_deg = nav_velocity(waypoint,speed)
		current_distance = get_distance_meters(waypoint,vehicle.location.global_relative_frame)
		send_global_ned_velocity(vx,vy,vz)
		if (current_distance<0.05*distance_to_waypoint):
			break
	print("******Reached******")
	stop()

#Dusshera park
wp1 = LocationGlobalRelative(26.916955,75.746240,1.65)
wp2 = LocationGlobalRelative(26.916763,75.746251,1.65)
wp3 = LocationGlobalRelative(26.916983,75.746571,1.65)
wp4 = LocationGlobalRelative(26.916761,75.746579,1.65)

#Lower height for SITL
wp5 = LocationGlobalRelative(26.916955,75.746240,0.6)
wp6 = LocationGlobalRelative(26.916763,75.746251,0.6)
wp7 = LocationGlobalRelative(26.9166363,75.7462531,0.6)
wp8 = LocationGlobalRelative(26.9166745,75.7463819,0.6)

connect_to_SITL_vehicle()
#connect_to_vehicle()
arm()
takeoff(0.6)
#travel(wp2,0.5)
#travel(wp1,0.5)
travel(wp6,1)
travel(wp7,1)
travel(wp8,1)
travel(wp5,1)
land()
disarm()
disconnect_to_vehicle()
