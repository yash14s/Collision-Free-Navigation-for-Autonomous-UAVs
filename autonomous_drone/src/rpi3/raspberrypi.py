#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import math

def connect_to_vehicle():
    global vehicle
    vehicle = connect('tcp:192.168.29.189:5762', wait_ready=True)
    print("Connected")

def disconnect_to_vehicle():
    vehicle.close()
    print("Disconnected")
    
def arm():
    connect_to_vehicle()
    while vehicle.is_armable!=True:
        print("Undergoing pre-arm checks")
        time.sleep(1)
    print('Drone is now armable')
    vehicle.mode=VehicleMode('GUIDED')
    while vehicle.mode!='GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Drone now in GUIDED MODE.")
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
    arm()
    vehicle.simple_takeoff(targetHeight)#meters
    while True:
        print("Current Altitude: %f"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.91*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

def land():
    vehicle.mode=VehicleMode("LAND")
    while vehicle.mode != 'LAND':
        print("Waiting for drone to enter LAND mode")
        time.sleep(1)
    print("Vehicle in LAND mode")
    disarm()
    disconnect_to_vehicle()

def get_distance_meters(waypoint,currentLocation):
    dLat = waypoint.lat - currentLocation.lat
    dLon = waypoint.lon - currentLocation.lon
    dAlt = waypoint.alt - currentLocation.alt
    return math.sqrt((dLon*dLon)+(dLat*dLat)+(dAlt*dAlt))        
    
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
    vehicle.flush()
    print("Yawing") 
    time.sleep(6)
    print("Yaw complete")

def yaw_initializer():
    print("Initialising yaw")
    lat=vehicle.location.global_relative_frame.lat
    lon=vehicle.location.global_relative_frame.lon
    alt=vehicle.location.global_relative_frame.alt
    aLocation=LocationGlobalRelative(lat,lon,alt)
    msg=vehicle.message_factory.set_position_target_global_int_encode(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,0b0000111111111000,aLocation.lat*1e7,aLocation.lon*1e7,aLocation.alt,0,0,0,0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(2)

def send_local_ned_velocity(vx,vy,vz,duration):
    msg=vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    vehicle.flush()
    
def send_global_ned_velocity(vx,vy,vz):
    msg=vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_LOCAL_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
    vehicle.send_mavlink(msg)    
    vehicle.flush()

def move_front(v,t):
    print("Moving front")
    send_local_ned_velocity(v,0,0,t)
    
def move_back(v,t):
    print("Moving back")
    send_local_ned_velocity(-1*v,0,0,t)

def move_left(v,t):
    print("Moving left")
    send_local_ned_velocity(0,-1*v,0,t)
    
def move_right(v,t):
    print("Moving right")
    send_local_ned_velocity(0,v,0,t)
    
def stop(t):
    print("Stop")
    send_local_ned_velocity(0,0,0,t)

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
    return vx,vy,vz,theta

def radians_to_degree(theta_rad):
    if theta_rad > 0:
        theta_deg = theta_rad*180.0/math.pi
    else:
        theta_deg = 360 + theta_rad*180.0/math.pi
    return theta_deg

#Getting data from arduino node
#ASCII, B:66, F:70, L:76, R:82, S:83, X:88, b:98, f:102, l:108, r:114, e:69

def get_data(cmd_msg):
    global data
    data = cmd_msg.data
    time.sleep(0.0075)

rospy.init_node("RPi", anonymous=False)
rospy.Subscriber("cmd_topic", Int8, get_data)

def obstacle_avoidance():
    print("OBSTACLE DETECTED!")
    if data == 70:
        move_front(0.5,1)
    elif data == 66:
        move_back(0.5,1)
    elif data == 76:
        move_left(0.5,1)
    elif data == 82:
        move_right(0.5,1)
    elif data == 102:
        move_front(1,1)
        stop(1)
        get_data()
        if data == 88:
            move_left(0.5,1)
        else:
            stop(1)
    elif data == 108:
        move_left(1,1)
        stop(1)
        get_data()
        if data == 88:
            move_back(0.5,1)
        else:
            stop(1)
    elif data == 114:
        move_right(1,1)
        stop(1)
        get_data()
        if data == 88:
            move_front(0.5,1)
        else:
            stop(1)
    elif data == 98:
        move_back(1,1)
        stop(1)
        get_data()
        if data == 88:
            move_right(0.5,1)
        else:
            stop(1)
    elif data == 83:  
        stop(1)        

def safe_travel(waypoint,speed):
    print("Travelling to waypoint")
    distanceToTargetLocation = get_distance_meters(waypoint,vehicle.location.global_relative_frame)
    currentDistance = get_distance_meters(waypoint,vehicle.location.global_relative_frame)
    vx,vy,vz,theta = nav_velocity(waypoint,speed)
    print("Distance to wp : %f"%distanceToTargetLocation)
    theta_deg = radians_to_degree(theta)
    condition_yaw(theta_deg,0,1)
    while True:
        if data != 88:
            obstacle_avoidance()
        vx,vy,vz,theta = nav_velocity(waypoint,speed)
        send_global_ned_velocity(vx,vy,vz)
        currentDistance = get_distance_meters(waypoint,vehicle.location.global_relative_frame)
        if (currentDistance<0.01*distanceToTargetLocation):
            break
    print("******Reached******")
    send_local_ned_velocity(0,0,0,3)
   
wp1=LocationGlobalRelative(-35.3631549,149.1652004,0.3)
wp2=LocationGlobalRelative(-35.3629416,149.1651736,0.3)
wp3=LocationGlobalRelative(-35.3632183,149.1652367,0.3)
wp4=LocationGlobalRelative(-35.3632107,149.1653252,4)
takeoff(0.3)
yaw_initializer()
safe_travel(wp1,1)
safe_travel(wp2,1)
safe_travel(wp3,1)
#safe_travel(wp4,2)
land()
disconnect_to_vehicle()
