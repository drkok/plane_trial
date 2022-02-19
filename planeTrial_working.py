#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 24 20:24:54 2021

@author: jiaming
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import dronekit as dk
import time
from time import sleep
from pymavlink import mavutil
import numpy as np
import math
from math import cos, sin 

import socket
import struct
import sys

import plane_serial_decode_encode as plane_encoder


HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 65433        # The port used by the server

#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.connect((HOST, PORT))

"""
To run, start up two instances of SITL
cd ardupilot/ArduPlane
../Tools/autotest/sim_vehicle.py --map --console -I 0 --out 127.0.0.1:14501
../Tools/autotest/sim_vehicle.py --map --console -I 1 --out 127.0.0.1:14511
../Tools/autotest/sim_vehicle.py --map --console -I 2 --out 127.0.0.1:14521

"""
# Connect to the first vehicle
vehicle = dk.connect("127.0.0.1:14501")
vehicle.mode = dk.VehicleMode("GUIDED")

# Connect to the second vehicle
vehicle2 = dk.connect("127.0.0.1:14511")
vehicle2.mode = dk.VehicleMode("GUIDED")

# Connect to the second vehicle
vehicle3 = dk.connect("127.0.0.1:14521")
vehicle3.mode = dk.VehicleMode("GUIDED")

agents = [vehicle, vehicle2, vehicle3]



#####################################################################
###############   Initialise Plane Locations   ######################
#####################################################################
"""
This section of code defines the locations that the plane agents will refer to
Home location is acquired from the plane (in SITL). In real world this will need
to be predefined
Destinations define where the plane will try and converge on 

Woomera is:
-30.934754851555784, 136.54496912108817

Fence is a rectangle.
North Western point is -30.91675046 136.52985274
South Eastern point is -30.93712316 136.56125030

"""
# This will define the home location and will serve as the origin
home = vehicle.location.global_relative_frame

# this defines the destinations for each plane
Destinations = []
TARGET_LAT, TARGET_LON = -30.93143610,136.54482253 #-35.36293982, 149.16069099
TARGET_ALT = 100
delta_alt = 10
AltitudeTargets = []
for i in range(np.size(agents)):
    AltitudeTargets.append(TARGET_ALT + delta_alt*i)
    destination = dk.LocationGlobalRelative(TARGET_LAT, TARGET_LON, AltitudeTargets[i])
    Destinations.append(destination)
    
# These are the altitude targets that each agent should maintain to deconflict
safeAltitudeTargets = AltitudeTargets

pos = dk.LocationGlobalRelative(-30.92594205, 136.54482077,100) # This location corresponds to ~1km north of 'X'#-35.35877180, 149.16347858, 100)
pos2 = dk.LocationGlobalRelative(-30.93149335, 136.55369084, 120) # ~1km east of 'X'
pos3 = dk.LocationGlobalRelative(-30.92676785, 136.53649502, 140) # ~1km NW of 'X'
loiter_locations = [pos, pos2, pos3]

if np.size(loiter_locations) != np.size(agents):
    print("Not enough loiter locations defined")
    exit()

# Define the max and min speeds
GND_SPEED_MAX = 28
GND_SPEED_MIN = 22
DEFAULT_GND_SPEED = GND_SPEED_MAX*0.5 + GND_SPEED_MIN*0.5

# Define Loiter params
DEFAULT_LOITER_RAD = 100
MAX_LOITER_RAD = 250
MIN_LOITER_RAD = 70


# Define a safety distance - when the drones are at this spacing, altitudes can
# be synced
safety_spacing = 0.8*2*MAX_LOITER_RAD*abs(cos(0.5*np.pi - np.pi/np.size(agents)))
print("the Safety spacing is", safety_spacing)




#####################################################################
###############   DroneKit Functions   ##############################
#####################################################################

def cart2pol(dEast, dNorth):
    theta = math.atan2(dNorth, dEast)       # returns in radians. 
    rho = np.sqrt(dEast**2 + dNorth**2)
    return theta, rho


def rotate_to_polar(state, theta):
    """
    This is a function that takes a state vector in Cartesian coordinates and finds the equivalent in 
    the rotated coordinate system
    State should be a 2x1 state vector [x,y]
    Theta is the rotation to the new coordinate system
    """
    dTheta = state[0]*cos(theta) + state[1]*sin(theta)
    dRho = -state[0]*sin(theta) + state[1]*cos(theta)
    
    return dTheta, dRho



def arm_and_takeoff(aTargetAltitude,vehicle = vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    # Plane should arm in AUTO or TAKEOFF
    vehicle.mode = VehicleMode("TAKEOFF")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    time.sleep(10)
    vehicle.mode = VehicleMode("GUIDED")
#    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
#
#    # Wait until the vehicle reaches a safe height before processing the goto
#    #  (otherwise the command after Vehicle.simple_takeoff will execute
#    #   immediately).
#    while True:
#        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
#        # Break and return from function just below target altitude.
#        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
#            print("Reached target altitude")
#            break
#        time.sleep(1)


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon,original_location.alt)


def get_distance_from_home(current_location,home_location):
    """
    Returns a distance in metres from the home location
    
    print(get_distance_from_home(vehicle.location.global_relative_frame,home))
    """
    earth_radius=6378137.0 #Radius of "spherical" earth

    # Difference in decimal degrees
    dLat = current_location.lat - home_location.lat
    dLon = current_location.lon - home_location.lon
    
    # Distance in metres
    dNorth = (dLat*math.pi/180) * earth_radius
    dEast = (dLon*math.pi/180) * (earth_radius*math.cos(math.pi*current_location.lat/180))
    return dNorth, dEast


def get_distance_from_agent(current_location,agent_location):
    """
    Returns a distance in metres from the home location
    
    print(get_distance_from_home(vehicle.location.global_relative_frame,home))
    """
    earth_radius=6378137.0 #Radius of "spherical" earth

    # Difference in decimal degrees
    dLat = current_location.lat - agent_location.lat
    dLon = current_location.lon - agent_location.lon
    
    # Distance in metres
    dNorth = (dLat*math.pi/180) * earth_radius
    dEast = (dLon*math.pi/180) * (earth_radius*math.cos(math.pi*current_location.lat/180))
    return dNorth, dEast


def get_distance_from_destination(current_location,destination_location):
    """
    Returns a distance (dNorth, dEast) in metres from the destination location
    
    print(get_distance_from_home(vehicle.location.global_relative_frame,home))
    """
    earth_radius=6378137.0 #Radius of "spherical" earth

    # Difference in decimal degrees
    dLat = current_location.lat - destination_location.lat
    dLon = current_location.lon - destination_location.lon
    
    # Distance in metres
    dNorth = (dLat*math.pi/180) * earth_radius
    dEast = (dLon*math.pi/180) * (earth_radius*math.cos(math.pi*current_location.lat/180))
    return dNorth, dEast

def get_polar_from_destination(current_location,destination_location):
    """
    Returns polar coordinates centred about the target location 
    """
    dNorth, dEast = get_distance_from_destination(current_location,destination_location)
    theta, rho = cart2pol(dEast, dNorth)
    return theta,rho


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
#    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    *Note that this does not work in arduplane
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b110111000111,#0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)

def send_serial_message(data, vehicle):
    """
    remotely control serial ports using this message
    """
#    s = ' '.join(data)
#    s = s.replace('\\r','\r')
#    s = s.replace('\\n','\n')
#    buf = [ord(x) for x in s]
    buf = data
    buf.extend([0]*(70-len(buf)))
    msg = vehicle.message_factory.serial_control_encode(
            3,# device (i.e. serial port); 3 corrsponds to GPS2 on mini carrier board
            0,#flags
            500,#timeout (ms)
            921600,#baudrate * note that this needs to be the same baudrate is RPi
            len(data),#count
            buf#data
            )
    
    vehicle.send_mavlink(msg)





#####################################################################
###############   Agent Policy         #############################
#####################################################################
"""
This set of scripts control the movement of the individual agents within the 
fixed wing swarm

The two variables it can control are 
- Speed (controlled through ground speed)
- Loiter Radius

Each agent should:
- Consider the location of the other agents
- If a certain distance threshold is met, react to it
"""

def agentControl(agents,destination,_id=0):
    
    agentPolicy = "speedOnly"
    
    # Check that the ID does not exceed the number of agents
    if _id > np.size(agents)-1:
        print("ID higher than the # of agents")
        return False

    selfTheta, selfRho = get_polar_from_destination(agents[_id].location.global_relative_frame, destination)

    dSpeed = 0 
    tmp_dNorth = 0 
    tmp_dEast = 0
    for i in range(np.size(agents)):
    # Calculate the distance of all agents to agent i
        if i != _id:
            dNorth, dEast = get_distance_from_agent(agents[_id].location.global_relative_frame, agents[i].location.global_relative_frame)
            tmpDistance = np.linalg.norm([dNorth,dEast])    # Measure the distance to the agent
            tmp_dNorth += dNorth
            tmp_dEast += dEast
            if tmpDistance < 500:       # if the spacing between agents is less than 500, do something
#                print("Distance to agent %d is %f" %(i, tmpDistance))
                agentTheta, agentRho = get_polar_from_destination(agents[i].location.global_relative_frame, destination)
                tmpAngle = (agentTheta-selfTheta)*180/np.pi
#                print("Theta difference from agent %d is %f" %(i, tmpAngle))
     
    dTheta, dRho = rotate_to_polar([tmp_dEast, tmp_dNorth],selfTheta-np.pi/2)       
    print("The speed difference requried is %f" %dTheta)
    return dTheta/5, dRho/10


def calc_agent_spacing(agents):
    """
    Calculates the spacing between all agents    
    Returns [d_1_2, d_1_3, d_1_4, d_2_3, d_2_4, D_3_4] for 4 drones as example
    """    
    spacing = []
    for i in range(np.size(agents)-1):
        for j in range(np.size(agents[i+1:])):
            tmpJ = j+i+1
            dNorth, dEast = get_distance_from_agent(agents[i].location.global_relative_frame, agents[tmpJ].location.global_relative_frame)
            tmpDistance = np.linalg.norm([dNorth,dEast])
            tmpDistance = np.sqrt(dNorth**2+ dEast**2)
            spacing.append(tmpDistance)
    print('Lateral spacing is:', spacing)
    return spacing

def calc_agent_z_spacing(agents):
    """
    Calculates the spacing between all agents in the z direction    
    Returns [d_1_2, d_1_3, d_1_4, d_2_3, d_2_4, D_3_4] for 4 drones as example
    """    
    z_spacing = []
    for i in range(np.size(agents)-1):
        for j in range(np.size(agents[i+1:])):
            tmpJ = j+i+1
            dZ =  agents[i].location.global_frame.alt - agents[tmpJ].location.global_frame.alt
            z_spacing.append(dZ)
    print("Z Spacing is:", z_spacing)
    return z_spacing

def calc_agent_theta(agents, destination):
    """
    This calculates the theta between all agents
    """
    
    for i in range(np.size(agents)):
        agentTheta, agentRho = get_polar_from_destination(agents[i].location.global_relative_frame, destination)
        
    #INCOMPLETE!!!!!!!!

def converge_safety_check(agentSpacing,safety_spacing):
    """
    MEasure the deviation from the mean spacing.
    If the highest deviation is less than threshold return True, else False
    """
    mean_spacing = np.mean(agentSpacing)
    deviation = agentSpacing-mean_spacing
    if np.max(deviation) < 0.1*mean_spacing and np.min(agentSpacing)>safety_spacing:
        return True
    else:
        return False
    
def diverge_safety_check(agentZSpacing,alt_safety_spacing):
    """
    Provides a criteria to decide if it is safe to enter a diverge mode
    """
    if np.min([abs(dz) for dz in agentZSpacing]) >  0.8*alt_safety_spacing:
        return True
    else:
        return False



#####################################################################
###############   Mode Change Thread       ##########################
#####################################################################
"""
This section of code sets up two hotkey <alt><h> and <alt><i> which switches between
diverging and converging modes respectively
"""
from pynput import keyboard

mode = 'CONVERGE'
converge_true = True

def on_activate_c():
    global mode
    mode = 'CONVERGE'
    print("Aircraft converging !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

def on_activate_d():
    global mode
    mode = 'DIVERGE'
    print("Aircraft diverging !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

listener = keyboard.GlobalHotKeys({
        '<alt>+c': on_activate_c,
        '<alt>+d': on_activate_d})

listener.start()


#####################################################################
###############   Execute Mission       #############################
#####################################################################
"""
Define and execute the mission in this section of code
Note that the arm_and_takeoff will only happen in SITL
In the real world, the planes will have a predefined loiter location
Takeoff will be done manually by pilot and switched to AUTO mode to go to loiter
waypoint
"""

# Takeoff all drones
# Drone1 will fly to here and loiter about here
arm_and_takeoff(100,vehicle)
vehicle.simple_goto(pos, groundspeed=20)

# Drone2 will fly to here and loiter about here
arm_and_takeoff(120,vehicle2)
vehicle2.simple_goto(pos2, groundspeed=20)

# Drone2 will fly to here and loiter about here
arm_and_takeoff(140,vehicle3)
vehicle3.simple_goto(pos3, groundspeed=20)


sleep(20) # give time to get these locations



radiusCentre = pos
i=0

# Go to the destination point
raw_input("Press Enter to continue...")
vehicle.simple_goto(destination, groundspeed=DEFAULT_GND_SPEED)
vehicle2.simple_goto(destination, groundspeed=DEFAULT_GND_SPEED)
vehicle3.simple_goto(destination, groundspeed=DEFAULT_GND_SPEED)

# Change the radius of the aircraft
vehicle.parameters['WP_LOITER_RAD']=DEFAULT_LOITER_RAD
vehicle2.parameters['WP_LOITER_RAD']=DEFAULT_LOITER_RAD
vehicle3.parameters['WP_LOITER_RAD']=DEFAULT_LOITER_RAD



#values = (1.2, 2.3, 3.4, 4.5, 5.6, 6.7 )
#packer = struct.Struct('f f f f f f')
#packed_data = packer.pack( * values)





while True:
    print("Vehicle ground speed is:", vehicle.groundspeed, vehicle2.groundspeed, vehicle3.groundspeed)
    print("Vehicle Altitude is:", vehicle.location.global_frame.alt, vehicle2.location.global_frame.alt, vehicle3.location.global_frame.alt)
    
    if mode == 'CONVERGE':
        converge_true = True
    elif mode == 'DIVERGE':
        converge_true = False
    
    # Capture the state information and forward to the agents
    # Note that in the encode/decode function, first 4 bits are header bits
    state_header=[np.size(agents), int(converge_true)]
    state_body = []
    for i in range(np.size(agents)):
        state_body.append(agents[i].location.global_relative_frame.lat)
        state_body.append(agents[i].location.global_relative_frame.lon)
        state_body.append(agents[i].location.global_frame.alt)
        
    state = plane_encoder.state_encode(state_header, state_body)
    for i in range(np.size(agents)):
        send_serial_message(state,agents[i])
        
    plane_encoder.state_decode(state)
    
    
    agent_spacing = calc_agent_spacing(agents)
    agent_z_spacing = calc_agent_z_spacing(agents)
    safe_to_converge = converge_safety_check(agent_spacing, safety_spacing)
    safe_to_diverge  = diverge_safety_check(agent_z_spacing, delta_alt)

    print("Safe to converge:", safe_to_converge)
    print("Safe to diverge:", safe_to_diverge)

    if mode == 'CONVERGE' or not safe_to_diverge:

        if mode == 'CONVERGE' and safe_to_converge:
            AltitudeTargets = [TARGET_ALT]*np.size(agents)
        else:
            AltitudeTargets = safeAltitudeTargets
    
    
        for i in range(np.size(agents)):
            dV,dr = agentControl(agents,destination,i)
            targetGndSpeed = DEFAULT_GND_SPEED+dV
            if targetGndSpeed > GND_SPEED_MAX:
                targetGndSpeed = GND_SPEED_MAX
            elif targetGndSpeed < GND_SPEED_MIN:
                targetGndSpeed = GND_SPEED_MIN
                
            r_Cmd = DEFAULT_LOITER_RAD + dr
            if r_Cmd > MAX_LOITER_RAD:
                r_Cmd = MAX_LOITER_RAD
            elif r_Cmd < MIN_LOITER_RAD:
                r_Cmd = MIN_LOITER_RAD
            agents[i].parameters['WP_LOITER_RAD']=r_Cmd
    
    
            destination = dk.LocationGlobalRelative(TARGET_LAT, TARGET_LON, AltitudeTargets[i])
            agents[i].simple_goto(destination, groundspeed=targetGndSpeed)

    elif mode == "DIVERGE" and safe_to_diverge:
        print("DIVERGING!!!")
        for i in range(np.size(agents)):
            agents[i].simple_goto(loiter_locations[i], groundspeed=DEFAULT_GND_SPEED)
        
        
        
        
#    dV,dr = agentControl(agents,destination,1)
#    targetGndSpeed = DEFAULT_GND_SPEED+dV
#    if targetGndSpeed > GND_SPEED_MAX:
#        targetGndSpeed = GND_SPEED_MAX
#    elif targetGndSpeed < GND_SPEED_MIN:
#        targetGndSpeed = GND_SPEED_MIN
#    vehicle2.simple_goto(Destinations[1], groundspeed=targetGndSpeed)
#    r_Cmd = 100 + dr
#    if r_Cmd > 150:
#        r_Cmd = 150
#    elif r_Cmd < 70:
#        r_Cmd = 70
#    vehicle2.parameters['WP_LOITER_RAD']=r_Cmd
#
#
#    dV,dr = agentControl(agents,destination,2)
#    targetGndSpeed = DEFAULT_GND_SPEED+dV
#    if targetGndSpeed > GND_SPEED_MAX:
#        targetGndSpeed = GND_SPEED_MAX
#    elif targetGndSpeed < GND_SPEED_MIN:
#        targetGndSpeed = GND_SPEED_MIN
#    vehicle3.simple_goto(Destinations[2], groundspeed=targetGndSpeed)
#    r_Cmd = L + dr
#    if r_Cmd > 150:
#        r_Cmd = 150
#    elif r_Cmd < 70:
#        r_Cmd = 70
#    vehicle3.parameters['WP_LOITER_RAD']=r_Cmd    
    
    
    x1,y1 = get_distance_from_destination(agents[0].location.global_relative_frame,destination)
    x2,y2 = get_distance_from_destination(agents[1].location.global_relative_frame,destination)
    x3,y3 = get_distance_from_destination(agents[2].location.global_relative_frame,destination)
    
#    values=(x1, y1, x2, y2, x3, y3)
#    packed_data = packer.pack( * values)
#    s.sendall(packed_data)
    
    sleep(1)

    
    
    
    
    
    
    
    
    
    
    
