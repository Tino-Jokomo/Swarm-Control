from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import time
import socket
import paho.mqtt.client as mqtt
import json

from pymavlink import mavutil


#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
vehicle = connect('tcp:0.0.0.0:5780', wait_ready=True)
'''vehicle2 = connect('tcp:0.0.0.0:5783', wait_ready=True)'''

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

    earth_radius=6378137.0 #Radius of "spherical" #earth           
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
        
    """Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
  :"""
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    home= vehicle.home_location
#home1.lat=home.lat+0.00005;
#home1.lon=home.lon+0.00005;
#home1.alt=home.alt

def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 

    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    point5 = get_location_metres(aLocation, aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point5.lat, point5.lon, 15))
    #add dummy waypoint "6" at point 5 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 15))    

    print(" Upload new commands to vehicle")
    cmds.upload()
    
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
follow_gps={}
leader_gps={}
broker="154.114.37.235"
port=1883
def on_disconnect(client, userdata, rc):
    #logging.info("disconnecting reason  "  +str(rc))
    print("Disconnecting..")
    client.connected_flag=False
    client.disconnect_flag=True
    vehicle.mode = VehicleMode("AUTO")
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    for variable in ["late", "longe", "alte"]:
        leader_gps[variable] = eval(variable)
        print('2nd Leader GPS POS=',vehicle.location.global_frame)
    time.sleep(5)   
    data_out=json.dumps(leader_gps) 
    client.on_publish = on_publish  
    client.connect(broker) 
    client.publish("test/1",data_out)  
              
def on_publish(client,userdata,result):             #create function for callback
    print("data published \n")
    pass


def on_message(client,userdata,msg): 
    topic=msg.topic
    m_decode= str(msg.payload.decode("utf-8","ignore"))     
    m_in=json.loads(m_decode)
    print("Leader GPS:", m_in)
    follow_gps["late"]=m_in["late"]
    follow_gps["longe"]=m_in["longe"]
    follow_gps["alte"]=m_in["alte"]
    follow_gps["timestamp"]=m_in["timestamp"]
    dest = LocationGlobal(m_in["late"]+0.0005, m_in["longe"], m_in["alte"])
    global timestamp 
    timestamp=time.time()
     
    print ("Going to: %s" , dest)
    vehicle.simple_goto(dest)
    time.sleep(2)


topic="test/follow-me"



fol_id="2" 
time.sleep(5)
#MISSION START
client =mqtt.Client(fol_id) 
client.connect(broker)
client.subscribe("test/1")


# print("Create a new mission (for current location)")

# adds_square_mission(home1,100)


# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)
vehicle.groundspeed = 25
vehicle.airspeed = 25
print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("GUIDED")
# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

 
#    while True:
 #   nextwaypoint=vehicle2.commands.next
  #  print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
  #
   # if nextwaypoint==4: #Skip to next waypoint
    #    print('Skipping to Waypoint 5 when reach waypoint 4')
     #   vehicle2.commands.next = 5
    #if# nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
       # print("Exit 'standard' mission when start heading to final waypoint (5)")
        #break;
    #time.sleep(1)


#print('Return to launch')
#vehicle2.mode = VehicleMode("RTL")

'''clients=[]
nclients=3
mqtt.Client.connected_flag=False
'''
timestamp=time.time() 
adds_square_mission(vehicle.location.global_frame,100)
while True:
        #create clients
    client.on_message=on_message
    client.on_publish = on_publish 
    client.connect(broker)
    client.loop_start()
    client.subscribe("test/1")
    #client.subscribe("test/3") 
    vehicle.commands.next=0 
    cur_time=time.time()
    
    if ((cur_time-timestamp)>45):
        print("Disconnected from original leader ")
        print("I am the captain now")
        client.unsubscribe("test/1")
            
        print("Adding  Recovery Mission")
        vehicle.mode = VehicleMode("AUTO")
        while True:
            late= vehicle.location.global_relative_frame.lat
            longe=vehicle.location.global_relative_frame.lon

            alte=vehicle.location.global_relative_frame.alt
             
            
            
            nextwaypoint=vehicle.commands.next
            print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
            for variable in ["late", "longe", "alte","timestamp"]:
                leader_gps[variable] = eval(variable)
                print('2nd Leader GPS POS=',vehicle.location.global_relative_frame)

            data_out=json.dumps(leader_gps)    
            time.sleep(4)   
             
            client.on_publish = on_publish  
            client.connect(broker) 
            client.publish("test/1",data_out)
            break
     
     


    
     
    #data_in=str(msg.payload.decode("utf-8","ignore"))
     
    #add publish if statement on disconnect here
    #later=follow_gps["late"]
    #longer=follow_gps["longe"]
    #alter=follow_gps["alte"]

    time.sleep(5)
    client.loop_stop()

# A better implementation would only send new waypoints if the position had changed significantly








#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()


""" 
try:
    # Use the python gps package to access the laptop GPS
    #gpsd = gps.gps(mode=gps.WATCH_ENABLE)
    gps_lead = vehicle.gps.gps(mode=gps.WATCH_ENABLE)
   # vehicle.gps_0
    #Arm and take off to an altitude of 5 meters
    arm_and_takeoff(5)

while True:

        if vehicle.mode.name != "GUIDED":
            print "User has changed flight modes - aborting follow-me"
            break

        # Read the GPS state from the laptop
       # gpsd.next()
#READ GPS FROM LEADER DRONE
#set up how to decide leader i.e if leader fails use sys_id to replace next drone closest to way point to be leader then 
        gps_lead=vehicle.location.global_frame

        # Once we have a valid location (see gpsd documentation) we can start moving our vehicle around
        if (gps_lead.valid & gps.LATLON_SET) != 0:
            altitude = 30  # in meters
            dest = LocationGlobalRelative(gps_lead.fix.latitude, gps_lead.fix.longitude, altitude)
            print "Going to: %s" % dest

            # A better implementation would only send new waypoints if the position had changed significantly
            vehicle2.simple_goto(dest)

            # Send a new target every two seconds
            # For a complete implementation of follow me you'd want adjust this delay
            time.sleep(2)
            """