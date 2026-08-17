import argparse
from html import parser
from pymavlink import mavutil
from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info
import time
import threading

'''
Use these for launching SITL, In three different terminal windows of Ubuntu ( 3 different instances)
Replace with your device specific IPs

sim_vehicle.py -v Copter -I0 --console --map -w \
-l -35.363243768,149.165231300,584,0 \
--out=172.21.64.1:14550 \
--out=127.0.0.1:14550 \
--sysid=1

sim_vehicle.py -v Copter -I1 --console --map -w \
-l -35.363282666,149.165258839,584,120 \
--out=172.21.64.1:14551 \
--out=127.0.0.1:14551 \
--sysid=2

sim_vehicle.py -v Copter -I2 --console --map -w \
-l -35.363282666,149.165203761,584,240 \
--out=172.21.64.1:14552 \
--out=127.0.0.1:14552 \
--sysid=3

'''
# FOR SITL ONLY, Mav auto incremented Sys ID for Swarm of 3 drones
sysid_1 = 1
sysid_2 = 2
sysid_3 = 3


# Feedback based Takeoff Function

def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int, tgt_comp_id: int = 1):

    print("Starting takeoff")

    # Wait_until_position_aiding(mav_connection) - investiagte why this is not working for swarm takeoff

    mode_id = mav_connection.mode_mapping()["GUIDED"]
    takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1000)
    print(f"Change Mode ACK:  {ack_msg}")

    # Arm the drone (duh)
    arm(mav_connection, int(tgt_sys_id))

    # Command Takeoff
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=100)
    print(f"Takeoff ACK:  {takeoff_msg}")

    tkoff_msg  = mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking = True)
    tkoff_live_alt = tkoff_msg.z * -1.0

    while tkoff_live_alt < (int(takeoff_altitude) - int(1)):
        tkoff_msg  = mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking = True)
        tkoff_live_alt = tkoff_msg.z * -1.0
        print("Takeoff in progress, current alt: ", tkoff_live_alt)

    print("Takeoff complete")

    return 1

# Feedback and non feedback based RTL functions 

def return_to_home_nofb(mav_connection,tgt_sys_id: int, tgt_comp_id: int = 1 ):

    mode_id = mav_connection.mode_mapping()["RTL"]

    # Change mode to RTL
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")


# Basic ARM and DISARM functions on the basis of Mav connection object ( IP + UDP) & Mav sys ID

def arm(mav_connection, tgt_sys_id: int, tgt_comp_id: int = 1):

    print("Arming drone")

    # Arm the UAS (Second bitfield controls arming/disarming, 0 for disarming and 1 for arming)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)                   
    print(f"Arm ACK: {arm_msg}")

def disarm(mav_connection, tgt_sys_id: int, tgt_comp_id: int = 1):

    print("Disrming drone")
                      
    # Disarm the UAS (Second bitfield controls arming/disarming, 0 for disarming and 1 for arming)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)                   
    print(f"Disrm ACK: {arm_msg}")



# Guided "go-to-function", which basically tells a MAV drone to go to a particular lat, long and at a particular requested altitude too
# Has a non-feedback based and a feedback based version
def go_to_coord_no_fb(mav_connection,lat, long, alt):

    # Go to this lat/long (first waypoint before going to the target)
    mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, mav_connection.target_system,
                               mav_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(lat * 10 ** 7), int(long * 10 ** 7), alt, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
       
    guided_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0)
    print(f"Guided ack:  {guided_msg}")


def go_to_coord_fb(mav_connection,lat, long, alt):

    # Go to this lat/long (first waypoint before going to the target)
    mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, mav_connection.target_system,
                               mav_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(lat * 10 ** 7), int(long * 10 ** 7), alt, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
       
    guided_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0)
    print(f"Guided ack:  {guided_msg}")

     # The main 'holding' implementation for the drone, NAV_CONTROLLER_OUPUT bit field 5 is accessed and compared in a while loop until less than 1 
    msg = mav_connection.recv_match(
                              type='NAV_CONTROLLER_OUTPUT', blocking=True)
    wp = msg.wp_dist
    print("Dist to waypoint: ", wp)
         
        # Holding implementation start 
    while int(wp) > 1:
        msg = mav_connection.recv_match(
                           type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp = msg.wp_dist
        print("Dist to waypoint: ", wp)

#Threading test functions 

def drone_1_instructions():

    mav_connection_1 = connect_to_sysid('udp:localhost:14550', int(sysid_1))
    print("Connected to drone 1")
    takeoff(mav_connection_1, 20.0, int(sysid_1))
    go_to_coord_fb(mav_connection_1,-35.3630772,149.1646466,20)
    go_to_coord_fb(mav_connection_1,-35.3625304,149.1646573,20)

    time.sleep(3)
    return_to_home_nofb(mav_connection_1, int(sysid_1))

def drone_2_instructions():

    mav_connection_2 = connect_to_sysid('udp:localhost:14551', int(sysid_2))
    print("Connected to drone 2")
    takeoff(mav_connection_2, 20.0, int(sysid_2))
    go_to_coord_fb(mav_connection_2,-35.3627754,149.1651294,20)
    go_to_coord_fb(mav_connection_2,-35.3621673,149.1650301,20)

    time.sleep(3)
    return_to_home_nofb(mav_connection_2, int(sysid_2))


def drone_3_instructions():

    mav_connection_3 = connect_to_sysid('udp:localhost:14552', int(sysid_3))
    print("Connected to drone 3")
    takeoff(mav_connection_3, 20.0, int(sysid_3))
    go_to_coord_fb(mav_connection_3,-35.3629482,149.1656765,20)
    go_to_coord_fb(mav_connection_3,-35.3624145,149.1655183,20)
    time.sleep(3)
    return_to_home_nofb(mav_connection_3, int(sysid_3))


 
def main():

# Assuming sys_id 1, 2 and 3 for the auto incremented Mav IDs

    print("Starting")

    time.sleep(5)

    t1 = threading.Thread(target=drone_1_instructions)
    t2 = threading.Thread(target=drone_2_instructions)
    t3 = threading.Thread(target=drone_3_instructions)

    t1.start()
    t2.start()
    t3.start()

    t1.join()
    t2.join()
    t3.join()

    print("Eveerything done and dusted")

if __name__ == "__main__":
    main()
