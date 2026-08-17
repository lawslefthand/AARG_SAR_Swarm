import argparse
from html import parser
from pymavlink import mavutil
from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info
import time

# FOR SITL ONLY, Mav auto incremented Sys ID for Swarm of 3 drones
sysid_1 = 1
sysid_2 = 2
sysid_3 = 3

def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int, tgt_comp_id: int = 1):

    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    #wait_until_position_aiding(mav_connection) - investiagte why this is not working for swarm takeoff

    print("Connected to ArduPilot autopilot")
    mode_id = mav_connection.mode_mapping()["GUIDED"]
    takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    # Arm the UAS
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)                   
    print(f"Arm ACK:  {arm_msg}")

    print("Arming drone, commencing takeoff")

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

def main():

# Assuming sys_id 1, 2 and 3 for the auto incremented Mav IDs
    mav_connection_1 = connect_to_sysid('udp:localhost:14550', int(sysid_1))
    print("Connected to drone 1")
    takeoff(mav_connection_1, 20.0, int(sysid_1))

    mav_connection_2 = connect_to_sysid('udp:localhost:14551', int(sysid_2))
    print("Connected to drone 2")
    takeoff(mav_connection_2, 20.0, int(sysid_2))

    mav_connection_3 = connect_to_sysid('udp:localhost:14552', int(sysid_3))
    print("Connected to drone 3")
    takeoff(mav_connection_3, 20.0, int(sysid_3))

    

if __name__ == "__main__":
    main()
