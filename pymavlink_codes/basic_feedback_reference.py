#basic nav py
import argparse
from pymavlink import mavutil
from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info
import time


def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):

    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    wait_until_position_aiding(mav_connection)
    
    autopilot_info = get_autopilot_info(mav_connection, tgt_sys_id)

    if autopilot_info["autopilot"] == "ardupilotmega":
        print("Connected to ArduPilot autopilot")
        mode_id = mav_connection.mode_mapping()["GUIDED"]
        takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

    elif autopilot_info["autopilot"] == "px4":
        print("Connected to PX4 autopilot")
        print(mav_connection.mode_mapping())
        mode_id = mav_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]

    else:
        raise ValueError("Autopilot not supported")


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

    print("Arming drone, commencing takeoff in 5 seconds")
    time.sleep(5)


    # Command Takeoff
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=100)
    print(f"Takeoff ACK:  {takeoff_msg}")

    tkoff_msg  = mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking = True)
    tkoff_live_alt = tkoff_msg.z * -1.0

    while tkoff_live_alt < 9:
        tkoff_msg  = mav_connection.recv_match(type='LOCAL_POSITION_NED', blocking = True)
        tkoff_live_alt = tkoff_msg.z * -1.0
        print("Takeoff in progress, current alt: ", tkoff_live_alt)



    print("Takeoff complete, Going to a guided waypoint now")

    time.sleep(5)

    # Go to this lat/long
    mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

    guided_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Guided ack:  {guided_msg}")


    # The main 'holding' implementation for the drone, NAV_CONTROLLER_OUPUT bit field 5 is accessed and compared in a while loop until less than 1 
    msg_3 = mav_connection.recv_match(
                       type='NAV_CONTROLLER_OUTPUT', blocking=True)
    wp = msg_3.wp_dist
    print("Dist to waypoint: ", wp)
  
   # Holding implementation start 
    while int(wp) > 1:
        msg_3 = mav_connection.recv_match(
                    type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp = msg_3.wp_dist
        print("Dist to waypoint: ", wp)


    print("Reached waypoint, waiting for 5 seconds before heading back to base")
    time.sleep(5)

    mode_id = mav_connection.mode_mapping()["RTL"]

    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id   , 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)

    print(f"Change Mode ACK:  {ack_msg}")
    print("Invoking return to base")

    time.sleep(2)

    # (This time for RTL) The main 'holding' implementation for the drone, NAV_CONTROLLER_OUPUT bit field 5 is accessed and compared in a while loop until less than 1 
    msg_3 = mav_connection.recv_match(
                           type='NAV_CONTROLLER_OUTPUT', blocking=True)
    wp = msg_3.wp_dist
    print("Dist to waypoint: ", wp)
      
    # Holding implementation start 
    while int(wp) > 1:
            msg_3 = mav_connection.recv_match(
                        type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp = msg_3.wp_dist
            print("Dist to waypoint: ", wp)

    # Same as seen earlier but on the basis of altitude to see if RTL has ben completed

    msg_4 = mav_connection.recv_match(type = 'LOCAL_POSITION_NED', blocking = True)
    altitude = msg_4.z * -1.0

    while altitude > 1:
        msg_4 = mav_connection.recv_match(type = 'LOCAL_POSITION_NED', blocking = True)
        altitude = msg_4.z * -1.0
        print("RTL Altitude: ", altitude)

    print("Any landing you can walk away from is a good landing, cheers lad!")


    return takeoff_msg.result

def main():
    parser = argparse.ArgumentParser(description="A simple script to command a UAV to takeoff.")
    parser.add_argument("--altitude", type=int, help="Altitude for the UAV to reach upon takeoff.", default=10)
    parser.add_argument("--sysid", type=int, help="System ID of the UAV to command.", default=1)


    args = parser.parse_args()
    mav_connection = connect_to_sysid('udp:localhost:14550', args.sysid)
    takeoff(mav_connection, args.altitude)

if __name__ == "__main__":
    main()
