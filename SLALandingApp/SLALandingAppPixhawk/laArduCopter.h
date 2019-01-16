/*
 * Copyright (C)2008-2013 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 *------------------------------------------------------------------------*/
#ifndef  _LA_APM_H_
#define  _LA_APM_H_


enum {
    COMMSTATE_INIT,        // Waiting for first message
    COMMSTATE_SN,          // Wating for serial number
    COMMSTATE_INIT_POS,    // Waiting for inital position
    COMMSTATE_WRITE_INIT,  // Write initial position hold
    COMMSTATE_READ_WRITE   // Normal read/write operation
};


typedef struct
{
	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;

} Time_Stamps;

void reset_timestamps(Time_Stamps* timeStamps)
{
	timeStamps->heartbeat = 0;
	timeStamps->sys_status = 0;
	timeStamps->battery_status = 0;
	timeStamps->radio_status = 0;
	timeStamps->local_position_ned = 0;
	timeStamps->global_position_int = 0;
	timeStamps->position_target_local_ned = 0;
	timeStamps->position_target_global_int = 0;
	timeStamps->highres_imu = 0;
	timeStamps->attitude = 0;
}

// Struct containing information on the MAV we are currently connected to
typedef struct
{

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

  // Time Stamps
	Time_Stamps time_stamps;

} Mavlink_Messages;

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

                                                // bit number          876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0xDF8 //0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0xFC7 //0b0000111111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0xC3F //0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0xE3F //0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0x9FF //0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0x5FF //0b0000010111111111

#endif