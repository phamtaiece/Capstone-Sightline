/*
 * Copyright (C)2007-2016 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 * Methods to communicate with the Pixhawk (APM) autopilot.
 *------------------------------------------------------------------------*/

#ifndef WIN32
  #include <sys/time.h>
#endif
#include "slcommon.h"
#include "slos.h"         // for getTime
#include "slrs232.h"
#include "mavlink.h"
#include "laAutopilot.h"
#include "laPX4.h"

static u64 APTelemetryTimeStamp = 0;          // timestamp for autopilot telemetry data
static const f32 MAX_LAT_STEERING_CMD = 0.5f; // max lateral or longitudinal velocity command used during landing. Range=[0:1]. Scaled by autopilot limits.
//static SLRs232 autopilotPort;                 // serial port to the Pixhawk
//static APMData_t apData;          // data passed between threads
static int system_id = 0;         // sytem id
static int autopilot_id = 0;      // autopilot component id
static int companion_id = 0;      // companion computer component id
static char reading_status;
static char writing_status;
//static char control_status;
bool time_to_exit = false;
static Mavlink_Messages current_messages;
static mavlink_status_t lastStatus;
static mavlink_set_position_target_local_ned_t initial_position;
static mavlink_set_position_target_local_ned_t current_setpoint;

// Local functions
static int AutopilotReadThread(void *context);
static int AutopilotWriteThread(void *context);
static void read_messages(GcContext *ctxt);
int read_message(mavlink_message_t &message, GcContext *ctxt);
static int write_message(mavlink_message_t &message, GcContext *ctxt);
static void write_setpoint(GcContext *ctxt);
static int toggle_offboard_control(bool flag, GcContext *ctxt);
static uint64_t get_time_usec();
static void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
static int CommState = 0;           //Communication state machine used in RunAutopilotComms

SLTaskPriority priorityCommand=SL_PRI_8, priorityRespond=SL_PRI_6, priorityTx=SL_PRI_7;

// Un-used messages
#define SERVO_OUTPUT_RAW 36
#define RC_CHANNELS 65
#define RC_CHANNELS_OVERRIDE 70

int RunAutopilotComms(GcContext* ctxt)
{

  switch(CommState)
  {
  case COMMSTATE_INIT:
    SLTrace("\n\nCOMMSTATE_INIT\n");

	//result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
    //SLCreateThread(AutopilotReadThread, SL_DEFAULT_STACK_SIZE, "AutopilotReadTask", &ctxt, priorityTx);

    read_messages(ctxt);

    #ifdef WIN32
      Sleep(500);
    #else
      usleep(500000); // check at 2Hz
    #endif

    if(current_messages.sysid)
    {
      SLTrace("Found a message. Sysid=%i\n", current_messages.sysid);

      CommState = COMMSTATE_SN;
	}

    break;

  case COMMSTATE_SN:
    SLTrace("COMMSTATE_SN\n");

	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.
    read_messages(ctxt);

	// System ID
	if (!system_id)
	{
		system_id = current_messages.sysid;
		SLTrace("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if (!autopilot_id)
	{
		autopilot_id = current_messages.compid;
		SLTrace("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		SLTrace("\n");
	}

    // Switch state when we got the sysId and Component Id
    if(system_id && autopilot_id)
    {
      SLTrace("COMMSTATE_INIT_POS\n");
      CommState = COMMSTATE_INIT_POS;
    }

    #ifdef WIN32
      Sleep(500);
    #else
      usleep(500000); // check at 2Hz
    #endif

    break;


  case COMMSTATE_INIT_POS:

    read_messages(ctxt);

	  // Wait for initial position ned
	  if ( current_messages.time_stamps.local_position_ned &&
				    current_messages.time_stamps.attitude )
	  {
	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	SLTrace("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	SLTrace("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	SLTrace("\n");

      //Switch to next state
      SLTrace("COMMSTATE_WRITE_INIT\n");
      CommState = COMMSTATE_WRITE_INIT;
	  }

    #ifdef WIN32
      Sleep(500);
    #else
      usleep(500000); // check at 2Hz
    #endif

    break;

  case COMMSTATE_WRITE_INIT:

    //READ
    read_messages(ctxt);

    // WRITE
	  //result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
    //SLCreateThread(AutopilotWriteThread, SL_DEFAULT_STACK_SIZE, "AutopilotWriteTask", &ctxt, priorityTx);

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	write_setpoint(ctxt);

    #ifdef WIN32
      Sleep(250);
    #else
	    usleep(250000);
    #endif

    SLTrace("COMMSTATE_READ_WRITE\n");
    CommState = COMMSTATE_READ_WRITE;

    break;

  case COMMSTATE_READ_WRITE:
    read_messages(ctxt);

    write_setpoint(ctxt);

    // Pixhawk needs to see off-board commands at minimum 2Hz,
	  // otherwise it will go into fail safe
    //#ifdef WIN32
    //  Sleep(10);
    //#else
	   // usleep(10000);   // run at 100Hz
    //#endif

    break;

  } //switch CommState

	return SLA_SUCCESS;
}// RunAutopilotComms


//static int AutopilotReadThread(void *context)
//{
//  reading_status = true;
//  char cp = 5;
//  int result;
//  GcContext* ctxt = (GcContext*)context;
//
//  SLTrace("Autopilot read thread started\n");
//
//	while ( !time_to_exit )
//	{
//		read_messages((GcContext*)context);
//    #ifdef WIN32
//      Sleep(100);
//    #else
//		  usleep(100000); // Read batches at 10Hz
//    #endif
//	}
//
//	reading_status = false;
//  return true;
//}

//static int AutopilotWriteThread(void *context)
//{
//  GcContext* ctxt = (GcContext*)context;
//	writing_status = 2;                     // signal startup
//
//  SLTrace("AutopilotWriteThread 1\n");
//
//	// prepare an initial setpoint, just stay put
//	mavlink_set_position_target_local_ned_t sp;
//	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
//				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
//	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
//	sp.vx       = 0.0;
//	sp.vy       = 0.0;
//	sp.vz       = 0.0;
//	sp.yaw_rate = 0.0;
//
//	// set position target
//	current_setpoint = sp;
//
//  SLTrace("AutopilotWriteThread 2\n");
//
//  SLTrace("test %i\n", ctxt->landingAidState.state);
//
//	// write a message and signal writing
//	write_setpoint(ctxt);
//	writing_status = true;
//
//  SLTrace("AutopilotWriteThread 3\n");
//
//	// Pixhawk needs to see off-board commands at minimum 2Hz,
//	// otherwise it will go into fail safe
//	while ( !time_to_exit )
//	{
//    #ifdef WIN32
//      Sleep(250);
//    #else
//		  usleep(250000);   // Stream at 4Hz
//    #endif
//		write_setpoint((GcContext*)context);
//	}
//
//	// signal end
//	writing_status = false;
//
//	return 1;
//}

// Determine if the autpilot is in the correct state to allow the SLA
// to start sending commands to the autopilot and transition to enable-2 state
SLStatus IsAutopilotReadyForEnable2()
{
	return SLA_SUCCESS;
}

// Send commands to the autopilot to prepare for landing
SLStatus SetAutopilotModeForEnable2(GcContext *ctxt)
{
	return SLA_SUCCESS;
}

// Check that the autopilot is in the correct mode to proceed with the landing
SLStatus IsAutopilotReadyForProsecute()
{
	return SLA_SUCCESS;
}

// Determine if the autpilot is in the correct state to allow a vision-based landing
SLStatus IsAutopilotReadyForLanding()
{
  //SLTrace("system_status: %i\n", current_messages.heartbeat.system_status);
  // The Pixhawk uses a switch on the RC transmitter to switch to "Guided" mode
//  if(current_messages.heartbeat.system_status == MAV_STATE_ACTIVE)  XXXX commented out for now.  Status is not read often enough?
    return SLA_SUCCESS;
//  else
//    return SLA_FAIL;
}

// Check that the autopilot is still in the correct mode to continue the landing
SLStatus IsAPInLandingMode()
{
  //if(current_messages.heartbeat.system_status == MAV_STATE_ACTIVE)
    return SLA_SUCCESS;
  //else
  //  return SLA_FAIL;
}

// Send commands to the autopilot to prepare for landing
SLStatus SetAutopilotModeForLanding(GcContext *ctxt)
{
  // Don't do anything.  RC transmitter should have a switch to enable offboard mode control.
  //toggle_offboard_control(true, ctxt);

  return SLA_SUCCESS;
}


SLStatus SendSteeringCommandsToAP(f32 Xcmd, f32 Ycmd, f32 ClimbRateFraction)
{

  // initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = initial_position;

  // Limit commands
  if(Xcmd < -MAX_LAT_STEERING_CMD)
    Xcmd = -MAX_LAT_STEERING_CMD;
  else if(Xcmd > MAX_LAT_STEERING_CMD)
    Xcmd = MAX_LAT_STEERING_CMD;

  if(Ycmd < -MAX_LAT_STEERING_CMD)
    Ycmd = -MAX_LAT_STEERING_CMD;
  else if(Ycmd > MAX_LAT_STEERING_CMD)
    Ycmd = MAX_LAT_STEERING_CMD;

	// Set Velocity
	set_velocity( Xcmd,               //x [m/s]
				        Ycmd,               //y [m/s]
				        -ClimbRateFraction,  //z [m/s]
				        sp );

	// SEND THE COMMAND
	//api.update_setpoint(sp);
  current_setpoint = sp;
  
  return SLA_SUCCESS;
}

// Get current vehicle altitude
SLStatus GetAutopilotData(AutopilotData* autopilotData)
{
  autopilotData->rollAngle = current_messages.attitude.roll;
  autopilotData->pitchAngle = current_messages.attitude.pitch;
  autopilotData->rollRate = current_messages.attitude.rollspeed;
  autopilotData->pitchRate = current_messages.attitude.pitchspeed;
  autopilotData->altitude = current_messages.global_position_int.alt;

  autopilotData->timestamp = APTelemetryTimeStamp;

  return SLA_SUCCESS;
}

void IndicateLandingAidTrackingStatus(u8 Status1, u8 Status2)
{
  // Turn on the lights and brakes.  These may be connected to a DIO on the vehicle
  // PCC will also display status via the "lights" and "brakes" buttons.
  
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int toggle_offboard_control(bool flag, GcContext *ctxt)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
  int len = write_message(message, ctxt);

	// Done!
	return len;
}

static void read_messages(GcContext *ctxt)
{
	int success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;
  static bool servoOutputRawNotHandled  = false;
  static bool rcChannelsNotHandled  = false;
  static bool rcChannelsOverrideNotHandled  = false;


  //SLTrace("read_messages started\n"); works

	// Blocking wait for new data
	while ( !received_all && !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = read_message(message, ctxt);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
      //SLTrace("read_messages success reading a message\n"); //works

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//SLTrace("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//SLTrace("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//SLTrace("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//SLTrace("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//SLTrace("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//SLTrace("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//SLTrace("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//SLTrace("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//SLTrace("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//SLTrace("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

        case SERVO_OUTPUT_RAW:
          if(!servoOutputRawNotHandled)
          {
            SLTrace("Warning, SERVO_OUTPUT_RAW Not handled.  Suppressing further warnings.\n");
            servoOutputRawNotHandled = true;
          }
          break;

        case RC_CHANNELS:
          if(!rcChannelsNotHandled)
          {
            SLTrace("Warning, RC_CHANNELS Not handled.  Suppressing further warnings.\n");
            rcChannelsNotHandled = true;
          }
          break;

        case RC_CHANNELS_OVERRIDE:
          if(!rcChannelsOverrideNotHandled)
          {
            SLTrace("Warning, RC_CHANNELS_OVERRIDE Not handled.  Suppressing further warnings.\n");
            rcChannelsOverrideNotHandled = true;
          }
          break;

				default:
				{
					SLTrace("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

    } // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
				this_timestamps.sys_status                 &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
				this_timestamps.position_target_global_int &&
				this_timestamps.highres_imu                &&
				this_timestamps.attitude                   ;

		// give the write thread time to use the port
		if ( writing_status > 0 )
      #ifdef WIN32
        Sleep(500);
      #else
			  usleep(100); // look for components of batches at 10kHz
      #endif

	} // end: while not received all

	return;
}

void write_setpoint(GcContext *ctxt)
{

	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

  //SLTrace("Writing setpoint wit vy=%f\n", current_setpoint.vy);

	// double check some system parameters
	if ( !sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message, ctxt);

	// check the write
	if ( !(len > 0) )
		SLTrace("WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}

// ------------------------------------------------------------------------------
//   Read a Single Mavlink Message from Serial
// ------------------------------------------------------------------------------
int read_message(mavlink_message_t &message, GcContext *ctxt)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;
  bool debug = false;

	// --------------------------------------------------------------------------
	//   READ FROM PORT
	// --------------------------------------------------------------------------

	// this function locks the port during read
	//int result = _read_port(cp);
  //int result = read(fd, &cp, 1);
  //SLTrace("read_message: Attempting to read serial port\n");
  s32 result = ctxt->autopilotPort->Read(&cp, 1, 1);  //xxxx using SL read
  //SLTrace("read_message: %c\n", cp);

	// --------------------------------------------------------------------------
	//   PARSE MESSAGE
	// --------------------------------------------------------------------------
	if(result > 0)
	{
    //SLTrace("read_message: Read something from serial port\n"); //works
		// the parsing
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

		// check for dropped packets
		if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
		{
			SLTrace("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v=cp;
			SLTrace("%02x ", v);
		}
		lastStatus = status;
	}

	// Couldn't read from port
	else if(result < 0)
	{
		SLTrace("ERROR: Could not read from serial port\n");
	}

	// --------------------------------------------------------------------------
	//   DEBUGGING REPORTS
	// --------------------------------------------------------------------------
	if(msgReceived && debug)
	{
		// Report info
		SLTrace("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

		SLTrace("Received serial data: ");
		unsigned int i;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

		// check message is write length
		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

		// message length error
		if (messageLength > MAVLINK_MAX_PACKET_LEN)
		{
			SLTrace("\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
		}

		// print out the buffer
		else
		{
			for (i=0; i<messageLength; i++)
			{
				unsigned char v=buffer[i];
				SLTrace("%02x ", v);
			}
			SLTrace("\n");
		}
	}

	// Done!
	return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int write_message(mavlink_message_t &message, GcContext* ctxt)
{
	char buf[300];
  s32 writeLen = 0;

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port
	writeLen = ctxt->autopilotPort->Write(buf,len);
  //SLTrace("Wrote %i bytes\n", len);

	return len;
}

// Get timestamp
uint64_t get_time_usec()
{
#ifdef WIN32
  LARGE_INTEGER Time;
	//Time.QuadPart = GetTickCount();
	//Time.QuadPart *= 1000;
	//return (UInt64)(Time.QuadPart - StartTime.QuadPart);

  QueryPerformanceCounter(&Time);
  return (uint64_t)Time.QuadPart;
#else
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
#endif
}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;  //xxx try MAV_FRAME_BODY_NED ;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}
