/*
 * Copyright (C)2008-2013 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 *------------------------------------------------------------------------*/
#pragma once
#include <stdio.h>

#include "slcommon.h"
#include "slport.h"
#include "slfip.h"

#define GC_FROM_VT_PORT 16002
typedef SLStatus (*HandlerCallback)(void *context, const u8 *data);

//!< Hardawre and Software information about the SightLine System.
typedef struct {
  u8 swMajor;     //!< Software major version
  u8 swMinor;     //!< Software minor version
  u8 hwVersion;   //!< Hardware type 
  u8 tempDegF;    //!< current operating temperature
  u8 hwId[8];     //!< Unique hardware ID (serial number)
  u8 swRevision;  //!< Software revision number
} VTVersion;

//!< Capture and Display image information
typedef struct {
  s16 capWide;
  s16 capHigh;
  s16 disWide;
  s16 disHigh;
  s16 disRectCol;   
  s16 disRectRow;   
  s16 disRectWide;  
  s16 disRectHigh;
} VTImageSize;

typedef struct {
  f32 XPos;           //!< Fore/aft position of landing target in vehicle camera frame
  f32 YPos;           //!< Lateral position of landing target in vehicle camera frame
  u16 angleDeg;       //!< Rotational angle of landing target in vehicle camera frame
  f32 distance;       //!< Distance to target in whatever units were used to define the target geometry in SLAPanel
  u8  confidence;     //!< Landing aid match confidence,  0-100
  u64 timestamp;      //!< microseconds
} VTLandingPosition;

//!< State information from VideoTrack
typedef struct {
  VTVersion         ver;        //!< software and hardware version information
  VTImageSize       img;        //!< Capture and display image properties
  VTLandingPosition landingPos; //!< Current landing target position information
} VTState;

//!< Control position for platform
typedef struct {
  f32 integralX;    //!< Integral state for fore/aft control
  f32 integralY;    //!< Integral state for left/right control
  f32 cmdX;         //!< Fore/aft output of the landing controller, fraction
  f32 cmdY;         //!< Left/Right output of the landing controller, fraction
  f32 pGain;        //!< Proportional gain for landing controller
  f32 dGain;        //!< Derivative gain for landing controller
  f32 iGain;        //!< Integral gain for landing controller
  f32 trimX;        //!< Trim fore/aft output
  f32 trimY;        //!< Trim left/right output
} LAPid;

typedef struct {
  f32 enableConfidenceThresh;     //!< Triggers entry to prosecuting mode
  f32 holdConfidenceThresh;       //!< Triggers state change from prosecuting to hold
  f32 holdAngleThresh;            //!< Triggers state change from prosecuting to hold, rad
  f32 resumeConfidenceThresh;     //!< Triggers state change from hold back to prosecuting
  f32 resumeAngleThresh;          //!< Triggers state change from hold back to prosecuting, rad
} Thresholds;

//!< Landing process can be in one of the following modes:
typedef enum {
  LANDING_APP_WAITING_STATE,       //!< Waiting to be enabled by user
  LANDING_APP_ENABLED1_STATE,      //!< Waiting for a target and autopilot in the correct mode
  LANDING_APP_ENABLED2_STATE,      //!< Checking autopilot handshake
  LANDING_APP_PROSECUTING_STATE,   //!< Descending under landing app control
  LANDING_APP_STICKIT_STATE,       //!< Final drop to the target
  LANDING_APP_HOLD_STATE,          //!< Stop descent and re-center on target
  LANDING_APP_HOLD_AND_CLIMB_STATE //!< Climb while trying to re-acquire target
} LA_APP_STATE;

//!< Data that tracks the visual landing aid position and the aircraft control rates.
typedef struct {
  LA_APP_STATE state;     //!< Current data used to define landing position information
  LAPid pid;              //!< Current data used to control the aircraft
  Thresholds thresholds;  
  f32 descentRate;
} LandingAidState;

///////////////////////////////////////////////////////////////////////////////
typedef struct {
  s32 len;
  u8 buf[MAX_SLFIP_PACKET];
} CommandPacket;

///////////////////////////////////////////////////////////////////////////////
typedef struct {
  VTState sla;
  u32 frame;
} VidState;

///////////////////////////////////////////////////////////////////////////////
typedef struct s_GcContext {
  SLSys *sys;
  bool done;
  SLPort *autopilotPort;   //!< serial object to communicate to/from autopilot
  u8 autopilotCommPort;
  SLPort *vtPort;

  HandlerCallback vtRes[LastFipNumber];
  u32 nVtRes;
  void* hmbxTx;           //!< mailbox handle for FIP transmit task
  VidState vidState;
  LandingAidState landingAidState;
  FILE* pLAFile;
} GcContext;

/*! Initialize the image capture size to default values
* This will be updated with request for information from VideoTrack
* @see getAllFromSla
*/void GcInitState(VidState *state);

/*! Start threads and intinialize VideoTrack
 * Two threads are created: one for receiving data from VideoTrack and the other for sending data to PixHawk
 * @param ctxt
 * @return Error if threads were not started or initialization failed.  Otherwise, return SUCCESS.
*/
SLStatus GcStartup(GcContext *ctxt);

/*! Simple wrapper for posting packets to the sending thread
 * @param ctxt 
 * @param pkt Data to be sent to PixHawk
 * @return 0 on success.
 */
s32 PostTxToAutopilotPacket(GcContext *ctxt, CommandPacket *pkt);

