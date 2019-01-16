/* 
 * Methods to communicate with an autopilot
 *
 * Copyright (C)2008-2013 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 *------------------------------------------------------------------------*/
#pragma once

///////////////////////////////////////////////////////////////////////////////
#include "sltypes.h"
#include "laSla.h"
//#include "CommManager.h"

#define RAD_TO_DEGf 57.2957795131f
#define DEG_TO_RADf 0.01745329252f

typedef struct {
  f32 rollAngle;
  f32 pitchAngle;
  f32 rollRate;
  f32 pitchRate;
  f32 altitude;
  f32 Xvel;       // Vehicle fore/aft inertial velocity. Pos=Forward.
  f32 Yvel;       // Vehicle left/right inertial velocity. Pos=Right.
  f32 VDown;      // Vehicle vertical velocity. Pos=down.
  u64 timestamp;
} AutopilotData;

// Main task to run indefinately and read data from the autopilot and write commands
int RunAutopilotComms(GcContext* ctxt);

// Retreive data from autopilot
SLStatus GetAutopilotData(AutopilotData* autopilotData);

// Determine if the autopilot is in a mode that is safe to proceed with vision-based landing
SLStatus IsAutopilotReadyForEnable2();

// Send whatever commands needed to steer the autopilot during a landing
SLStatus SetAutopilotModeForEnable2(GcContext *ctxt);

// Check that the autopilot is in the correct mode to proceed with the landing
SLStatus IsAutopilotReadyForProsecute();

// Semd longitudinal velocity, lateral velocity, and vertical velocity commands to the autopilot
SLStatus SendSteeringCommandsToAP(f32 Xvel, f32 YVel, f32 ClimbRateFraction);

// Do something on the autopilot to indicate SL landing tracking status.  This could be
//  something like turning on a light.
void IndicateLandingAidTrackingStatus(u8 Status1, u8 Status2);

// Check that the autopilot is still in the correct mode to continue the landing
SLStatus IsAPInLandingMode();