/*
 * Copyright (C)2007-2016 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 *------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>

#include "slcommon.h"
#include "slos.h"
#include "slfip.h"
#include "slfipport.h"
#include "slsock.h"
#include "slmemory.h"
#include "slmath.h"

#include "laVtRes.h"
#include "laSla.h"
#include "laAutopilot.h"

#define LA_STATE_COUNTER_PROSECUTE 3        // number of frames of good landing data before doing a landing
#define LA_STATE_COUNTER_HOLD 40            // number of frames of bad angle before the landing is halted
#define LA_STATE_COUNTER_HOLD_AND_CLIMB 120 // number of frames to climb while looking for the target
#define LA_STATE_COUNTER_HOLD_RESUME 6      // number of frames of good data before landing state goes back to prosecute
#define LA_STATE_COUNTER_GOOD_STICKIT 3     // number of frames of stikit conditions met before transition from Prosecute to Stickit state
#define VT_LANDING_DT 0.0588f               // typical time difference between landing position updates from video tracker
#define GAIN_FADEOUT_DIST 2.0f              // distance to start reducing the lateral and longitudinal gains
#define DIST_CG_TO_CAMERA 0.095f            // vertical distance from cg to camera (pos down)

static SLStatus WriteDataToFile(GcContext *ctxt,
  f32 roll, f32 pitch, f32 rollUsed, f32 pitchUsed, f32 cmdX, f32 cmdY, f32 ClimbRateFractionCmd, u8 landingAidState,
  f32 distRaw, f32 distUsed, 
  f32 xAngleToTargetLL, f32 xAngleRate, f32 yAngleToTargetLL, f32 yAngleRate,
  f32 confidence, f32 stateCounter, f32 stateCounter2, f32 stikItArmed,
  f32 landingPosTimestamp, f32 autopilotTimestamp );
    
////////////////////////////////////////////////////////////////////////////////
s32 GcFromVtTask(void *context)
{
  GcContext *ctxt = (GcContext*)context;

  SLPort *port = ctxt->vtPort;
  if(port == NULL) {
    SLTrace("ERROR: GcFromVtTask: No port defined.\n");
    ctxt->done = true;
    return -1;
  }

  u8 buf[MAX_SLFIPEX_PACKET];
  const s32 timeout = 50;
  while (!ctxt->done) {
    SLSockUDP *sock = (SLSockUDP*)ctxt->vtPort;

    s32 len = FIPReadPacket(&buf[0], sock, timeout, true);

    if(len>=5 && buf[0]==0x51 && buf[1]==0xac) { // FIP protocol
      u8 offset = SLFIP_OFFSET_TYPE;
      u32 till = buf[SLFIP_OFFSET_LENGTH]-1;
      if( len > MAX_SLFIP_PAYLOAD ) {
        offset += 1;
        till = len-5;
      }
      u8 chk = SLComputeFIPChecksum(&buf[offset], till);
      if( chk == buf[len-1] )
      {
        u8 type = buf[offset];
        if(ctxt->vtRes[type]) {  // If we have registered for this callback 
          ctxt->vtRes[type](ctxt, buf);
        }
      }
    } 
    SLSleep(1);
  }

  ctxt->done = true;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
static SLStatus cbVersionNumber(void *context, const u8 *buf)
{
  GcContext *ctxt = (GcContext*)context;

  u8 size = buf[2];
  if(size>=15) {
    ctxt->vidState.sla.ver.swMajor    = buf[4];
    ctxt->vidState.sla.ver.swMinor    = buf[5];
    ctxt->vidState.sla.ver.hwVersion  = buf[6];
    ctxt->vidState.sla.ver.tempDegF   = buf[7];
    for(int i=0; i<8; i++)
      ctxt->vidState.sla.ver.hwId[i] = buf[8+i];
    ctxt->vidState.sla.ver.swRevision = buf[16];
  }

  SLTrace("GOT VERSION %d.%d\n", ctxt->vidState.sla.ver.swMajor , ctxt->vidState.sla.ver.swMinor);
  return SLA_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
static SLStatus cbCurrentImageSize(void *context, const u8 *buf)
{
  GcContext *ctxt = (GcContext*)context;

  u8 size = buf[2];
  if(size>=18) {
    u8 shortCount = 0;
    s16 *b = (s16*)&buf[4];
    ctxt->vidState.sla.img.capWide     = b[shortCount++];      
    ctxt->vidState.sla.img.capHigh     = b[shortCount++];      
    ctxt->vidState.sla.img.disWide     = b[shortCount++];      
    ctxt->vidState.sla.img.disHigh     = b[shortCount++];      
    ctxt->vidState.sla.img.disRectCol  = b[shortCount++];   
    ctxt->vidState.sla.img.disRectRow  = b[shortCount++];   
    ctxt->vidState.sla.img.disRectWide = b[shortCount++];  
    ctxt->vidState.sla.img.disRectHigh = b[shortCount++];
  }
  return SLA_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
static SLStatus cbLandingPosition(void *context, const u8 *buf)
{
  GcContext *ctxt = (GcContext*)context;
  u8 camIdx, confidence;
  s16 col, row;
  u16 angleDeg7;
  u32 distance16;
  f32 halfHFOV_rad = 0.0f;
  f32 halfVFOV_rad = 0.0f;
  s16 capHigh     = 720;
  s16 capWide     = 1280;
  f32 halfPixelHeight = capHigh/2.0f;
  f32 halfPixelWidth = capWide/2.0f;
  u16 camHFovDeg8 = 0;
  f32 aspectRatio = 0;
  u16 ctrlParam0 = 0;
  u16 ctrlParam1 = 0;
  u16 ctrlParam2 = 0;
  u16 ctrlParam3 = 0;
  u8 keepOutState = 0;
  u8 keepOutConf  = 0;
  u16 keepOutSz   = 0;
  u32 keepOutDist16 = 0;
  static u8 firstTime = 1;  

  // Get landing position data - 
  SLFIPUnpackLandingPosition((SLPacketType)buf, &camIdx, &col, &row, &angleDeg7, &distance16, &confidence,
                               &camHFovDeg8, &capWide, &capHigh, &ctrlParam0, &ctrlParam1, &ctrlParam2, &ctrlParam3, 
                               &keepOutState, &keepOutConf, &keepOutSz, &keepOutDist16 );

  // Timestamp landing data from Vt
  SLGetMHzTime(&(ctxt->vidState.sla.landingPos.timestamp));

  if(firstTime)
  {
    SLTrace("GOT LANDING PACKET FROM VT\n");
    firstTime = 0;
  }

  halfPixelWidth = capWide / 2.0f;
  halfPixelHeight = capHigh / 2.0f;
  aspectRatio = (f32)capHigh / (f32)capWide;
  halfHFOV_rad = ((f32)camHFovDeg8 / 256.0f) * SLPI180 / 2.0f;
  halfVFOV_rad = (((f32)camHFovDeg8 * aspectRatio) / 256.0f) * SLPI180 / 2.0f;

  //SLTrace("capHigh: %i\n", (int)capHigh);
  //SLTrace("capWide: %i\n", (int)capWide);

  if(halfPixelWidth < 1)
    halfPixelWidth = 1;
  if(halfPixelHeight < 1)
    halfPixelHeight = 1;

  // Convert image coordinates to angles to the target. col, row starts at top left.
  ctxt->vidState.sla.landingPos.XPos       = (-((f32)row - halfPixelHeight) / halfPixelHeight) * halfVFOV_rad;
  ctxt->vidState.sla.landingPos.YPos       = (((f32)col - halfPixelWidth) / halfPixelWidth) * halfHFOV_rad;
  ctxt->vidState.sla.landingPos.angleDeg   = (u16)(angleDeg7 / 128.0f); //angle 7 means divide by 128.0f
  ctxt->vidState.sla.landingPos.distance   = distance16 / 65536.0f;     //distance 16 means divide by 65536.0f 
  ctxt->vidState.sla.landingPos.confidence = confidence;

  // Set the tunable parameters (params come from SLAPanelPlus)
  ctxt->landingAidState.pid.pGain = (f32)(ctrlParam0) / 100.0f;
  ctxt->landingAidState.pid.dGain = (f32)(ctrlParam1) / 100.0f;
  ctxt->landingAidState.descentRate = (f32)(ctrlParam2) / 100.0f;
  ctxt->landingAidState.thresholds.holdAngleThresh = (f32)(ctrlParam3) * SLPI180;
  ctxt->landingAidState.thresholds.resumeAngleThresh = ctxt->landingAidState.thresholds.holdAngleThresh * 0.5f;

  if(ctxt->landingAidState.descentRate > 1.0f)
    ctxt->landingAidState.descentRate  = 1.0f;

  // Call landing controller immediately with fresh position data
  TrackLandingPosition(ctxt);

  return SLA_SUCCESS;
}

SLStatus LaInitState(LandingAidState* landingAidState)
{
  landingAidState->state = LANDING_APP_WAITING_STATE;

  SLMemset(&landingAidState->pid, 0, sizeof(LAPid));
  landingAidState->pid.integralX = 0.0f;
  landingAidState->pid.integralY = 0.0f;
  landingAidState->pid.pGain = 4.0f;
  landingAidState->pid.iGain = 0.0f;
  landingAidState->pid.dGain = 3.0f;
  landingAidState->pid.trimX = 0.0f;
  landingAidState->pid.trimY = 0.0f;
  landingAidState->pid.cmdX = 0;
  landingAidState->pid.cmdY = 0;

  landingAidState->thresholds.enableConfidenceThresh = 50.0f;
  landingAidState->thresholds.holdConfidenceThresh = 20.0f;
  landingAidState->thresholds.holdAngleThresh = 5.0f * SLPI180;
  landingAidState->thresholds.resumeConfidenceThresh =  70.0f;
  landingAidState->thresholds.resumeAngleThresh = 10.0f * SLPI180;

  return SLA_SUCCESS;
}


////////////////////////////////////////////////////////////////////////////////
// This is the controller used to input landing position information and create
//  a Autopilot control packet and send.
SLStatus TrackLandingPosition(void *context)
{
  GcContext *ctxt = (GcContext*)context;
  
  // Current landing aid data
  u8 confidence = ctxt->vidState.sla.landingPos.confidence;
  Thresholds* pThresholds = &(ctxt->landingAidState.thresholds);

  SLStatus apReady = SLA_FAIL;
  AutopilotData apData;
  static f32 holdAlt = -99.0f;
  f32 diffsec = 0;
  f32 apRollAngleUsed = 0.0f;     // Vehicle roll angle in body axis, corrected for delay
  f32 apPitchAngleUsed = 0.0f;    // Vehicle pitch angle in body axis, corrected for delay
  f32 xAngleToTargetLL = 0.0f;    // Longitudinal angle from vehicle to target in local level frame. Pos=target in front of the vehicle
  f32 yAngleToTargetLL = 0.0f;    // Lateral angle from vehicle to target in local level frame. Pos=target to the right of the vehicle
  f32 xAngleRate = 0.0f;          // Longitudinal angle rate from vehicle to target
  f32 yAngleRate = 0.0f;          // Lateral angle rate from vehicle to target
  f32 ClimbRateFractionCmd = 0.0f;// Vertical velocity command to the autopilot, fraction of maximum. Positive=Up.
  static u16 stateCounter = 0;    // Counter used for state transitions.  Replace with timer later.
  static u16 stateCounter2 = 0;   // 2nd counter used for state transitions
  static u16 staleAPdataCounter = 0;  // Counter used to determine stale autopilot data
  static u16 traceCounter = 1;        // Counter for debug messages
  static u16 logCounter = 1;		  // Counter for logging at a lower rate
  static f32 xAngleToTargetLL_prev = 0;
  static f32 yAngleToTargetLL_prev = 0;
  f32 yAngleAbs = 0.0f;           // Absolute value of y angle in local level frame, rad
  f32 xAngleAbs = 0.0f;           // Absolute value of x angle in local level frame, rad
  f32 fadeDistRatio = 1.0f;       // gain fade-out based on height above target
  static f32 distUsed = 0.0f;     // "valid" distance used in calculations.  If invalid, hold last value.
  static bool stikItArmed = false;// Used to prevent stickIt state too early

  // Get autopilot data
  memset(&apData, 0, sizeof(AutopilotData));
  if(GetAutopilotData(&apData) == SLA_SUCCESS)
    staleAPdataCounter = 50;
  else if(staleAPdataCounter > 0)
    staleAPdataCounter--;

  // Use the autopilot to announce the status of the landing aid.
  //  This could be something like turning on a light
  IndicateLandingAidTrackingStatus(staleAPdataCounter == 0,
    confidence > pThresholds->enableConfidenceThresh );

  // Time difference between receipt of autopilot data (older) and target data (newer)
  diffsec = (f32)(ctxt->vidState.sla.landingPos.timestamp - apData.timestamp) / 1000000.0f;
  if(diffsec < 0)
    diffsec = 0.0f;
  else if(diffsec > 0.2f)
    diffsec = 0.2f;

  //Calculate angles in local-level frame.  
  // Comment these lines out for HIL testing
  apRollAngleUsed = apData.rollAngle + apData.rollRate*diffsec;
  apPitchAngleUsed = apData.pitchAngle + apData.pitchRate*diffsec;
  
  // Subtract vehicle attitude from camera angle
  // but only if the camera angle is good.  Otherwise use previous value.
  if(confidence > pThresholds->holdConfidenceThresh)
  {
    yAngleToTargetLL = ctxt->vidState.sla.landingPos.YPos - apRollAngleUsed;
    xAngleToTargetLL = ctxt->vidState.sla.landingPos.XPos + apPitchAngleUsed;
  }
  else
  {
    yAngleToTargetLL = yAngleToTargetLL_prev;
    xAngleToTargetLL = xAngleToTargetLL_prev;
  }

  // Correct for camera vertical position
  //yAngleToTargetLL += atan(DIST_CG_TO_CAMERA * sin(apRollAngleUsed) / distUsed);
  //xAngleToTargetLL -= atan(DIST_CG_TO_CAMERA * sin(apPitchAngleUsed) / distUsed);

  yAngleAbs = SLABS(yAngleToTargetLL);
  xAngleAbs = SLABS(xAngleToTargetLL);

  // Calculate rate of change of the angle to the target
  yAngleRate = (yAngleToTargetLL - yAngleToTargetLL_prev) * VT_LANDING_DT;
  xAngleRate = (xAngleToTargetLL - xAngleToTargetLL_prev) * VT_LANDING_DT;
  yAngleToTargetLL_prev = yAngleToTargetLL;
  xAngleToTargetLL_prev = xAngleToTargetLL;

  //STATE MACHINE FOR LANDING AID
  switch(ctxt->landingAidState.state) 
  {
   case LANDING_APP_WAITING_STATE:
      // Wait for landing tracker to be enabled.  Using confidence until there is a way to determine enable
      if(confidence > 10)
      {
        SLTrace("\n\nENABLED-1 STATE\n");
        ctxt->landingAidState.state = LANDING_APP_ENABLED1_STATE;
        stateCounter = LA_STATE_COUNTER_PROSECUTE;
      }

      break;

   case LANDING_APP_ENABLED1_STATE:
      // Look for autopilot and landing tracker to be ready
      apReady = IsAutopilotReadyForEnable2();

      if( (apReady == SLA_SUCCESS)
        && (confidence > ctxt->landingAidState.thresholds.enableConfidenceThresh) )
      {
        if(stateCounter > 0)
          stateCounter--;

        if(stateCounter <= 0 && staleAPdataCounter > 0)
        {
          // Put autopilot into landing mode.  This may pause execution for some time
          SLStatus ready = SetAutopilotModeForEnable2(ctxt);
          
          if(ready == SLA_SUCCESS)
          {
            SLTrace("ENABLE-2 STATE\n");
            stateCounter = 40;
            ctxt->landingAidState.state = LANDING_APP_ENABLED2_STATE;
          }
        }
      }
      else
      {
        stateCounter = LA_STATE_COUNTER_PROSECUTE;  // Must have 3 good frames in a row
      }

      break;
      
   case LANDING_APP_ENABLED2_STATE:
      // Look for autopilot and landing tracker to be ready
      apReady = IsAutopilotReadyForProsecute();

      if( (apReady == SLA_SUCCESS)
        && (confidence > pThresholds->enableConfidenceThresh) )
      {
        if(stateCounter > 0)
          stateCounter--;

        if(stateCounter <= 0 && staleAPdataCounter > 0)
        {
          SLTrace("PROSECUTING STATE\n");
          stateCounter = LA_STATE_COUNTER_HOLD;
          stateCounter2 = LA_STATE_COUNTER_GOOD_STICKIT;
          ctxt->landingAidState.state = LANDING_APP_PROSECUTING_STATE;
        }
      }
      else
      {
        stateCounter = LA_STATE_COUNTER_PROSECUTE;  // Must have 3 good frames in a row
      }

      break;

   case LANDING_APP_PROSECUTING_STATE:
     // Stop landing if things get bad
     if( (confidence < pThresholds->holdConfidenceThresh) ||
         (((yAngleAbs > pThresholds->holdAngleThresh)
            || (xAngleAbs > pThresholds->holdAngleThresh)) )
        || (staleAPdataCounter <= 0) )
      {
        if(stateCounter > 0)
          stateCounter--;
     }
     else
     {
       if(stateCounter < LA_STATE_COUNTER_HOLD)
         stateCounter++;
     }

        // Too many bad measurements, go to hold
        if(stateCounter <=0)
        {
          // Remember current altitude
          holdAlt = apData.altitude;

       // Climb if needed to re-acquire target
       if( (confidence < pThresholds->holdConfidenceThresh)
       && (distUsed < 5.0f) )
       {
          stateCounter = LA_STATE_COUNTER_HOLD_AND_CLIMB;
          SLTrace("HOLD AND CLIMB STATE\n");
          ctxt->landingAidState.state = LANDING_APP_HOLD_AND_CLIMB_STATE;
       }
       else
       {
          stateCounter = LA_STATE_COUNTER_HOLD_RESUME;
          stateCounter2 = LA_STATE_COUNTER_HOLD_RESUME;
          SLTrace("HOLD STATE\n");
          ctxt->landingAidState.state = LANDING_APP_HOLD_STATE;
        }
      }

     if(distUsed > 3.0f)
       stikItArmed = true;


     // If we are just over the target and things look good then stick the landing
     if( stikItArmed
         && (yAngleAbs < 8.0f * SLPI180)
         && (xAngleAbs < 8.0f * SLPI180)
         && (yAngleRate < 0.523f)
         && (xAngleRate < 0.523f)
         && (distUsed < 0.5)
         && (confidence > 50) )
      {
        stateCounter2--;
      }
     else
     {
         if(stateCounter2 < LA_STATE_COUNTER_GOOD_STICKIT)
            stateCounter2++;
     }

     if(stateCounter2 <=0)
     {
        SLTrace("STICKIT STATE\n");
        ctxt->landingAidState.state = LANDING_APP_STICKIT_STATE;
     }

      //Exit this mode if the autopilot mode has changed
      if(IsAPInLandingMode() == SLA_FAIL)
      {
        SLTrace("WAITING STATE\n");
        ctxt->landingAidState.state = LANDING_APP_WAITING_STATE;
      }

      break;


   case LANDING_APP_STICKIT_STATE:
   
     //Exit this mode if the autopilot mode has changed
      if(IsAPInLandingMode() == SLA_FAIL)
      {
        SLTrace("WAITING STATE\n");
        ctxt->landingAidState.state = LANDING_APP_WAITING_STATE;
      }
      
     break; // no exit from this state for now


   case LANDING_APP_HOLD_STATE:
      // Determine when to start again
      if( //(confidence > ctxt->landingAidState.thresholds.resumeConfidenceThresh)
         (yAngleAbs < pThresholds->resumeAngleThresh)
        && (xAngleAbs < pThresholds->resumeAngleThresh)
        && (staleAPdataCounter > 0) )
      {
        if(stateCounter > 0)
          stateCounter--;
      }

      if(stateCounter <= 0)
      {
        SLTrace("PROSECUTING STATE\n");
        ctxt->landingAidState.state = LANDING_APP_PROSECUTING_STATE;
        stateCounter = LA_STATE_COUNTER_HOLD;
        stateCounter2 = LA_STATE_COUNTER_GOOD_STICKIT;
      }

      // Climb if needed to re-acquire target
      if( (confidence < pThresholds->holdConfidenceThresh)
      && (distUsed < 4.0f) )
      {
         stateCounter2--;
      }

      if(stateCounter2 <= 0)
      {
          SLTrace("HOLD AND CLIMB STATE\n");
          ctxt->landingAidState.state = LANDING_APP_HOLD_AND_CLIMB_STATE;
          stateCounter = LA_STATE_COUNTER_HOLD_AND_CLIMB;

          distUsed = 4.1f;    // a little extra insurance so we don't get stuck climbing
      }
      
      //Exit this mode if the autopilot mode has changed
      if(IsAPInLandingMode() == SLA_FAIL)
      {
        SLTrace("WAITING STATE\n");
        ctxt->landingAidState.state = LANDING_APP_WAITING_STATE;
      }

      break;

  
 
  case LANDING_APP_HOLD_AND_CLIMB_STATE:
  
      if(stateCounter > 0)
        stateCounter--;
        
      // Timeout so we don't climb forever
      if(stateCounter <=0)
      {
        SLTrace("Hold and Climb timed out\n");
        SLTrace("HOLD STATE\n");
        ctxt->landingAidState.state = LANDING_APP_HOLD_STATE;
        stateCounter = LA_STATE_COUNTER_HOLD_RESUME;
        stateCounter2 = LA_STATE_COUNTER_HOLD_RESUME;
      }
      
      // If target is back in view, stop climb
      if(confidence > pThresholds->resumeConfidenceThresh)
      {
        SLTrace("HOLD STATE\n");
        ctxt->landingAidState.state = LANDING_APP_HOLD_STATE;  
        stateCounter = LA_STATE_COUNTER_HOLD_RESUME;
        stateCounter2 = LA_STATE_COUNTER_HOLD_RESUME;
      }
      
      //Exit this mode if the autopilot mode has changed
      if(IsAPInLandingMode() == SLA_FAIL)
      {
        SLTrace("WAITING STATE\n");
        ctxt->landingAidState.state = LANDING_APP_WAITING_STATE;
      }
      
      break;

  } //switch landing state


  // LATERAL CONTROL
  if( ctxt->landingAidState.state == LANDING_APP_PROSECUTING_STATE
      || ctxt->landingAidState.state == LANDING_APP_HOLD_STATE
      || ctxt->landingAidState.state == LANDING_APP_HOLD_AND_CLIMB_STATE
      || ctxt->landingAidState.state == LANDING_APP_STICKIT_STATE )
  {
    
    if(confidence > 20)
    {
      //Calculate lateral commands
      ctxt->landingAidState.pid.cmdX = xAngleToTargetLL * ctxt->landingAidState.pid.pGain
                                    + xAngleRate * ctxt->landingAidState.pid.dGain;
      ctxt->landingAidState.pid.cmdY = yAngleToTargetLL * ctxt->landingAidState.pid.pGain
                                    + yAngleRate * ctxt->landingAidState.pid.dGain;

      distUsed = ctxt->vidState.sla.landingPos.distance;
    }
    else
    {
      ctxt->landingAidState.pid.cmdX = 0.0f;
      ctxt->landingAidState.pid.cmdY = 0.0f;
      // distUsed is remembered from previous value
    }

    // Scale gain based on distance to target
    fadeDistRatio = distUsed / GAIN_FADEOUT_DIST;
    if(fadeDistRatio < 1.0f)
    {
      // Minimum gain
      if(fadeDistRatio < 0.2f)
        fadeDistRatio = 0.2f;

      // Scale the commands
      ctxt->landingAidState.pid.cmdX *= fadeDistRatio;
      ctxt->landingAidState.pid.cmdY *= fadeDistRatio;
    }
    //SLTrace("fadeDistRatio: %f", fadeDistRatio);

  }
  else //if not Prosecute, Hold, or StickIt
  {
    // Command zero velocity
    ctxt->landingAidState.pid.cmdX = 0.0f;
    ctxt->landingAidState.pid.cmdY = 0.0f;
  }

  // VERTICAL CONTROL
  switch(ctxt->landingAidState.state)
  {
    case LANDING_APP_PROSECUTING_STATE:
      // Command descent
      if(distUsed > 1.0)
        ClimbRateFractionCmd = -ctxt->landingAidState.descentRate;
      else
        ClimbRateFractionCmd = -ctxt->landingAidState.descentRate * 0.5f; 
      break;

    case LANDING_APP_HOLD_STATE:
      // Command altitude hold
      ClimbRateFractionCmd = 0.0f;
      break;
      
    case LANDING_APP_HOLD_AND_CLIMB_STATE:
      // Command slow climb
      ClimbRateFractionCmd = ctxt->landingAidState.descentRate;
      break;

    case LANDING_APP_STICKIT_STATE:
      // Drop to the target
      ClimbRateFractionCmd = -ctxt->landingAidState.descentRate * 1.5f;
      break;

    default:
      break;
  }


  // Send command to autopilot
  SendSteeringCommandsToAP(
    ctxt->landingAidState.pid.cmdX, 
    ctxt->landingAidState.pid.cmdY,
    ClimbRateFractionCmd);
    
  // Write test data to file
  if (logCounter++ > 2)
  {
    WriteDataToFile(ctxt,
      apData.rollAngle,
      apData.pitchAngle,
      apRollAngleUsed,
      apPitchAngleUsed,
      ctxt->landingAidState.pid.cmdX,
      ctxt->landingAidState.pid.cmdY,
      ClimbRateFractionCmd,
      ctxt->landingAidState.state,
      ctxt->vidState.sla.landingPos.distance,
      distUsed,
      xAngleToTargetLL,
      xAngleRate,
      yAngleToTargetLL,
      yAngleRate,
      confidence,
      stateCounter,
      stateCounter2,
      stikItArmed,
      (f32)(ctxt->vidState.sla.landingPos.timestamp) / 1000000.0f,
      (f32)(apData.timestamp) / 1000000.0f);

    logCounter = 0;
  }

  return SLA_SUCCESS;
}

/*! Set up callback functions to handle replies from VideoTrack
 * Version Number (0x40) - used to verify communication with VideoTrack is active
 * Landing Position (0x83) - data from the located visual target (Precision Landing Aid)
 * Current Image Size (0x4e) - used to translate pixel to real-world coordinates
 * @return Total number of SightLine Command and Control Messages availale
 */
s32 GcVtResSetupCb(HandlerCallback *cb)
{
  SLMemset(cb, 0, LastFipNumber*sizeof(HandlerCallback));

  cb[VersionNumber]     = cbVersionNumber;
  cb[LandingPosition]   = cbLandingPosition;
  cb[CurrentImageSize]  = cbCurrentImageSize;

  return LastFipNumber;
}

/*! Write column headers to debug file
 @return Succces
 */
SLStatus PrintFileHeader(GcContext *ctxt)
{
  // Print file header
  fprintf(ctxt->pLAFile, " roll pitch rollUsed pitchUsed cmdX cmdY ClimbRateFractionCmd");
  fprintf(ctxt->pLAFile, " landingAidState distRaw distUsed");
  fprintf(ctxt->pLAFile, " xAngleToTargetLL xAngleRate yAngleToTargetLL yAngleRate");
  fprintf(ctxt->pLAFile, " confidence stateCounter stateCounter2 stikItArmed");
  fprintf(ctxt->pLAFile, " landingPosTimestamp autopilotTimestamp");
  fprintf(ctxt->pLAFile, " pid_pGain pid_iGain pid_dGain");
  fprintf(ctxt->pLAFile, " enableConfidenceThresh");
  fprintf(ctxt->pLAFile, " holdConfidenceThresh");
  fprintf(ctxt->pLAFile, " holdAngleThresh");
  fprintf(ctxt->pLAFile, " resumeConfidenceThresh");
  fprintf(ctxt->pLAFile, " resumeAngleThresh");
  fprintf(ctxt->pLAFile, "\n");

  return SLA_SUCCESS;
}

/*! Output data to file for debug / diagnostic purposes
 * @return Error if no file point is available, otherwise SUCCESS
 */
static SLStatus WriteDataToFile(GcContext *ctxt,
  f32 roll,
  f32 pitch,
  f32 rollUsed,
  f32 pitchUsed,
  f32 cmdX,
  f32 cmdY,
  f32 ClimbRateFractionCmd,
  u8 landingAidState,
  f32 distRaw,
  f32 distUsed,
  f32 xAngleToTargetLL,
  f32 xAngleRate,
  f32 yAngleToTargetLL,
  f32 yAngleRate,
  f32 confidence,
  f32 stateCounter,
  f32 stateCounter2,
  f32 stikItArmed,
  f32 landingPosTimestamp,
  f32 autopilotTimestamp)
{

  if(!ctxt->pLAFile)
    return SLA_ERROR;
    
  fprintf(ctxt->pLAFile, "%f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
    (float)roll, (float)pitch, (float)rollUsed, (float)pitchUsed,
    (float)cmdX, (float)cmdY, (float)ClimbRateFractionCmd,
    (u8)landingAidState, (float)distRaw, (float)distUsed,
    (float)xAngleToTargetLL, (float)xAngleRate, (float)yAngleToTargetLL, (float)yAngleRate,
    (float)confidence, (float)stateCounter, (float)stateCounter2, (float)stikItArmed,
    (float)landingPosTimestamp, (float)autopilotTimestamp,
    (float)ctxt->landingAidState.pid.pGain, (float)ctxt->landingAidState.pid.iGain, (float)ctxt->landingAidState.pid.dGain,
    (float)ctxt->landingAidState.thresholds.enableConfidenceThresh,
    (float)ctxt->landingAidState.thresholds.holdConfidenceThresh,
    (float)ctxt->landingAidState.thresholds.holdAngleThresh,
    (float)ctxt->landingAidState.thresholds.resumeConfidenceThresh,
    (float)ctxt->landingAidState.thresholds.resumeAngleThresh);
    
  return SLA_SUCCESS;
}
    
