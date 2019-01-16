/*
 * Copyright (C)2007-2016 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 * NOTES:
 * Throughout the code we use _WIN32 preprossor defines so that code can be
 * run on Windows (for initial logic debugging) or on the TARGET (final debugging)
 *
 * This demo code was built with the following assumptions:
 *  - VideoTrack is running as a seperate process on the ARM 
 *    - VideoTrack listens on 14003 (FIP2 - See Sightline IDD) (inbound)
 *    - We will send commands to the autopilot over serial port 
 *    - VideoTrack sends telemetry and responses back to 16002
 *    
 *------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>

#include "slcommon.h"
#include "slos.h"
#include "slfip.h"
#include "slsock.h"
#include "slrs232.h"

#include "laVtRes.h"
#include "laSla.h"

#ifdef _WIN32
  #define MICROCTRL_SERIALPORT 6               // Replace with your PC COM PORT
  #define IPADDR_VIDEOTRACK   "192.168.0.116"	 // IP address of SightLine Hardware
#else
  #define MICROCTRL_SERIALPORT  0              // SLA-1500 Serial Port
  #define IPADDR_VIDEOTRACK     "127.0.0.1"    // localhost
#endif

////////////////////////////////////////////////////////////////////////////////
static int runMain(GcContext *ctxt)
{
  // Setup socket to VideoTrack
  SLSockUDP *vtPort = new SLSockUDP();
  if(vtPort && vtPort->Initialize(IPADDR_VIDEOTRACK, SLFIP_TO_BOARD_PORT2, GC_FROM_VT_PORT)!=SLA_SUCCESS) {
    SLTrace("Failed to setup socket to VideoTrack\n");
    return SLA_FAIL;
  }
  ctxt->vtPort = vtPort;

  // Open serial port to Autopilot
  SLRs232 *SLRs232Port = new SLRs232();
  if(SLRs232Port->Open((u32)MICROCTRL_SERIALPORT, 57600, 8, 1, 0) != 0) 
  {
    SLTrace("Failed to open autopilot serial port at %i\n", ctxt->autopilotCommPort);
    return SLA_FAIL;
  }
  ctxt->autopilotPort = SLRs232Port;

  // Start the main processing threads
  GcStartup(ctxt);

  // Wait for done - this would most likely come from other threads 
  // failing for some reason.
  do {
    SLSleep(200);
  } while(!ctxt->done);

  return 0;
}

/*! Primary entry point that initializes state data, communication, and starts processing threads
 */
int main(int argc, char* argv[])
{
  GcContext context;
  SLMemset(&context, 0, sizeof(context));
  SLSysParams params = SLSysParamsDefault();
  context.sys = SLSysInit(&params);

  // Set pointers to command processing functions
  context.nVtRes = GcVtResSetupCb(context.vtRes);

  // Initialize structure to store the "state" of parameters related to the VideoTrack
  GcInitState(&context.vidState);

  // Initialize landing aid
  LaInitState(&context.landingAidState);

  // Start the main process
  return runMain(&context);
}
