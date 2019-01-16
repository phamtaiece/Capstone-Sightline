/*
 * Copyright (C)2007-2016 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 *------------------------------------------------------------------------*/
#include <stdlib.h>

#include "slsock.h"
#include "slos.h"

#include "laVtRes.h"
#include "laSla.h"
#include "laAutopilot.h"

void GcInitState(VidState *state)
{
  // SLA State
  state->sla.img.capWide = 0;
  state->sla.img.capHigh = 0;
  state->sla.img.disWide = 640;
  state->sla.img.disHigh = 480;
  state->sla.img.disRectCol = 0;
  state->sla.img.disRectRow = 0;
  state->sla.img.disRectWide = 640;
  state->sla.img.disRectHigh = 480;
}

/*! Get state information from VideoTrack such as image sizes.
 * Configure VideoTrack to send telemtry data to this applicaiton using SetTelemetryDestination (0x64).
 * Enable Landing Aid telemetry and set the rate to correspond to frame rate using CoordinateReportingMode (0x0B)
 * Request the version number and image size to initialize pixel calibration.
 * @param ctxt global data that provide communication interfaces to VideoTrack
 * @return SUCCESS
 */
static SLStatus getAllFromSla(GcContext *ctxt)
{
  s32 sleepMs = 30;

  // Request state from SLA
  u8 buf[80];
  s32 len;

#ifdef _WIN32
  char *ipaddr = "192.168.0.99";
#else
  char *ipaddr = "127.0.0.1";
#endif

  len = SLFIPSetPacketDestination(buf, SL_SETEL_ADD, 0, ntohl(inet_addr(ipaddr)), GC_FROM_VT_PORT/*, 1, 1*/); 
  ctxt->vtPort->Write(buf, len); SLSleep(sleepMs); SLSleep(100);
  // disable telemtry while we request other data
  len = SLFIPSetCoordinateReportingPeriod(buf, 0, SL_COORD_REPORT_LANDING_AID);
  ctxt->vtPort->Write(buf, len); SLSleep(sleepMs);
  SLSleep(100);

  // Get data
  len = SLFIPGetVersionNumber(buf);              ctxt->vtPort->Write(buf, len);  SLSleep(sleepMs);
  len = SLFIPGetImageSize(buf);                  ctxt->vtPort->Write(buf, len);  SLSleep(sleepMs);

  // Let parameter change take
  SLSleep(200); 

  // Report every frame
  len = SLFIPSetCoordinateReportingPeriod(buf, 1, SL_COORD_REPORT_LANDING_AID); 
  ctxt->vtPort->Write(buf, len);  SLSleep(sleepMs);

  // Let parameter change take
  SLSleep(200); 

  return SLA_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// Thread used to run autopilot communications over serial
///////////////////////////////////////////////////////////////////////////////
static int AutopilotCommsTask(void *context)
{
  GcContext *ctxt = (GcContext*)context;

  while(!ctxt->done)
    RunAutopilotComms(ctxt);

  return 0;
}

s32 PostTxToAutopilotPacket(GcContext *ctxt, CommandPacket *pkt)
{
  return SLMbxPost(ctxt->hmbxTx, pkt, 0);
}

SLStatus GcStartup(GcContext *ctxt)
{
  SLTaskPriority priorityCommand=SL_PRI_8, priorityRespond=SL_PRI_6, priorityTx=SL_PRI_7;

  ctxt->hmbxTx = SLMbxCreate(ctxt->sys, sizeof(CommandPacket), 10, "hmbxTx");

  if(SLCreateThread(AutopilotCommsTask, SL_DEFAULT_STACK_SIZE, "AutopilotCommsTask", ctxt, priorityTx)) {
    if(SLCreateThread(GcFromVtTask, SL_DEFAULT_STACK_SIZE, "respond", ctxt, priorityRespond)) {
      getAllFromSla(ctxt);
      return SLA_SUCCESS;
    }
  }

  SLTrace("ERROR: GcStartup: Failed to start a thread\n");

  ctxt->done = true;
  return SLA_ERROR;
}
