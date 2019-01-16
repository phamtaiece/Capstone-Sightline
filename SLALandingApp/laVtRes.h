/*
 * Copyright (C)2008-2013 SightLine Applications Inc
 * SightLine Applications Library of signal, vision, and speech processing
 * http://www.sightlineapplications.com
 *
 *------------------------------------------------------------------------*/
#pragma once

#include "slcommon.h"
#include "laSla.h"

///////////////////////////////////////////////////////////////////////////////
s32 GcVtResSetupCb(HandlerCallback *cb);
s32 GcFromVtTask(void *context);

/*! Initialize global landing process state, PID values, and landing aid position values
 * @param landingAidState landing state data to be initialized
 * @return always returns success
 */
SLStatus LaInitState(LandingAidState* landingAidState);

SLStatus TrackLandingPosition(void *context);

SLStatus PrintFileHeader(GcContext *ctxt);
