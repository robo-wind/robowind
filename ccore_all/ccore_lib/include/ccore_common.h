//
//  Copyright © 2022 Robowind. All rights reserved.
//

#pragma once

#include <stdlib.h>

#ifndef ZMQ_BUILD_DRAFT_API
  #define ZMQ_BUILD_DRAFT_API
#endif
#ifndef CZMQ_BUILD_DRAFT_API
  #define CZMQ_BUILD_DRAFT_API
#endif

#if defined _WIN32
#   if defined CCORE_STATIC
#       define CCORE_EXPORT
#   elif defined CCORE_INTERNAL_BUILD
#       if defined DLL_EXPORT
#           define CCORE_EXPORT __declspec(dllexport)
#       else
#           define CCORE_EXPORT
#       endif
#   elif defined CCORE_EXPORTS
#       define CCORE_EXPORT __declspec(dllexport)
#   else
#       define CCORE_EXPORT __declspec(dllimport)
#   endif
#else
#   define CCORE_EXPORT
#endif


//// General Settings
#define CCORE_DEFAULTS_HUB "127.0.0.1"

/// Default pub port
#define CCORE_DEFAULTS_PUB_PORT 5555
/// Default sub port
#define CCORE_DEFAULTS_SUB_PORT 5556
/// Default port hub uses to broadcast traffic it passes through it
#define CCORE_DEFAULTS_FIREHOSE_PORT 5560

/// Default task hub port
#define CCORE_DEFAULTS_TASK_PORT 5561

/// Default websocket proxy port
#define CCORE_DEFAULTS_WEBSOCKET_PORT 8081

/// Send heartbeat to all subscribers every X seconds
#define CCORE_DEFAULTS_TIMER_SECONDS 5

//// Task Hub Defaults
/// If Hub does not find a valid worked in specified
/// number of seconds, return with UNAVAILABLE error
#define CCORE_DEFAULTS_TASK_START_TIMEOUT 30
/// If Task does not finish in specified number of
/// seconds, return with TIMEOUT error
#define CCORE_DEFAULTS_TASK_TIMEOUT       600


// Coarse health status
enum health_status_t
{
    /// Error
    STATUS_RED,
    /// Warning
    STATUS_YELLOW,
    /// OK
    STATUS_GREEN
};

enum task_status_t
{
    /// Worker could not execute task or sent back invalid result
    TASK_ERROR   = 1,
    /// Task hub could not find a worker to execute the task
    TASK_UNAVAILABLE = 2,
    /// Task timed out before completion
    TASK_TIMEOUT = 3,
    /// Task was sent to a worked which does not handle task path
    TASK_UNHANDLED = 4,
    /// Worker could not parse function arguments
    TASK_INVALID_ARGS = 5,
    /// Success
    TASK_SUCCESS = 6
};

enum log_level_t
{
    CCORE_DEBUG, CCORE_INFO, CCORE_WARN, CCORE_ERROR
};
