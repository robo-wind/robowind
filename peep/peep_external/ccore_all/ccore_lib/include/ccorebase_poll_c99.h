/*  =========================================================================
    ccore - Project

    Copyright (c) the Authors
    =========================================================================
*/

#pragma once

#ifdef __cplusplus
    extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "ccore_common.h"

struct ccore_poll_t;
typedef struct ccore_poll_t ccore_poll_t;
        
//  Create a new CommuniCore client
CCORE_EXPORT ccore_poll_t *
ccore_poll_new (void);

CCORE_EXPORT ccore_poll_t *
ccore_poll_new_with_name (const char *name);
        
//  Destroy a CommuniCore client
CCORE_EXPORT void
ccore_poll_destroy (ccore_poll_t **self_p);

//
CCORE_EXPORT bool
ccore_poll_add_pub (ccore_poll_t *self, const char *pub_uri);

//
CCORE_EXPORT bool
ccore_poll_add_sub (ccore_poll_t *self, const char *sub_uri);

//
CCORE_EXPORT void
ccore_poll_subscribe (ccore_poll_t *self, const char *path);

//
CCORE_EXPORT void
ccore_poll_unsubscribe (ccore_poll_t *self, const char *path);

//
CCORE_EXPORT bool
ccore_poll_receive_raw (ccore_poll_t *self, char **path, unsigned char **payload, size_t *payload_size, int max_wait);


//
CCORE_EXPORT bool
ccore_poll_send_raw (ccore_poll_t *self, const char *path, const unsigned char *payload, size_t payload_size);

//
CCORE_EXPORT void
ccore_poll_enable_watchdog(ccore_poll_t *self, bool enable);
        
//
CCORE_EXPORT void
ccore_poll_free (void *ptr);

//
CCORE_EXPORT void
ccore_poll_health_start(ccore_poll_t *self, int32_t time_ms);

//
CCORE_EXPORT void
ccore_poll_health_stop(ccore_poll_t *self);

//
CCORE_EXPORT void
ccore_poll_set_health (ccore_poll_t *self, enum health_status_t status, const char* message);

//
CCORE_EXPORT void
ccore_poll_set_health_userdata (ccore_poll_t *self, const uint8_t* bsonData, int32_t size);
        
//
CCORE_EXPORT void
ccore_poll_set_project_name(ccore_poll_t *self, const char *project);
        
//
CCORE_EXPORT bool
ccore_poll_log(ccore_poll_t *self, enum log_level_t level, const char* message);
        
#ifdef __cplusplus
    }
#endif

