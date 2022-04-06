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
#include <stdlib.h>
#include "ccore_common.h"

struct ccore_t;
typedef struct ccore_t ccore_t;
        
        
// Callback for data received from ZMQ. Each callback will include message path and payload,
// along with callback_data passed during ccore_subscribe
typedef void (ccore_sub_callback_f)(const char* path, const uint8_t* payload, size_t payload_size,
                                    void* callback_data);

typedef void (ccore_customer_callback_f)(int32_t status, const uint8_t* payload,
                                          size_t payload_size, void* callback_data);

struct ccore_send_results_t;
typedef struct ccore_send_results_t ccore_send_results_t;

typedef void (ccore_worker_callback_f)(const char* path, const uint8_t* in_payload,
                                       size_t in_payload_size, ccore_send_results_t* send_context,
                                       void* callback_data);

//  Create a new CommuniCore client
CCORE_EXPORT ccore_t *
ccore_new (void);

CCORE_EXPORT ccore_t *
ccore_new_with_name (const char *name);

//  Destroy a CommuniCore client
CCORE_EXPORT void
ccore_destroy (ccore_t **self_p);

//
CCORE_EXPORT bool
ccore_add_pub (ccore_t *self, const char *pub_uri);

//
CCORE_EXPORT bool
ccore_add_sub (ccore_t *self, const char *sub_uri);

//
CCORE_EXPORT int
ccore_subscribe_raw (ccore_t *self, const char *path, ccore_sub_callback_f callback, void *callback_data);

//
CCORE_EXPORT void
ccore_unsubscribe (ccore_t *self, int subscription_id);

//
CCORE_EXPORT bool
ccore_send_raw (ccore_t *self, const char *path, const unsigned char *payload, size_t payload_size);

//
CCORE_EXPORT void
ccore_enable_watchdog(ccore_t *self, bool enable);

//
CCORE_EXPORT void
ccore_health_start(ccore_t *self, int32_t time_ms);

//
CCORE_EXPORT void
ccore_health_stop(ccore_t *self);
        
//
CCORE_EXPORT void
ccore_set_health (ccore_t *self, enum health_status_t status, const char* message);
        
//
CCORE_EXPORT void
ccore_set_health_userdata (ccore_t *self, const uint8_t* bsonData, int32_t size);

//
CCORE_EXPORT void
ccore_set_project_name(ccore_t *self, const char *project);
        
//
CCORE_EXPORT bool
ccore_log(ccore_t *self, enum log_level_t level, const char* message);
        
//
CCORE_EXPORT void
ccore_shutdown(void);

//
CCORE_EXPORT bool
ccore_add_task_hub(ccore_t *self, const char* task_hub_uri);

//
CCORE_EXPORT uint32_t
ccore_register_task_handler(ccore_t *self, const char *path, ccore_worker_callback_f callback,
                            void* callback_data);
        
        
CCORE_EXPORT void
ccore_send_task_results (ccore_send_results_t* send_context, bool success, const uint8_t* payload,
                         size_t payload_size);

//
CCORE_EXPORT void
ccore_run_task(ccore_t *self, const char *path, const uint8_t *payload, size_t payload_size,
              ccore_customer_callback_f callback, void* callback_data, uint32_t timeout,
              uint32_t start_timeout);
        
#ifdef __cplusplus
    }
#endif

