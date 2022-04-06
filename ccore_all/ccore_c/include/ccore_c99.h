#pragma once

#include <stdlib.h>
#include <bson/bson.h>
#include <ccorebase_c99.h>
#include <ccore_msg_c99.h>

#ifdef __cplusplus
    extern "C" {
#endif


typedef void (ccore_sub_callback_msg_f)(ccore_msg_t* message, void* callback_data);

//
CCORE_EXPORT int
    ccore_subscribe (ccore_t *self, const char *path, ccore_sub_callback_msg_f callback, void *callback_data);

//
CCORE_EXPORT bool
    ccore_send_bson (ccore_t *self, const char *path, bson_t* payload);
        
#ifdef __cplusplus
    }
#endif

