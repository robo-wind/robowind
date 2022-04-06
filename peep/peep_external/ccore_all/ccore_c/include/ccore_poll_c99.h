#pragma once

#include <stdlib.h>
#include <bson/bson.h>
#include <ccorebase_poll_c99.h>
#include <ccore_msg_c99.h>

#ifdef __cplusplus
extern "C" {
#endif
    
    //
    CCORE_EXPORT bool
    ccore_poll_receive_msg (ccore_poll_t *self, ccore_msg_t** msg, int max_wait);
    
    //
    CCORE_EXPORT bool
    ccore_poll_send_bson (ccore_poll_t *self, const char *path, bson_t* payload);

#ifdef __cplusplus
}
#endif

