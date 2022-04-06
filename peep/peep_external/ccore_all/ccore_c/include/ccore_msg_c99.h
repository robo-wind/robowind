#pragma once

#include <stdlib.h>
#include <bson/bson.h>
#include <ccorebase_c99.h>

#ifdef __cplusplus
    extern "C" {
#endif

struct ccore_msg_t
{
    char* path;
    // does this object own memory held by path
    bool owned_path;
    bson_t bson;
    uint8_t* raw_payload;
    // does this object own memory held by payload
    bool owned_payload;
    size_t raw_payload_size;
    
};

typedef struct ccore_msg_t ccore_msg_t;
        
        
void
ccore_msg_free(ccore_msg_t* msg);
        
#ifdef __cplusplus
    }
#endif

