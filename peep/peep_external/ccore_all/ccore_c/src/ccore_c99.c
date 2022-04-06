
//
//  Copyright © 2022 Robowind. All rights reserved.
//


#include "ccore_c99.h"

struct callback_info_t
{
    ccore_sub_callback_msg_f* callback;
    void *callback_data;
};
typedef struct callback_info_t callback_info_t;


void
_handle_callback(const char* path, const uint8_t* payload, size_t payload_size, void* callback_data)
{
    callback_info_t* info = (callback_info_t*) callback_data;
    
    ccore_msg_t* msg = (ccore_msg_t*) malloc (sizeof (ccore_msg_t));
    msg->raw_payload = (uint8_t*) payload;
    // callback uses shared_ptrs for payload, it will free it once we
    // are done
    msg->owned_payload = false;
    msg->raw_payload_size = payload_size;
    msg->path = (char*) path;
    // callback uses std::string for path, it will free it once we are
    // done
    msg->owned_path = false;
    
    bson_init_static(&msg->bson, payload, payload_size);
    
    info->callback(msg, info->callback_data);
}


int
ccore_subscribe (ccore_t *self, const char *path, ccore_sub_callback_msg_f callback, void *callback_data)
{
    // Todo: we are leaking callback_info. Free on unsubscribe
    callback_info_t* info = (callback_info_t *) malloc (sizeof (callback_info_t));
    info->callback = callback;
    info->callback_data = callback_data;
    
    return ccore_subscribe_raw(self, path, _handle_callback, (void*)info);
}

//
bool
ccore_send_bson (ccore_t *self, const char *path, bson_t* payload)
{
    return ccore_send_raw(self, path, bson_get_data(payload), payload->len);
}
