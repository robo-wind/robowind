
//
//  Copyright © 2022 Robowind. All rights reserved.
//


#include "ccore_poll_c99.h"
#include "ccore_msg_c99.h"

//
bool
ccore_poll_receive_msg (ccore_poll_t *self, ccore_msg_t** msg, int max_wait)
{
    char* path;
    uint8_t* payload;
    size_t payload_size;
    
    bool ret = ccore_poll_receive_raw(self, &path, &payload, &payload_size, max_wait);
    if (!ret)
    {
        return false;
    }

    // We own both payload and path and need to free them
    // when we are done
    *msg = (ccore_msg_t*) malloc (sizeof (ccore_msg_t));
    (*msg)->raw_payload = payload;
    (*msg)->owned_payload = true;
    (*msg)->raw_payload_size = payload_size;
    (*msg)->path = path;
    (*msg)->owned_path = true;

    bson_init_static(&(*msg)->bson, payload, payload_size);
    
    return true;
}

//
bool
ccore_poll_send_bson (ccore_poll_t *self, const char *path, bson_t* payload)
{
    return ccore_poll_send_raw(self, path, bson_get_data(payload), payload->len);
}

//
void
ccore_poll_free_bson (void *ptr)
{
    bson_free(ptr);
}
