
//
//  Copyright © 2022 Robowind. All rights reserved.
//


#include "ccore_msg_c99.h"
#include <bson/bson.h>

void
ccore_msg_free(ccore_msg_t* msg)
{
    bson_destroy(&msg->bson);
    if (msg->owned_payload)
    {
        free(msg->raw_payload);
    }
    if (msg->owned_path)
    {
        free(msg->path);
    }
    free (msg);
}

