#include "ccore_poll_c99.h"
#include <bson/bson.h>

#ifdef _WIN32
#include <Windows.h>
#endif

int
main (int argc, char *argv [])
{
    ccore_poll_t* client = ccore_poll_new();
    ccore_poll_add_pub(client, "zmq-tcp://127.0.0.1");
    ccore_poll_add_sub(client, "zmq-tcp://127.0.0.1");
    
    bson_t *bson_doc = BCON_NEW ("path", "/test",
                                 "data", "Hello World");
    
    ccore_poll_subscribe(client, "/test");
    
    for(int i=0; i<500000; i++)
    {
        ccore_poll_send_bson(client, "/test", bson_doc);
        #ifdef _WIN32
            Sleep(0.01);
        #else
            sleep(0.01);
        #endif
        
        char* path;
        ccore_msg_t* msg = NULL;
        bool ret = ccore_poll_receive_msg(client, &msg, 100);
        if (!ret)
        {
            continue;
        }
        char* result = bson_as_json(&msg->bson, NULL);
        
        printf("Path: %s, payload %s\n", msg->path, result);
        
        bson_free(result);
        ccore_msg_free(msg);
    }
    
    ccore_poll_destroy(&client);

}

