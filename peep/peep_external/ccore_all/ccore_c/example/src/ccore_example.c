#include "ccore_c99.h"
#include <bson/bson.h>
#include <bson/bcon.h>

#ifdef _WIN32
#include <Windows.h>
#endif


void ccore_sleep(unsigned int seconds)
{
#ifdef _WIN32
    Sleep(seconds);
#else
    sleep(seconds);
#endif
}
void
handle_ccore_msg(ccore_msg_t* msg, void* callback_data)
{
    char* result;
    result = bson_as_json(&msg->bson, NULL);
    //BCON_EXTRACT(msg->bson, "data", BCONE_UTF8(result));
    
    printf("Path %s, payload %s\n", msg->path, result);
    
    bson_free(result);
    ccore_msg_free(msg);
}


int
main (int argc, char *argv [])
{
    ccore_t* client = ccore_new();
    ccore_add_pub(client, "zmq-tcp://*:15555");
    ccore_add_sub(client, "zmq-tcp://127.0.0.1:15555");
    
    bson_t *bson_doc = BCON_NEW ("path", "/test",
                                 "data", "Hello World");
    
    ccore_subscribe(client, "/test", &handle_ccore_msg, NULL);
    
    for(int i=0; i<5; i++)
    {
        ccore_sleep(1);
        ccore_send_bson(client, "/test", bson_doc);

    }
    
    ccore_sleep(1);
        
    ccore_destroy(&client);

    
}

