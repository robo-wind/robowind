#include "ccore_libbson.h"
#include "bson/bson.h"
#include <thread>
#include <chrono>
#include <iostream>

void
handle_ccore_msg(std::unique_ptr<ccore::msg_t> msg, void* callback_data)
{
    const char* result;
    BCON_EXTRACT(&msg->get_bson(), "data", BCONE_UTF8(result));
    std::cout << msg->get_path() << ":" << result << std::endl;
}


int
main (int argc, char *argv [])
{
    ccore::client_t* client = new ccore::client_t();
    client->add_pub("zmq-tcp://127.0.0.1");
    client->add_sub("zmq-tcp://127.0.0.1");
    
    
    // Example: Hello world. Path and data fields are required
    bson_t *simple_bson = BCON_NEW ("path", "/test",
                                    "data", "Hello World");

    
    // Example: parent with a child and binary data
    bson_t *child_bson = BCON_NEW ("version", "Beta",
                                   "binary", BCON_BIN(BSON_SUBTYPE_BINARY, (uint8_t *) "deadbeef", 8));
    bson_t *parent_bson = BCON_NEW ("path", "/version",
                                    "data",  BCON_DOCUMENT(child_bson));
    
    client->subscribe("/test", handle_ccore_msg, NULL);
    
    for(int i=0; i<5; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        client->send("/test", *simple_bson);
        client->send("/version", *parent_bson);
        
    }
    
    bson_destroy(simple_bson);
    bson_destroy(parent_bson);
    bson_destroy(child_bson);
    delete client;
    
//ccore_add
}

