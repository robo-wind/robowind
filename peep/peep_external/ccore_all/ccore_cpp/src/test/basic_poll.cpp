#include <thread>
#include <chrono>
#include <iostream>
#include "bson/bson.h"
#include "ccore_poll_libbson.h"

void
handle_ccore_msg(std::unique_ptr<ccore::msg_t> &msg)
{
    if (msg->get_path() == "/test")
    {
        const char* result;
        BCON_EXTRACT(&msg->get_bson(), "data", BCONE_UTF8(result));
        std::cout << msg->get_path() << ":" << result << std::endl;
    }
}


int
main (int argc, char *argv [])
{
    ccore::client_poll_t* client = new ccore::client_poll_t();
    client->add_pub("zmq-tcp://127.0.0.1");
    client->add_sub("zmq-tcp://127.0.0.1");
    
    bson_t *bson_doc = BCON_NEW ("data", "Hello World");
     
    client->subscribe("");
    
    for(int i=0; i<50; i++)
    {
        client->send("/test", *bson_doc);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::unique_ptr<ccore::msg_t> msg = client->receive_bson_t(0);

        if (msg != nullptr)
        {
            handle_ccore_msg(msg);
        }
        else
        {
            printf("Nope\n");
        }
    }
    
    delete client;
}

