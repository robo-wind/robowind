#include <ccore_poll.h>
#include <bson.h>
#include <thread>
#include <chrono>
#include <iostream>

void
handle_test_msg(std::unique_ptr<ccore::message> msg, void* callback_data)
{
    std::string data = msg->get_bson()["data"];
    
    std::cout << "1:" << msg->get_path() << ", " << data << std::endl;
}


int
main (int argc, char *argv [])
{
    ccore::client_poll client;
    client.enable_watchdog();
    // In default docker-compose hub_failover(15555/15556) is disabled.
    // Client should switch to hub after watchdog.
    client.add_pub("zmq-tcp://127.0.0.1:15555,zmq-tcp://127.0.0.1:5555");
    client.add_sub("zmq-tcp://127.0.0.1:15556,zmq-tcp://127.0.0.1:5556");
    
    client.subscribe("/foo");
    
    bool exit = false;
    
    std::thread t([&client, &exit](){
        while(!exit)
        {
            std::unique_ptr<ccore::message> msg = client.receive(1000);
            if (msg)
            {
                handle_test_msg(std::move(msg), nullptr);
            }
        }
            
    });
    
    
    for(int i=0; i<500; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::string tdata = "Sending id: " + std::to_string(i);
        client.send("/foo", tdata);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    exit = true;
    t.join();
    
}

