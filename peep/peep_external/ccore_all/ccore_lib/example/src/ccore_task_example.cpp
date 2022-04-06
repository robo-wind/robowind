#include <ccore.h>
#include <bson.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <cassert>
#include <ccore_struct.h>

using namespace ccore_struct;
using namespace std;
using namespace ccore_bson;

std::unique_ptr<ccore::message> lastmsg = nullptr;
std::unique_ptr<ccore::message> lastmsg2 = nullptr;
std::string log_path;
std::string log_service;
std::string log_message;
int64_t log_time;
uint8_t lastmsg_struct = 0;




void worker_task(const string& path, const document &request,
                 ccore::worker_send_results_f send_results)
{

    int arg1 = request["arg1"];
    int arg2 = request["arg2"];
    
    std::thread( [arg1, arg2, send_results] {
        bson response;

        std::cout << "Performing work" << std::endl;
        if (arg2 == 0)
        {
            send_results(false, response);
            return;   // Unable to complete request, division by zero
        }
        response["result"] = arg1 / arg2;
    
        send_results(true, response);
    }).detach();
};

void return_handler(task_status_t status, const document &response)
{
    if(status == TASK_SUCCESS)
    {
        std::cout << "Result is: " << (int)(response["result"]) << std::endl;
    }
    else
    {
        std::cout << "Error, status is: " << status  << std::endl;
    }
};




int
main (int argc, char *argv [])
{
    ccore::client client("ccore_task_example");
    
    client.add_task_hub("zmq-tcp://127.0.0.1");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Handle all requests (sequentially) for the next 10 seconds
    auto worker_id = client.register_task_handler("/calculator/div", worker_task);
    
    bson request;
    request["arg1"]  = 10;
    request["arg2"] =  2;
    
    // Issue a single request to be handled by the next available worker or send
    // timeout message after 5000ms if no worker available.
    while(true)
    {
        client.run_task("/calculator/div", request, return_handler, 5000);
        this_thread::sleep_for(chrono::milliseconds(100));

    }
    this_thread::sleep_for(chrono::seconds(20));


    return 0;

}

