#include <ccore.h>
#include <bson.h>
#include <chrono>
#include <thread>
#include <iostream>


/// In this example, we are using a loopback connection, where same client opens
/// a port and connects to it. This is rarely used in real world,
/// but provides a simple example.

// This handler will be called when we receive a message on
// subscribed path
void
handle_test_msg(std::unique_ptr<ccore::message> msg, void* callback_data)
{
    
    const ccore_bson::bson &doc = msg->get_bson();
    const ccore_bson::document &data = doc["data"];
    int32_t first = data["First key"];
    double second = data["Second key"];
    std::string  last  = data["Last key"];
    
    std::cout << "First key is: " << first << std::endl;
    
}


int
main (int argc, char *argv [])
{
    
    ccore::client client("ccore_example_test");
    
    /// This is Subscriber side
    //
    // Connect a subscriber to above port. It does not matter that nothing
    // is listening on that port yet. Client will connect once Publisher
    // opens the port
    
    client.add_sub("zmq-tcp://*:15555");
    client.subscribe("/foo", handle_test_msg, NULL);
    
    /// This is Publisher side
    //
    // Bind a publisher to socket 5555 on localhost
    client.add_pub("zmq-tcp://127.0.0.1:15555");
    
    // Wait for connection to be established and subscription
    // information to be exchanged
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Construct and send bson object.
    ccore_bson::element bson_obj;
    ccore_bson::document& doc = bson_obj.make_document();
    doc["First key"]  = 42;
    doc["Second key"] = 3.14159;
    doc["Last key"]   = "That's all!";
    
    client.send("/foo", bson_obj);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
