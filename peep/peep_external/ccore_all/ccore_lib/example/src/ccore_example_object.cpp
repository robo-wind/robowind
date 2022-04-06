#include <ccore.h>
#include <bson.h>
#include <thread>
#include <chrono>
#include <iostream>


struct payload: public ccore_bson::serializable
{
    int32_t number;
    std::string text;
    std::shared_ptr<uint8_t> buffer;
    size_t buffer_size;
    
    void serialize(ccore_bson::document &doc) const override
    {
        doc["number"]    = number;
        doc["text"]      = text;
        
        ccore_bson::shared_buffer buff(buffer, buffer_size);
        doc["buffer"]    = buff;
    }
    
    void deserialize(const ccore_bson::document &doc) override
    {
        number = doc["number"];
        text   = (std::string) doc["text"];
        ccore_bson::shared_buffer buff = doc["buffer"];
        buffer = buff.ptr();
        buffer_size = buff.size();
        
        
    }
};

void
handle_test_msg(std::unique_ptr<ccore::message> msg, void* callback_data)
{
    payload p;
    msg->get_bson()["data"].deserialize(p);
    
    std::string buffer_string(p.buffer.get(), p.buffer.get() + p.buffer_size);
    
    std::cout << "1:" << msg->get_path() << " [" << p.number << "]"
        << p.text << ", " << buffer_string << std::endl;
}


void
handle_test_msg2(std::unique_ptr<ccore::message> msg, void* callback_data)
{
    payload p;
    msg->get_bson()["data"].deserialize(p);
    std::cout << "2:" << msg->get_path() << " [" << p.number << "]"
        << p.text << ", " << p.buffer_size << std::endl;
}

int
main (int argc, char *argv [])
{
    ccore::client client;
    client.add_pub("zmq-tcp://127.0.0.1:15555");
    client.add_sub("zmq-tcp://*:15555");
    
    client.subscribe("/test/object", handle_test_msg, NULL);
    client.subscribe("/test/object", handle_test_msg2, NULL);
    
    payload p;
    p.number = 100;
    p.text = "Number is 100";
    p.buffer = std::shared_ptr<uint8_t>((uint8_t *) "deadbeef", [](uint8_t*){});
    p.buffer_size = 8;
    
    for(int i=0; i<5; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Send" << std::endl;
        client.send("/test/object", p);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

