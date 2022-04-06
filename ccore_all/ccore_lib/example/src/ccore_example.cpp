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

void
handle_test_msg(std::unique_ptr<ccore::message> msg, void* callback_data)
{

    ccore_bson::element data = msg->get_bson()["data"];
    if (data.get_type() == ccore_bson::bson_type::INT32)
    {
        std::cout << "1:" << msg->get_path() << ":" << (int) data << std::endl;
    }
    else if (data.get_type() == ccore_bson::bson_type::STRING)
    {
        std::cout << "1:" << msg->get_path() << ":" << (std::string) data << std::endl;
    }
    lastmsg = std::move(msg);
}


void
handle_test_msg2(std::unique_ptr<ccore::message> msg, void* callback_data)
{
    ccore_bson::element data = msg->get_bson()["data"];
    if (data.get_type() == ccore_bson::bson_type::INT32)
    {
        std::cout << "2:" << msg->get_path() << ":" << (int)data << std::endl;
    }
    else if (data.get_type() == ccore_bson::bson_type::STRING)
    {
        std::cout << "2:" << msg->get_path() << ":" << (std::string) data << std::endl;
    }
    lastmsg2 = std::move(msg);
}

constexpr size_t TEN_MB = 10*1024*1024;

struct some_struct {
    int32_t a, b;
    float vec[3];
    uint8_t big[TEN_MB];
};

void on_data_sent(void* structure)
{
    auto* s = (some_struct*)structure;
    s = s;
}

void
handle_struct_msg(const std::string& path, std::shared_ptr<uint8_t> payload,
                  size_t payload_size, void* callback_data)
{
    auto* received_data = struct_cast<some_struct>(path, payload);
    received_data = received_data;
    lastmsg_struct = received_data->big[TEN_MB - 1];
}

void
handle_log_msg(std::unique_ptr<ccore::message> msg, void* callback_data)
{
    auto &data = (const ccore_bson::document&) msg->get_bson()["data"];
    log_service = data["service"].get("");
    log_message = data["message"].get("");
    log_time = data["time"].get((int64_t)0);
    log_path = msg->get_path();
}

void log_test_and_reset(const std::string &level)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    assert(log_time > 0);
    assert(log_service == "ccore_example_test");
    assert(log_message == level);
    assert(log_path == std::string("/logging/project/") + level + "/ccore_example_test" );
    log_path = "";
    log_service = "";
    log_message = "";
    log_time = 0;
}


bool worker_task(const string& path, const document &request, document &response)
{
    int arg1 = request["arg1"];
    int arg2 = request["arg2"];
    if (arg2 == 0)
        return false;   // Unable to complete request, division by zero
    
    response["result"] = arg1 / arg2;
    return true;
};

void return_handler(task_status_t status, const document &response)
{
    if(status == TASK_SUCCESS)
    {
        std::cout << "Result is: " << (int)(response["result"]) << std::endl;
    }
};

int
main (int argc, char *argv [])
{
    ccore::client client("ccore_example_test");
    client.add_sub("zmq-tcp://*:15555");
    client.add_pub("zmq-tcp://127.0.0.1:15555");

    client.subscribe("/foo", handle_test_msg, NULL);
    client.subscribe("/foo*", handle_test_msg2, NULL);

    // subscription paths to remove later
    int sub_id1 = client.subscribe("/xxx/bar", handle_test_msg);
    int sub_id2 = client.subscribe("/xxx/bar", handle_test_msg2);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Logging
    {
        client.set_project_name("project");
        int log_sub = client.subscribe("/logging/project*", handle_log_msg, NULL);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        ccore::logger log(client);
        log.debug("debug"); log_test_and_reset("debug");
        log.info("info"); log_test_and_reset("info");
        log.warn("warn"); log_test_and_reset("warn");
        log.error("error"); log_test_and_reset("error");
        client.unsubscribe(log_sub);
    }

    // send string
    {
        std::string tdata = "should be received on /foo and /foo*";
        client.send("/foo", tdata);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg != nullptr);
        std::string data = lastmsg->get_bson()["data"];
        assert(tdata.compare(data) == 0);

        assert(lastmsg2 != nullptr);
        std::string data2 = lastmsg2->get_bson()["data"];
        assert(tdata.compare(data2) == 0);

        lastmsg = nullptr;
        lastmsg2 = nullptr;
    }
    // send integer
    {
        client.send("/foo", 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg != nullptr);
        int data = lastmsg->get_bson()["data"];
        assert(data == 1);

        lastmsg = nullptr;
        lastmsg2 = nullptr;
    }

    // not listening to path
    {
        client.send("/none", "gogo");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "Lastmsg should be null and that is " << (lastmsg == nullptr ? "true" : "false") << std::endl;
        assert(lastmsg == nullptr);
    }

    // test unsubscribing
    {
        std::string tdata = "should be seen in 1 and 2";
        client.send("/xxx/bar", tdata);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg != nullptr);
        std::string data = lastmsg->get_bson()["data"];
        assert(tdata.compare(data) == 0);

        assert(lastmsg2 != nullptr);
        std::string data2 = lastmsg2->get_bson()["data"];
        assert(tdata.compare(data2) == 0);

        lastmsg = nullptr;
        lastmsg2 = nullptr;
    }
    {
        client.unsubscribe(sub_id1);
        std::string tdata = "should be seen in 2 only";
        client.send("/xxx/bar", tdata);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg == nullptr);

        assert(lastmsg2 != nullptr);
        const std::string data = lastmsg2->get_bson()["data"];
        assert(tdata.compare(data) == 0);

        lastmsg2 = nullptr;
        
    }

    {
        // We don't currently support sending preformatted data
        // Functionality exists, but it is not exposed
        /*
        const ccore_bson::bson &bson = lastmsg2->get_bson();
        std::vector<uint8_t> data;
        bson.write_buffer(data);
        const uint8_t* data_ptr = data.data;
        size_t data_size = data.size();
        client.send("/xxx/bar", data_ptr, data_size);
        */
    }

    // ZMQ over ipc test (not suppored on Windows)

#ifndef _WIN32
    {
        ccore::client ipc_client;
        ipc_client.add_sub("zmq-ipc:///tmp/testipc*");
        ipc_client.add_pub("zmq-ipc:///tmp/testipc");
        ipc_client.subscribe("/ipc-testpath", handle_test_msg);
        ipc_client.subscribe("/ipc-testpath", handle_test_msg2);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::string tdata = "Should be seen on 1 and 2";

        ipc_client.send("/ipc-testpath", tdata);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg != nullptr);
        std::string data = lastmsg->get_bson()["data"];
        assert(tdata.compare(data) == 0);

        assert(lastmsg2 != nullptr);
        std::string data2 = lastmsg2->get_bson()["data"];
        assert(tdata.compare(data2) == 0);

        lastmsg = nullptr;
        lastmsg2 = nullptr;
    }

#endif // _WIN32

    // ZMQ over inproc test
    {
        ccore::client ipc_client;
        ipc_client.add_sub("zmq-inproc://test-endpoint*");
        ipc_client.add_pub("zmq-inproc://test-endpoint");
        ipc_client.subscribe("/ipc-testpath", handle_test_msg);
        ipc_client.subscribe("/ipc-testpath", handle_test_msg2);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::string tdata = "Should be seen on 1 and 2";

        ipc_client.send("/ipc-testpath", tdata);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg != nullptr);
        std::string data = lastmsg->get_bson()["data"];
        assert(tdata.compare(data) == 0);

        assert(lastmsg2 != nullptr);
        std::string data2 = lastmsg2->get_bson()["data"];
        assert(tdata.compare(data) == 0);

        lastmsg = nullptr;
        lastmsg2 = nullptr;
    }

    // Sending array over ccode
    {
        const std::vector<std::string> tdata{ "Sending Array test" };

        client.send("/foo", tdata);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg != nullptr);
        const std::vector<std::string> data = lastmsg->get_bson()["data"];
        
        assert(tdata == data);
        
        lastmsg = nullptr;
        lastmsg2 = nullptr;
        
    }

    // Sending another bson document as payload
    {
        ccore_bson::element bson_obj;
        ccore_bson::document& doc = bson_obj.make_document();
        const std::vector<std::string> string_array{ "Sending document test" };

        doc["array"] = string_array;

        client.send("/foo", bson_obj);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        assert(lastmsg != nullptr);
        const ccore_bson::document& data_doc = lastmsg->get_bson()["data"];
        const std::vector<std::string>& data_array = data_doc["array"];
        
        lastmsg = nullptr;
        lastmsg2 = nullptr;
    }

    // Sending struct data
    {
        auto ss_id = client.subscribe_struct("/struct", handle_struct_msg, NULL);
        auto* my_data = alloc<some_struct>("/struct", on_data_sent);
        // Fill with some arbitrary data
        my_data->a = 12;
        my_data->b = 34;
        my_data->vec[0] = 9;
        my_data->vec[1] = 8;
        my_data->vec[2] = 7;
        const uint8_t TEST_VALUE = 0xAA;
        my_data->big[TEN_MB - 1] = TEST_VALUE;
        
        for(int i=0; i<10; i++)
        {
            client.send_struct(my_data);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        dealloc(my_data);
        client.unsubscribe(ss_id);
        std::cout << "Received struct value should be 0xAA and that is " << (lastmsg_struct == TEST_VALUE ? "true" : "false") << std::endl;
        assert(lastmsg_struct == TEST_VALUE);
    }

    return 0;

}

