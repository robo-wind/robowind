#pragma once

#include <stdlib.h>
#include <memory>
#include "bson/bson.h"
#include "ccore_msg_libbson.h"
#include "ccore_poll.h"


namespace ccore
{
    // CommuniCore client
    //
    // Client allows users to send and receive Communicore messages using
    // libbson encoded messages.
    // This version of client should be using for applications that require
    // network to be polled at specific times (for example, inside of main loop)
    // or for applications that do not allow data to be received on external
    // threads
    class client_poll_t : public client_poll
    {
    public:
        // receive
        //
        // @param max_wait_ms maximum time in miliseconds that function will wait for data.
        //  specify -1 for no limit
        std::unique_ptr<msg_t> receive_bson_t(int max_wait_ms);
        
        // Send a message to path
        // @param path CommuniCore path to send the message to.
        // @param payload bson payload to send. It should include keys "path"
        //  which will also specify the path and "data" which will include the
        //  payload
        // @return If send was successful (In case of ZMQ, added to send buffer)
        bool send(const std::string &path, bson_t &payload);
        
        // Allow overload resolution to use base class methods.
        using client_poll::send;
    private:
        impl::client_poll_t* self;
    };
    
}
