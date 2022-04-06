#pragma once

#include <stdlib.h>
#include <functional>
#include <memory>
#include "ccore.h"
#include "ccore_msg_libbson.h"

namespace ccore
{
    typedef std::function<void(std::unique_ptr<msg_t> message, void* callback_data)> sub_callback_msg_f;
    
    // CommuniCore client
    //
    // Client allows users to send and receive Communicore messages using
    // libbson encoded messages.
    // This version of client provides background thread that monitors
    // subscribed sockets and notifies the user utlizing callbacks.
    class client_t : public client
    {
    public:
        // Subscribe to messages on path.
        //
        // @param path CommuniCore path to subscribe to. If path ends with "*",
        //  messages on subpaths will be subscribed to as well
        // @param callback Function to be called when message matching path is
        //  received.
        // @param callback_data Any data which should be passed to callback
        //  function
        // @return callback id which can be used remove specific subscription
        callback_id subscribe (const std::string &path, sub_callback_msg_f callback, void *callback_data = nullptr);
        
        // Send a message to path
        // @param path CommuniCore path to send the message to.
        // @param payload bson payload to send. It should include keys "path"
        //  which will also specify the path and "data" which will include the
        //  payload
        // @return If send was successful (In case of ZMQ, added to send buffer)
        bool send (const std::string &path, bson_t &payload);
        
        // Allow overload resolution to use base class methods.
        using client::send;
        using client::subscribe;
    };
    
}

