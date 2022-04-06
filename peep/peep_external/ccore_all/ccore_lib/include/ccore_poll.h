//
//  Copyright © 2018 Robowind. All rights reserved.
//
#pragma once
#include <cstdint>
#include <stdlib.h>
#include <string>
#include <memory>
#include "ccore_common.h"
#include "ccore_message.h"
#include "bson.h"

namespace ccore
{
    namespace impl
    {
        class client_poll_t;
    }
    
    /// CommuniCore base client
    ///
    /// Client allows users to send and receive Communicore message using
    /// pre-encoded data. It should be used for language bindings or clients
    /// that have pre-encoded message.
    ///
    /// This version of client should be using for applications that require
    /// network to be polled at specific times (for example, inside of main loop)
    /// or for applications that do not allow data to be received on external
    /// threads
    class client_poll
    {
    public:
        /// Create a polling CommuniCore client
        /// @param client_name Optional client name
        client_poll(const std::string& client_name = "");
        ~client_poll();
        
        /// Connect to a publish socket
        ///
        /// @param pub_uri URI of publish socket
        ///   for example "zmq-tcp://127.0.0.1:5555"
        /// @return if URI is of valid format. This will not return false if pier
        ///  is not available, only if format is incorrect.
        bool add_pub(const std::string &pub_uri);
        
        /// Connect to a subscribe socket
        ///
        /// @param sub_uri URI of subscribe socket
        ///   for example "zmq-tcp://127.0.0.1:5556"
        /// @return if URI is of valid format. This will not return false if pier
        ///  is not available, only if format is incorrect.
        bool add_sub(const std::string &sub_uri);
        
        /// Subscribe to messages on path.
        ///
        /// @param path CommuniCore path to subscribe to. Subpaths will be included.
        void subscribe(const std::string &path);
        
        /// Unsubscribe to messages on path. If path was subscribed to multiple times,
        /// it will have to be unsubscribed to multiple times.
        ///
        /// @param path CommuniCore path to unsubscribe from. Subpaths will be included.
        void unsubscribe(const std::string &path);

        /// Send a message to path
        /// @param path CommuniCore path to send the message to.
        /// @param payload bson payload to send. Both payload and path will
        /// automatically be added to a BSON document to be sent over the wire.
        /// @return If send was successful (In case of ZMQ, added to send buffer)
        bool send (const std::string &path, const ccore_bson::element &payload);

        /// receive
        ///
        /// @param max_wait_ms maximum time in miliseconds that function will wait for data.
        ///  specify -1 for no limit
        std::unique_ptr<message> receive(int max_wait_ms);
        
        /// Set watchdog state for future add_sub calls.
        /// @param enable Enable/disable watchdog usage
        void enable_watchdog(bool enable = true);
        
        /// Start sending periodic health updates
        /// @param time_ms Number of milliseconds between health updates
        void health_start(int32_t time_ms = CCORE_DEFAULTS_TIMER_SECONDS * 1000);
        /// Stop sending periodic health updates
        void health_stop();
        
        /// All future health reports will include the status level and custom message
        /// @param status Coarse status indicator - Green, Yellow, or Red enum
        /// @param message Optional status message
        void set_health (health_status_t status, const std::string &message = "");
        
        /// All future health reports will include this extra bson data in user_data
        /// @param bsonData CCore BSON document
        void set_health_userdata (ccore_bson::document &&bsonData);
        /// All future health reports will include this extra BSON data in user_data
        /// @param bsonData Binary BSON data
        /// @param size Number of bytes of binary BSON data
        void set_health_userdata (const uint8_t* bsonData, int32_t size);
        
        /// Optional name used in the logging path
        void set_project_name(const std::string &project);
        
        /// Send a log message to subscribers
        /// @param level One of the predefined log levels
        /// @param message String to send as part of the log packet
        bool log(log_level_t level, const std::string &message);
        
        /// Send a pre-allocated structure
        /// @param structure Instance of data allocated with ccore_struct::alloc()
        bool send_struct(void* structure);
    protected:
        bool receive(std::string &path, std::shared_ptr<uint8_t> &payload, size_t &payload_size, int max_wait);
        int send(const std::string &path, const uint8_t* payload, size_t payload_size);
        
    private:
        impl::client_poll_t* self;
        std::vector<uint8_t> send_buffer;
    };
    
}
