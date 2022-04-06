//
//  Copyright © 2018 Robowind. All rights reserved.
//
#pragma once
#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include "ccore_common.h"
#include "ccore_message.h"
#include "ccore_struct.h"
#include "bson.h"

namespace ccore
{
    namespace impl
    {
        class client_callback_t;
        
        // Callback for data received from ZMQ. Each callback will include message path and payload,
        // along with callback_data passed during ccore_subscribe
        using ccore_internal_callback_f = std::function<void(const std::string& path,
                                                             std::shared_ptr<uint8_t> payload,
                                                             size_t payload_size,
                                                             void* callback_data)>;

        // Callback for task workers
        
        using worker_internal_send_results_f = std::function<void(bool success,
                                                         const uint8_t* payload,
                                                         size_t payload_size)>;
        
        // Callback for task workers
        using worker_internal_callback_f =
            std::function<void(const std::string &path,
                           const std::shared_ptr<uint8_t> in_payload,
                           const size_t in_payload_size,
                           worker_internal_send_results_f send_results)>;

        // Callback for task customer
        using customer_internal_callback_f = std::function<void(task_status_t status,
            std::shared_ptr<uint8_t> payload,
            size_t payload_size)>;
    }
    
    typedef int callback_id;
    using sub_callback_bson_f = std::function<void(std::unique_ptr<message> message,
                                                   void* callback_data)>;
    
    using worker_send_results_f =  std::function<void(bool success,
                                                      ccore_bson::bson &output)>;
    
    using worker_callback_f = std::function<void (const std::string& path,
                                                  const ccore_bson::document &input,
                                                  worker_send_results_f send_results
                                                  )>;
    
    using customer_callback_f = std::function<void (task_status_t status,
                                                     const ccore_bson::document &result)>;
    
    
    struct callback_bson_t;
    struct callback_worker_t;
    class  callback_customer_t;
    
    /// CommuniCore base client
    ///
    /// Client allows users to send and receive Communicore message using
    /// pre-encoded data. It should be used for language bindings or clients
    /// that have pre-encoded message.
    ///
    /// This version of client provides background thread that monitors
    /// subscribed sockets and notifies the user utlizing callbacks.
    class client
    {
    public:
        /// Create a CommuniCore client
        ///
        /// @param client_name Optional client name
        client(const std::string& client_name = "");
        
        /// Client destructor
        ~client();
        
        /// Connect to a publish socket
        ///
        /// @param pub_uri URI of publish socket for example "zmq-tcp://127.0.0.1:5555"
        /// @return        If URI is of valid format. This will not return false if pier
        ///                is not available, only if format is incorrect.
        bool add_pub(const std::string &pub_uri);
        
        /// Connect to a subscribe socket
        ///
        /// @param sub_uri URI of subscribe socket for example "zmq-tcp://127.0.0.1:5556"
        /// @return        If URI is of valid format. This will not return false if pier
        ///                is not available, only if format is incorrect.
        bool add_sub(const std::string &sub_uri);

        /// Remove subscription
        ///
        /// @param subscription_id id of subcription to remove
        /// @return                true if subscription_id was found & removed
        bool unsubscribe(callback_id subscription_id);
    
        /// Send a message to path
        ///
        /// @param path    CommuniCore path to send the message to.
        /// @param payload bson payload to send. Both payload and path will automatically be added
        ///                to a BSON document to be sent over the wire.
        /// @return        If send was successful (In case of ZMQ, added to send buffer)
        bool send (const std::string &path, const ccore_bson::element &payload);
        
        /// Subscribe to a path
        ///
        /// @param path          CommuniCore path to subscribe to. If path ends with "*",
        ///                      messages on subpaths will be subscribed to as well
        /// @param callback      Function to be called when message matching path is
        ///                      received.
        /// @param callback_data Any data which should be passed to callback
        ///                      function
        /// @return              callback id which can be used remove specific subscription
        callback_id subscribe (const std::string &path, sub_callback_bson_f callback, void *callback_data = nullptr);
        
        /// Set watchdog state for future add_sub calls.
        ///
        /// @param enable Enable/disable watchdog usage
        void enable_watchdog(bool enable = true);
        
        /// Start sending periodic health updates
        /// @param time_ms Number of milliseconds between health updates
        void health_start(int32_t time_ms = CCORE_DEFAULTS_TIMER_SECONDS * 1000);
        /// Stop sending periodic health updates
        void health_stop();
        
        /// All future health reports will include the status level and custom message
        ///
        /// @param status  Coarse status indicator - Green, Yellow, or Red enum
        /// @param message Optional status message
        void set_health (health_status_t status, const std::string &message = "");
        
        /// All future health reports will include this extra bson data in user_data
        ///
        /// @param bsonData ccore bson document
        void set_health_userdata (ccore_bson::document &&bsonData);
        /// All future health reports will include this extra bson data in user_data
        ///
        /// @param bsonData binary BSON data
        /// @param size     number of bytes of binary BSON data
        void set_health_userdata (const uint8_t* bsonData, int32_t size);

        /// Set optional name used in the logging path
        void set_project_name(const std::string &project);
        
        /// Send a log message to subscribers
        ///
        /// @param level   One of the predefined log levels
        /// @param message String to send as part of the log packet
        bool log(log_level_t level, const std::string &message);
        
        /// Send a pre-allocated structure
        bool send_struct(void* structure);
        
        /// Subscribe to a path, expecting a struct as data value
        /// @param path          CommuniCore path to subscribe to. If path ends with "*",
        ///                      messages on subpaths will be subscribed to as well
        /// @param callback      Function to be called when message matching path is
        ///                      received.
        /// @param callback_data Any data which should be passed to callback
        ///                      function
        /// @return              callback id which can be used remove specific subscription
        callback_id subscribe_struct(const std::string &path,
                                     impl::ccore_internal_callback_f callback,
                                     void* callback_data);
        
        /// Connect to a task hub socket. Same socket is used for both customer and worker
        ///
        /// @param task_hub_uri URI of tash hub socket for example "zmq-tcp://127.0.0.1:5561"
        /// @return             If URI is of valid format. This will not return false if pier
        ///                     is not available, only if format is incorrect.
        bool add_task_hub(const std::string& task_hub_uri);
        
        /// Run a task on remote machine.
        ///
        /// @param path          Function name
        /// @param args          Function arguments
        /// @param callback      Callback which gets called once call completes or fails
        /// @param timeout       If worker does don't complete task in specified number of
        ///                      miliseconds, callback will be called with TASK_TIMEOUT
        ///                      error
        /// @param start_timeout If worker does not start the task in specified number of
        ///                      miliseconds, callback will be called with TASK_UNAVAILABLE
        void run_task(const std::string &path, const ccore_bson::bson &args,
                      customer_callback_f callback,
                      uint32_t timeout = CCORE_DEFAULTS_TASK_TIMEOUT * 1000,
                      uint32_t start_timeout = CCORE_DEFAULTS_TASK_START_TIMEOUT * 1000);

        /// Register a task handler on worker
        ///
        /// @param path          Function name
        /// @param callback      Callback to call once task is completed
        /// @return              Task handler id. Required for task handler cancelation.
        uint32_t register_task_handler(const std::string &path, worker_callback_f callback);
        
    protected:
        /// Subscribe to a path, expecting a raw buffer. Used by SDKs which do their own
        /// conversion to BSON
        /// @param path          CommuniCore path to subscribe to. If path ends with "*",
        ///                      messages on subpaths will be subscribed to as well
        /// @param callback      Function to be called when message matching path is
        ///                      received.
        /// @param callback_data Any data which should be passed to callback
        ///                      function
        /// @return              callback id which can be used remove specific subscription
        callback_id subscribe(const std::string &path, impl::ccore_internal_callback_f callback,
                              void* callback_data);
        /// Send a message to path
        ///
        /// @param path          CommuniCore path to send the message to.
        /// @param payload       BSON payload to send. Buffer should contain serialized BSON
        ///                      object.
        /// @param payload_size  Payload size.
        /// @return        If send was successful (In case of ZMQ, added to send buffer)
        bool send(const std::string &path, const uint8_t* payload, size_t payload_size);
        
        void run_task(const std::string &path, const uint8_t* payload, size_t payload_size,
                     impl::customer_internal_callback_f callback,
                      uint32_t timeout = CCORE_DEFAULTS_TASK_TIMEOUT * 1000,
                      uint32_t start_timeout = CCORE_DEFAULTS_TASK_START_TIMEOUT * 1000);
        uint32_t register_task_handler(const std::string &path,
                                       impl::worker_internal_callback_f callback);

    private:
        impl::client_callback_t* self;
        std::vector<uint8_t> send_buffer;
        
        std::vector<callback_bson_t*> registered_callbacks;
    };
    
    /// Convenience functions for logging. Usage:
    /// ccore::logger log(my_client);
    /// log.error("Something went wrong!");
    class logger {
    public:
        /// Create a logger
        logger(client &ccore_client) : ccore_client(ccore_client) {}
        /// Send a debug message
        /// @param message Message to send
        void debug(const std::string &message) const { ccore_client.log(CCORE_DEBUG, message); }
        /// Send an info message
        /// @param message Message to send
        void info(const std::string &message) const { ccore_client.log(CCORE_INFO, message); }
        /// Send a warning message
        /// @param message Message to send
        void warn(const std::string &message) const { ccore_client.log(CCORE_WARN, message); }
        /// Send an error message
        /// @param message Message to send
        void error(const std::string &message) const { ccore_client.log(CCORE_ERROR, message); }

    private:
        client &ccore_client;
    };
}
