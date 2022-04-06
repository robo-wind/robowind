
//
//  Copyright © 2022 Robowind. All rights reserved.
//


#include "ccore_libbson.h"
#include "ccore_msg_libbson.h"

namespace ccore
{
    
    struct callback_info_t
    {
        sub_callback_msg_f callback;
        void *callback_data;
    };

    void
    _handle_callback(const std::string &path, std::shared_ptr<uint8_t> payload, const size_t payload_size, void* callback_data)
    {
        callback_info_t* info = (callback_info_t*) callback_data;
        
        msg_t* msg = new msg_t(path, payload, payload_size);
        info->callback(std::unique_ptr<msg_t>(msg), info->callback_data);
    }

    int
    client_t::subscribe (const std::string &path, sub_callback_msg_f callback, void *callback_data)
    {
        // Todo: we are leaking callback_info. Free on unsubscribe
        callback_info_t* info = new callback_info_t();
        info->callback = callback;
        info->callback_data = callback_data;
        
        return this->client::subscribe(path, _handle_callback, (void*)info);
    }

    //
    bool
    client_t::send(const std::string &path, bson_t &payload)
    {
        return this->client::send(path, bson_get_data(&payload), payload.len);
    }
}
