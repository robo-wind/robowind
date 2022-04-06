
//
//  Copyright © 2022 Robowind. All rights reserved.
//


#include "ccore_poll_libbson.h"
#include "ccore_msg_libbson.h"


namespace ccore
{
    
    std::unique_ptr<msg_t>
    client_poll_t::receive_bson_t(int max_wait)
    {
        std::string path;
        std::shared_ptr<uint8_t> payload;
        size_t payload_size;
        
        bool ret = client_poll::receive(path, payload, payload_size, max_wait);
        if (ret == false)
        {
            return nullptr;
        }
        
        return std::unique_ptr<msg_t>(new msg_t(path, payload, payload_size));
    }
    
    bool
    client_poll_t::send(const std::string &path, bson_t &payload)
    {
        return this->client_poll::send(path, bson_get_data(&payload), payload.len);

    }

    
}


