
//
//  Copyright © 2022 Robowind. All rights reserved.
//


#include "ccore_msg_libbson.h"

namespace ccore
{

    msg_t::msg_t(const std::string &path_, std::shared_ptr<uint8_t> payload_,
                 size_t payload_size_)
    {
        path = path_;
        // keep reference to payload and wrap it with
        // bson object.
        raw_payload = payload_;
        valid = bson_init_static(&bson, payload_.get(), payload_size_);
    }
    
    const std::string&
    msg_t::get_path() const
    {
        return path;
    }
    
    bson_t&
    msg_t::get_bson()
    {
        return bson;
    }
    
    bool
    msg_t::is_valid()
    {
        return valid;
    }
    
}
