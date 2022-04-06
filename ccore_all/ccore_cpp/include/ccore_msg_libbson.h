#pragma once

#include <stdlib.h>
#include <cstdint>
#include <memory>
#include <string>
#include "bson/bson.h"

namespace ccore
{
    
    // CommuniCore message
    //
    // Class holds CommuniCore data in decoded (bson) and raw byte format.
    // During destruction, both are freed.
    class msg_t
    {
    public:
        // msg_t constuctor
        //
        // @param path CommuniCore network path
        // @param payload message received in binary format
        // @param payload_size size of binary message
        msg_t(const std::string &path, std::shared_ptr<uint8_t> payload, size_t payload_size);
      
        const std::string& get_path() const;
        bson_t& get_bson();
        bool is_valid();
       
    private:
        std::string path;
        std::shared_ptr<uint8_t> raw_payload;
        bson_t bson;
        bool valid;
    };
}

