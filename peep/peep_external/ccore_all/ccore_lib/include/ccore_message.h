#pragma once

#include <stdlib.h>
#include <cstdint>
#include <string>
#include <memory>
#include "bson.h"

namespace ccore
{
    
    /// CommuniCore message
    ///
    /// Class holds CommuniCore data in decoded (bson) and raw byte format.
    /// During destruction, both are freed.
    class message
    {
    public:
        /// msg_bson_t constuctor
        ///
        /// @param path CommuniCore network path
        /// @param payload Message received in binary format
        /// @param payload_size Size of binary message
        message(const std::string &path, std::shared_ptr<uint8_t> payload, size_t payload_size);
        
        /// Get message path
        const std::string& get_path() const;
        /// Get message data
        const ccore_bson::bson& get_bson() const;
        /// Is message valid.
        /// @return True if payload could be parsed to a BSON object
        bool is_valid() const;
        
    private:
        std::string path;
        std::shared_ptr<uint8_t> raw_payload;
        ccore_bson::bson bson;
        bool valid;

    };
}

