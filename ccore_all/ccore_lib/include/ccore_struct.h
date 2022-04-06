#pragma once

#include <stdlib.h>
#include <cstdint>
#include <string>
#include <memory>
#include <functional>
#include "bson.h"

namespace ccore_struct
{
    /// Callback indicating when it is safe to access data, i.e. after send is complete.
    using struct_send_callback_f = std::function<void(void* data)>;

    extern void* alloc_internal(const char*, size_t, struct_send_callback_f);
    extern void dealloc_internal(void*);

    /// Allocates and returns a new structure of type T.
    /// @param path Path that future messages of this type should be sent to
    /// @param callback_on_send Whenever ccore is done sending a message of this type, the
    /// callback function will be invoked. At that point it is safe to deallocate the instance
    /// or modify the contained data.
    /// Usage:
    /// my_type* data = ccore_struct::alloc<my_type>(path, callback);
    /// client.send_struct(data);
    template <typename T>
    T* alloc(const char* path, struct_send_callback_f callback_on_send)
    {
        return (T*) alloc_internal(path, sizeof(T), callback_on_send);
    }
    
    /// Frees a structure previously created with ccore_struct::alloc
    /// Must not be used while a send call is pending and callback_on_send has not been called.
    template <typename T>
    void dealloc(T* userdata)
    {
        dealloc_internal((void*) userdata);
    }
    
    /// Given a payload returns its associated structure of type T.
    /// Reminder: the returned pointer's lifetime is identical to that of the payload shared_ptr
    /// Once payload goes out of scope the returned pointer should no longer be used!
    /// Usage:
    /// void received(const std::string& path, shared_ptr<uint8_t> payload,
    ///               size_t ignore1, void* ignore2)
    /// {
    ///    my_type* data = struct_cast<my_type>(path, payload);
    ///    cout << data->var;
    /// }
    template <typename T>
    T* struct_cast(const std::string& path, const std::shared_ptr<uint8_t> &payload)
    {
        uint8_t* ptr = payload.get();
        
        const char bson_template[] =
        "0000" /* int32_t total_size */
        "\2path\0"
        "1111" /* int32_t path_len_excluding_zero */
        ""  /* path characters */
        "\0" /* path terminator */
        "\5data\0"
        "2222" /* int32_t binary_size */
        "\x80";
        
        ptr += sizeof(bson_template) - 1 + path.length();
        return reinterpret_cast<T*>(ptr);
    }

}

