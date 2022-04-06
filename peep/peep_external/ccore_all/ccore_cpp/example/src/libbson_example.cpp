#include <bson/bson.h>
#include <thread>
#include <chrono>
#include <iostream>


int
main (int argc, char *argv [])
{


   std::string text = R"(
    {
        "path": "/perception/vision/scenegraph",
        "data": {
            "_ts": 1522694009119373600,
            "_source_id": "world",
            "people": [
                       {
                       "pos": [
                               0.461,
                               -3.268,
                               1.253
                               ],
                       "heading": 180,
                       "id": 111,
                       "/track20lidar": 766,
                       "name": "???"
                       }
                       ]
        }
    }
    )";



    bson_t b;
    bson_error_t error;
    bool r;


    r = bson_init_from_json(&b, text.c_str(), text.size(), &error);

    if (!r)
    {
        std::cout << error.domain << " " << " " <<  error.code << " " << error.message;
    }

    const char* path;
    int64_t ts;
    const char* source_id;
    float pos_x;
    float pos_y;
    float pos_z;

    bson_t people;
    bson_iter_t people_iter;

    BCON_EXTRACT (&b,
                  "path", BCONE_UTF8(path),
                  "data", "{",
                    "_ts", BCONE_INT64(ts),
                    "_source_id", BCONE_UTF8(source_id),
                    "people", BCONE_ARRAY(people),
                  "}"
                  );

    bson_iter_init (&people_iter, &people);


    while (bson_iter_next (&people_iter)) {
        uint32_t doc_len;
        const uint8_t* doc;
        bson_t people_bson;

        bson_iter_document(&people_iter, &doc_len, &doc);
        bson_init_static(&people_bson, doc, doc_len);

        bson_t pos;
        double x, y, z;
        int32_t heading;
        int32_t id;
        int32_t lidar;
        const char* name;

         BCON_EXTRACT(&people_bson,
                         "pos", "[", BCONE_DOUBLE(x), BCONE_DOUBLE(y), BCONE_DOUBLE(z), "]",
                         "heading", BCONE_INT32(heading),
                         "id", BCONE_INT32(id),
                         "/track20lidar", BCONE_INT32(lidar),
                         "name", BCONE_UTF8(name)
                     );
    }

}

