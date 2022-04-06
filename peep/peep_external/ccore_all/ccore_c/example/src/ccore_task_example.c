#include "ccore_c99.h"
#include <bson/bson.h>
#include <bson/bcon.h>

#ifdef _WIN32
#include <Windows.h>
#endif


void ccore_sleep_ms(unsigned int miliseconds)
{
#ifdef _WIN32
    Sleep(miliseconds);
#else
    usleep(1000 * miliseconds);
#endif
}
void
handle_ccore_msg(ccore_msg_t* msg, void* callback_data)
{
    char* result;
    result = bson_as_json(&msg->bson, NULL);
    //BCON_EXTRACT(msg->bson, "data", BCONE_UTF8(result));
    
    printf("Path %s, payload %s\n", msg->path, result);
    
    bson_free(result);
    ccore_msg_free(msg);
}


void div_worker(const char* path, const uint8_t* in_payload, size_t in_payload_size,
                 ccore_send_results_t* send_context, void* callback_data)
{
    
    bson_t doc;
    bson_init_static (&doc, in_payload, in_payload_size);
    
    int32_t arg0, arg1;
    BCON_EXTRACT (&doc, "arg0", BCONE_INT32(arg0), "arg1", BCONE_INT32(arg1));
    
    int32_t result = arg0/arg1;
    
    bson_t *bson_doc = BCON_NEW ("result", BCON_INT32(result));
    ccore_send_task_results(send_context, true, bson_get_data(bson_doc), bson_doc->len);
    bson_destroy(bson_doc);
    
}

struct thread_data
{
    bson_t* doc;
    ccore_send_results_t* send_context ;
};

static void*
thread_runner(void *arg)
{
    printf("In thread\n");
    ccore_sleep_ms(100);
    struct thread_data* data = (struct thread_data*) arg;
    
    int32_t arg0, arg1;
    BCON_EXTRACT (data->doc, "arg0", BCONE_INT32(arg0), "arg1", BCONE_INT32(arg1));
    
    int32_t result = arg0/arg1;
    
    bson_t *bson_doc = BCON_NEW ("result", BCON_INT32(result));
    ccore_send_task_results(data->send_context, true, bson_get_data(bson_doc), bson_doc->len);
    bson_destroy(bson_doc);
    bson_destroy(data->doc);
    free(data);
    printf("Out thread\n");

    return NULL;

}

void
div_thread_worker(const char* path, const uint8_t* in_payload, size_t in_payload_size,
                 ccore_send_results_t* send_context, void* callback_data)
{
    // in_payload is will get freed after this function exists.
    // Copy it to new bson in this thread, or extract it from existing
    // bson before you exit
    bson_t* doc = bson_new_from_data(in_payload, in_payload_size);
    
    struct thread_data* data = malloc(sizeof(struct thread_data));
    data->doc = doc;
    data->send_context = send_context;
    
    pthread_t t;
    pthread_create(&t, NULL, thread_runner, (void*) data);
    pthread_detach(&t);
}



void
customer_callback(int32_t status, const uint8_t* payload, size_t payload_size,
                       void* callback_data)
{
    if (status == TASK_SUCCESS)
    {
        bson_t doc;
        bson_init_static (&doc, payload, payload_size);
        int32_t result;
        BCON_EXTRACT (&doc, "result", BCONE_INT32(result));
        printf("Result is %d\n", result);
    }
    else
    {
        printf("Error Status is %d", status);

    }
}



int
main (int argc, char *argv [])
{
    ccore_t* client = ccore_new();
    ccore_add_task_hub(client, "zmq-tcp://sb-ml4.disid.disney.com");
    
    bson_t *bson_doc = BCON_NEW ("arg0", BCON_INT32(10),
                                 "arg1", BCON_INT32(5));
    
    ccore_register_task_handler(client, "/div", div_worker, NULL);
    ccore_register_task_handler(client, "/div_threaded", div_thread_worker, NULL);


    for(int i=0; i<50; i++)
    {
        ccore_sleep_ms(500);
        
    //    ccore_run_task(client, "/div", bson_get_data(bson_doc), bson_doc->len,
    //                   customer_callback, NULL, 30000, 5000);
        ccore_run_task(client, "/div_threaded", bson_get_data(bson_doc), bson_doc->len,
                       customer_callback, NULL, 30000, 5000);
        
    }
    
    ccore_sleep_ms(5000);
    
    ccore_destroy(&client);
    
    
}

