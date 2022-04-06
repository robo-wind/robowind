#define UDP_SUPPORT

#ifndef _WIN32
#include <sys/resource.h>
#endif
#include <string>
#include <thread>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <ccore.h>
#include "clara.hpp"
#include <climits>
#include <cstring>

const char* PATH = "/t";
const char* SUMMARY_PATH = "/summary";
const char* PACKETS_SENT = "packetsSent";

using namespace std;
using namespace std::chrono;

// Command-line arguments:
int8_t      verbosity = 0;
bool        isUdp = false;
bool        isStruct = false;
bool        isListener = false;
uint64_t    delayusec = 100 * 1000;
uint64_t    dataKb = 1;
int         durationSec = 10;
string      uri;

// Memory usage at startup time
long baseMemoryKb;
// Total number of packets sent / received
uint64_t packetsSent = 0;
uint64_t packetsReceived = 0;

// UDP buffer size is limited to 8kB
struct StructSampleData {
    uint8_t data[7*1024];
};
struct StructSummaryData {
    uint64_t packetsSent;
};

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

TimePoint startTimer()
{
    return high_resolution_clock::now();
}

uint64_t usecElapsed(const TimePoint &start)
{
    auto end = high_resolution_clock::now();
    return duration_cast<microseconds>(end - start).count();
}

uint64_t msecElapsed(const TimePoint &start)
{
    auto end = high_resolution_clock::now();
    return duration_cast<milliseconds>(end - start).count();
}

uint64_t secElapsed(const TimePoint &start)
{
    auto end = high_resolution_clock::now();
    return duration_cast<seconds>(end - start).count();
}

void printLogo()
{
    cout <<
    " ▒▒▒▒▒▒╗ ▒▒▒▒▒▒╗ ▒▒▒╗   ▒▒▒╗▒▒▒╗   ▒▒▒╗▒▒╗   ▒▒╗▒▒▒╗   ▒▒╗▒▒╗ ▒▒▒▒▒▒╗ ▒▒▒▒▒▒╗ ▒▒▒▒▒▒╗ ▒▒▒▒▒▒▒╗\n"
    "▒▒╔════╝▒▒╔═══▒▒╗▒▒▒▒╗ ▒▒▒▒║▒▒▒▒╗ ▒▒▒▒║▒▒║   ▒▒║▒▒▒▒╗  ▒▒║▒▒║▒▒╔════╝▒▒╔═══▒▒╗▒▒╔══▒▒╗▒▒╔════╝\n"
    "▒▒║     ▒▒║   ▒▒║▒▒╔▒▒▒▒╔▒▒║▒▒╔▒▒▒▒╔▒▒║▒▒║   ▒▒║▒▒╔▒▒╗ ▒▒║▒▒║▒▒║     ▒▒║   ▒▒║▒▒▒▒▒▒╔╝▒▒▒▒▒╗\n"
    "▒▒║     ▒▒║   ▒▒║▒▒║╚▒▒╔╝▒▒║▒▒║╚▒▒╔╝▒▒║▒▒║   ▒▒║▒▒║╚▒▒╗▒▒║▒▒║▒▒║     ▒▒║   ▒▒║▒▒╔══▒▒╗▒▒╔══╝\n"
    "╚▒▒▒▒▒▒╗╚▒▒▒▒▒▒╔╝▒▒║ ╚═╝ ▒▒║▒▒║ ╚═╝ ▒▒║╚▒▒▒▒▒▒╔╝▒▒║ ╚▒▒▒▒║▒▒║╚▒▒▒▒▒▒╗╚▒▒▒▒▒▒╔╝▒▒║  ▒▒║▒▒▒▒▒▒▒╗\n"
    " ╚═════╝ ╚═════╝ ╚═╝     ╚═╝╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝ ╚═════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝\n";
}

// Maximum resident set size in kB.
long getMemoryUsage()
{
#ifndef _WIN32
    struct rusage usage;
    if(0 == getrusage(RUSAGE_SELF, &usage))
        return usage.ru_maxrss / 1024; // on OSX this is in bytes. Linux claims it's kbyte!
    else
        return 0;
#else
    return 0;
#endif
}

void printMemoryUsage()
{
    // Max RSS value is not easy to interpret. It might show a steady increase even when memory is properly
    // allocated and freed. Disabling logs for now.
    //    cout << endl << "Memory high watermark : " << getMemoryUsage()-baseMemoryKb << " kB" << endl;
}

inline void printTick()
{
    if (verbosity >= 2)
    {
        cout << ".";
        std::flush(cout);
    }
}

void OnStructDataSent(void* data)
{
    ; // nop
}

void makeSender()
{
    string addr = "zmq-tcp://";
    if (isUdp)
    {
        addr = "zmq-udp://";
        dataKb = 7; // fixed allocation size for UDP
    }
    if (isStruct)
    {
        dataKb = 7; // fixed allocation size for structured data
        cout << "core_struct  : True" << endl;
    }
    
    addr = addr + uri;
    cout << "Destination  : " << addr << endl;
    cout << "Data size    : ~ " << dataKb << " kB" << endl;
    cout << "Packet delay : " << delayusec << " µs" << endl;
    
    ccore::client client;
    client.add_pub(addr);
    
    // Allocate a bson message with given amount of data
    auto buffer = ccore_bson::shared_buffer(dataKb * 1024);
    auto bson = ccore_bson::element(buffer);
    
    // Send statistics at the end of run
    ccore_bson::element bsonSummary;
    auto& summaryDoc = bsonSummary.make_document();
    summaryDoc[PACKETS_SENT] = packetsSent;
    
    uint64_t minSendTime = std::numeric_limits<uint64_t>::max();
    uint64_t maxSendTime = std::numeric_limits<uint64_t>::min();
    uint64_t totalSendTime = 0;
    
    // Wait for subscription to happen
    std::this_thread::sleep_for(seconds(1));
    
    auto reportTime = startTimer();
    TimePoint startTime;
    StructSampleData* sampleData = nullptr;
    StructSummaryData* summaryData = nullptr;
    
    if (!isStruct)
    {
        // Standard bson
        startTime = startTimer();

        while (secElapsed(startTime) < durationSec)
        {
            std::this_thread::sleep_for(microseconds(delayusec));
            auto sendStart = startTimer();
            client.send(PATH, bson);
            uint64_t sendTime = usecElapsed(sendStart);
            totalSendTime += sendTime;
            minSendTime = std::min(minSendTime, sendTime);
            maxSendTime = std::max(maxSendTime, sendTime);
            
            ++packetsSent;
            if (verbosity >= 1)
            {
                // Print memory every 11 seconds
                if (secElapsed(reportTime) >= 11)
                {
                    printMemoryUsage();
                    std::flush(cout);
                    reportTime = startTimer();
                }
                printTick();
            }
        }
    } else {
        sampleData = ccore_struct::alloc<StructSampleData>(PATH, OnStructDataSent);
        summaryData = ccore_struct::alloc<StructSummaryData>(SUMMARY_PATH,
                                                          [](void *p) {
                                                              ccore_struct::dealloc(p);
                                                          }
                                                          );
        startTime = startTimer();

        // ccore_struct send
        while (secElapsed(startTime) < durationSec)
        {
            std::this_thread::sleep_for(microseconds(delayusec));
            auto sendStart = startTimer();
            client.send_struct(sampleData);
            uint64_t sendTime = usecElapsed(sendStart);
            totalSendTime += sendTime;
            minSendTime = std::min(minSendTime, sendTime);
            maxSendTime = std::max(maxSendTime, sendTime);
            
            ++packetsSent;
            if (verbosity >= 1)
            {
                // Print memory every 11 seconds
                if (secElapsed(reportTime) >= 11)
                {
                    printMemoryUsage();
                    std::flush(cout);
                    reportTime = startTimer();
                }
                printTick();
            }
        }
    }
    uint64_t elapsedTime = usecElapsed(startTime);
    
    // Send statistics to listener after brief pause
    std::this_thread::sleep_for(seconds(2));
    if (!isStruct)
    {
        summaryDoc[PACKETS_SENT] = packetsSent;
        client.send(SUMMARY_PATH, bsonSummary);
    } else {
        summaryData->packetsSent = packetsSent;
        client.send_struct(summaryData);
        ccore_struct::dealloc(sampleData);
    }
    
    const float USEC_TO_SEC = 1e-6;
    const float KB_TO_MB = 1 / 1024.0f;
    const uint64_t HEADER_SIZE = 29;
    const uint64_t kbytes = (packetsSent * (1024 * dataKb + HEADER_SIZE)) / 1024;
    float mbSec = (kbytes * KB_TO_MB) / (totalSendTime * USEC_TO_SEC);
    float packetsSec = (packetsSent) / (elapsedTime * USEC_TO_SEC);
    
    cout << endl;
    cout << "Min send time: " << minSendTime << " µs" << endl;
    cout << "Max send time: " << maxSendTime << " µs" << endl;
    cout << "Throughput   : " << mbSec << " MiB/sec (excl. delay)" << endl;
    cout << "Packet rate  : " << packetsSec << " packets/sec (incl. " <<
    delayusec << " µs delay)" << endl;
    cout << "Packets sent : " << packetsSent << endl;
}

void makeListener()
{
    // Bind with port number
    string addr = "zmq-tcp://*:";
    if (isUdp)
    {
        addr = "zmq-udp://*:";
        dataKb = 7; // fixed allocation size for UDP
    }
    if (isStruct)
    {
        dataKb = 7; // fixed allocation size for structured data
        cout << "core_struct  : True" << endl;
    }
    
    addr = addr + uri;

    cout << "Listening at " << addr << endl;
    
    ccore::client client;
    client.add_sub(addr);
    
    auto reportTime = startTimer();

    int summary_id;
    int sub_id;
    if (!isStruct)
    {
        // Standard BSON
        summary_id = client.subscribe(SUMMARY_PATH,
                                           [](std::unique_ptr<ccore::message> message, void* callback_data)
                                           {
                                               // Sender is done - retrieve statistics
                                               const ccore_bson::document &data = message->get_bson()["data"];
                                               packetsSent = data[PACKETS_SENT];
                                               if (verbosity >= 1)
                                               {
                                                   cout << endl << "Sender finished." << endl;
                                               }
                                           });
        
        sub_id = client.subscribe(PATH,
                                       [](std::unique_ptr<ccore::message> message, void* callback_data)
                                       {
                                           ++packetsReceived;
                                           
                                           if (verbosity >= 2)
                                           {
                                               cout << "O";
                                               std::flush(cout);
                                           }
                                       });
    } else {
        // ccore_struct data
        summary_id =
        client.subscribe_struct(SUMMARY_PATH,
                                [](const std::string& path,
                                   std::shared_ptr<uint8_t> payload,
                                   size_t payload_size, void* callback_data)
                                {
                                    using namespace ccore_struct;
                                    auto *p = struct_cast<StructSummaryData>(path, payload);
                                    packetsSent = p->packetsSent;
                                    if (verbosity >= 1)
                                    {
                                        cout << endl << "Sender finished." << endl;
                                    }
                                }, nullptr);

        sub_id =
        client.subscribe_struct(PATH,
                                [](const std::string& path,
                                   std::shared_ptr<uint8_t> payload,
                                   size_t payload_size, void* callback_data)
                                {
                                    ++packetsReceived;
                                    
                                    if (verbosity >= 2)
                                    {
                                        cout << "O";
                                        std::flush(cout);
                                    }
                                }, nullptr);
    } // if !isStruct

    auto timer = startTimer();
    while (secElapsed(timer) < durationSec)
    {
        std::this_thread::sleep_for(seconds(1));
        if (verbosity >= 1)
        {
            // Print memory every 11 seconds
            if (secElapsed(reportTime) >= 11)
            {
                printMemoryUsage();
                std::flush(cout);
                reportTime = startTimer();
            }
            printTick();
        }
    }

    client.unsubscribe(summary_id);
    client.unsubscribe(sub_id);
    std::this_thread::sleep_for(seconds(1));
    
    if (packetsSent > 0)
    {
        uint64_t packetsDropped = packetsSent - packetsReceived;
        cout << endl;
        cout << "Packets sent     : " << packetsSent << endl;
        cout << "Packets received : " << packetsReceived << endl;
        cout << "Packets dropped  : " << packetsDropped << endl;
        cout << "Packet loss      : " << 100.0f * packetsDropped / packetsSent << " %" << endl;
    } else {
        cout << endl << "Packets received : " << packetsReceived << endl;
        cout << "No packet summary received" << endl;
    }
}

// Check conversion results from strtol
bool isValidConversion(long l)
{
    return l != 0 && l != LONG_MIN && l != LONG_MAX;
}

void parseOpts(int argc, char* argv[])
{
    bool showHelp = (argc <= 1);
    
    using namespace clara;
    auto opts =
    Opt(dataKb, "kbyte")["-s"]["--size"]("Packet size in kB (1)") |
    Opt(durationSec, "seconds")["-t"]["--time"]("Test duration in seconds (default: 10)") |
    Opt(delayusec, "microseconds")["-d"]["--delay"]("Delay between sends in µs (100,000)") |
    Opt([](const bool) { verbosity = 1; })["--v"]("Print some information") |
    Opt([](const bool) { verbosity = 2; })["--vv"]("Print more information") |
    Opt(isStruct)["--struct"]("Use ccore_struct packets (default: BSON)") |
#ifdef UDP_SUPPORT
    Opt(isUdp)["-u"]["--udp"]("Use UDP packets (default: TCP)") |
#endif
    Arg([](const string &input)
        {
            // Check if it's an integer or sth. else
            const char* cstr = input.c_str();
            char* endptr;
            long portNum = strtol(cstr, &endptr, 10);
            // If only part of the string was converted, it does not count
            if (endptr != strlen(cstr) + cstr)
                portNum = 0;
            isListener = isValidConversion(portNum);
            uri = input;
            return ParserResult::ok(ParseResultType::Matched);
        },
        "(hostname:)port") |
    Help(showHelp);
    
    auto result = opts.parse(Args(argc, argv));
    if( !result ) {
        cerr << "Error in command line: " << result.errorMessage() << endl;
        exit(1);
    }
    if (showHelp)
    {
        printLogo();

        cout << opts;
        cout << "\nExamples:\n" << "perf 5555\t\t\t\t"
        << "Create a listener that waits for TCP connection on port 5555" << endl;
        cout << "perf hostname:5555\t\t\t"
        << "Create a sender that connects to hostname on port 5555" << endl;
        cout << "perf --size=120 hostname:1234\t\t"
        << "Create a sender that sends BSON messages with a size of 120kB each" << endl;
        cout << "perf 127.0.0.1:5555 --struct\t\t"
        << "TCP sender using preformatted data (ccore_struct)" << endl;
        cout << "perf 127.0.0.1:5555 --udp -d 2 -t 5\t"
        << "UDP sender with an inter-packet delay of 2 µs, active for 5 sec" << endl;
        cout << "perf 1234 --udp --struct --v\t\t"
        << "UDP struct listener with verbose output" << endl;

        cout <<endl;
        exit(0);
    }
}

int main(int argc, char* argv[])
{
    // Helpful delay for profiling only..
    // std::this_thread::sleep_for(seconds(5));
    parseOpts(argc, argv);
    baseMemoryKb = getMemoryUsage();

    cout << "Test duration: " << durationSec << " seconds" << endl;
    
    if (isListener)
    {
        makeListener();
    } else {
        ccore::client summarySender;
        makeSender();
    }
    
    std::this_thread::sleep_for(seconds(1));
    printMemoryUsage();
    cout << endl;
    return 0;
}
