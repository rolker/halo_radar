#ifndef HALO_RADAR_HALO_RADAR_H
#define HALO_RADAR_HALO_RADAR_H

#include <ifaddrs.h>
#include <netinet/in.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <map>
#include <chrono>

#include "halo_radar_structures.h"

namespace halo_radar
{
    
bool validInterface(ifaddrs const *i);

std::vector<uint32_t> getLocalAddresses();

std::string ipAddressToString(uint32_t a);
uint32_t ipAddressFromString(const std::string &a);

struct AddressSet
{
    std::string label;
    IPAddress data;
    IPAddress send;
    IPAddress report;
    uint32_t interface;
    
    std::string str() const;
};

std::vector <AddressSet> scan();
std::vector <AddressSet> scan(const std::vector<uint32_t> &addresses);

struct Scanline
{
    float angle; // degrees clockwise relative to fwd
    float range; // meters
    std::vector<uint8_t> intensities;
};

class Radar
{
public:
    Radar(AddressSet const &addresses);
    ~Radar();
    
    void sendCommand(std::string const &key, std::string const &value);
    bool checkHeartbeat();

protected:
    virtual void processData(std::vector<Scanline> const &scanlines)=0;
    virtual void stateUpdated()=0;
    void startThreads();

    std::map <std::string, std::string> m_state;
private:
    void dataThread();
    void reportThread();
    int createListenerSocket(uint32_t interface, uint32_t mcast_address, uint16_t port);
    void sendCommand(const uint8_t data[], int size);
    template<typename T> void sendCommand(const T &data)
    {
        sendCommand(reinterpret_cast<const uint8_t*>(&data),sizeof(T));
    }
    void sendHeartbeat();
    
    AddressSet m_addresses;
    std::thread m_dataThread;
    
    int m_sendSocket;
    sockaddr_in m_sendAddress;
    
    std::thread m_reportThread;
    bool m_exitFlag;
    std::mutex m_exitFlagMutex;
    
    std::chrono::system_clock::time_point m_lastHeartbeat;
};

class HeadingSender
{
public:
    HeadingSender(uint32_t bindAddress);
    ~HeadingSender();
    void setHeading(double heading);

private:
    void senderThread();

private:
    int m_socket = 0;
    sockaddr_in m_sendAddress;
    double m_heading = 0.0;
    std::mutex m_headingMutex;
    std::thread m_senderThread;
    bool m_exitFlag = false;
    std::mutex m_exitFlagMutex;

    uint16_t m_counter = 0;

    std::chrono::system_clock::time_point m_lastHeadingSent;
    std::chrono::milliseconds m_headingSendInterval = std::chrono::milliseconds(100);
    std::chrono::system_clock::time_point m_lastMysterySent;
    std::chrono::milliseconds m_mysterySendInterval = std::chrono::milliseconds(250);

    HaloHeadingPacket m_headingPacket = {
        {'N', 'K', 'O', 'E'},  // marker
        {0, 1, 0x90, 0x02},    // u00 bytes containing '00 01 90 02'
        0,                     // counter
        {0, 0, 0x10, 0, 0, 0x14, 0, 0, 4, 0, 0, 0, 0, 0, 5, 0x3C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x20},  // u01
        {0x12, 0xf1},                                                                                // u02
        {0x01, 0x00},                                                                                // u03
        0,                                                                                           // epoch
        2,                                                                                           // u04
        0,                                                                                           // u05a, likely position
        0,                                                                                           // u05b, likely position
        {0xff},                                                                                      // u06
        0,                                                                                           // heading
        {0xff, 0x7f, 0x79, 0xf8, 0xfc}                                                               // u07
    };

    HaloMysteryPacket m_mysteryPacket  = {
        {'N', 'K', 'O', 'E'},  // marker
        {0, 1, 0x90, 0x02},    // u00 bytes containing '00 01 90 02'
        0,                     // counter
        {0, 0, 0x10, 0, 0, 0x14, 0, 0, 4, 0, 0, 0, 0, 0, 5, 0x3C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x20},  // u01
        {0x02, 0xf8},                                                                                // u02
        {0x01, 0x00},                                                                                // u03
        0,                                                                                           // epoch
        2,                                                                                           // u04
        0,                                                                                           // u05a, likely position
        0,                                                                                           // u05b, likely position
        {0xff},                                                                                      // u06
        {0xfc},                                                                                      // u07
        0,                                                                                           // mystery1
        0,                                                                                           // mystery2
        {0xff, 0xff}                                                                                 // u08
    };
};

} // namespace halo_radar

#endif
