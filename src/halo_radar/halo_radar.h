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

} // namespace halo_radar

#endif
