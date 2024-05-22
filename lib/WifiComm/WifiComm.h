#ifndef WifiComm_h
#define WifiComm_h

#include <WiFi.h>
#include <esp_now.h>    

class WifiComm
{
    private:
        esp_now_peer_info_t peerInfo;
        uint8_t broadcastAddress[6] = {0xFF};
        String message;

    public:
        bool new_data = 0;

        WifiComm();
        void begin();
        void send(String message);
        String get_message();
        void set_broadcast_address(uint8_t* broadcast_address);

};

#endif