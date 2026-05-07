#ifndef INCLUDED_MLRCS_H
#define INCLUDED_MLRCS_H

#include <iostream>
#include <string>
#include "mbed.h"
#include "EthernetInterface.h"
#include "rtos.h"

enum PacketType : uint8_t {
    GAMEPAD_DATA = 1,
    PING = 2,
    PONG = 3,
    ODOMETRY_DATA = 4
};

union ButtonBitmap {
    uint32_t value;
    struct {
        unsigned CROSS     : 1;
        unsigned CIRCLE    : 1;
        unsigned SQUARE    : 1;
        unsigned TRIANGLE  : 1;
        unsigned L1        : 1;
        unsigned R1        : 1;
        unsigned L2        : 1;
        unsigned R2        : 1;
        unsigned SHARE     : 1;
        unsigned OPTIONS   : 1;
        unsigned L3        : 1;
        unsigned R3        : 1;
        unsigned UP        : 1;
        unsigned DOWN      : 1;
        unsigned LEFT      : 1;
        unsigned RIGHT     : 1;
        unsigned PSBUTTON  : 1;

        unsigned CUSTOM_1  : 1;
        unsigned CUSTOM_2  : 1;
        unsigned CUSTOM_3  : 1;
        unsigned CUSTOM_4  : 1;
        unsigned CUSTOM_5  : 1;
        unsigned CUSTOM_6  : 1;
        unsigned CUSTOM_7  : 1;
        unsigned CUSTOM_8  : 1;
        unsigned CUSTOM_9  : 1;
        unsigned CUSTOM_10 : 1;
        unsigned RESERVED  : 5;
    } button;
};

struct GamepadData {
    uint32_t timestamp;
    int16_t axes[4];
    ButtonBitmap buttons;
} __attribute__((packed));

#ifndef STRUCT_POSE_DEFINED
#define STRUCT_POSE_DEFINED
struct Pose {
    float x;
    float y;
    float theta;
};
#endif

struct CommandPacket {
    uint8_t command;
};

extern const char* ODOMETRY_MULTICAST_GROUP;

class MLRCS {
public:
    GamepadData control_data;
    Pose current_pose = {0.0f, 0.0f, 0.0f};

    MLRCS(const char* toIP, const char* meIP);
    int init();

    void multicastOdometry(float odom[3]);

    int startOdometryReceiver();
    void sendResetCommandToOdom(const char* odom_ip);
    void send_tof_correction(const char* odom_ip,int enable);
    EthernetInterface* get_net_interface();

private:
    const char* destinationIP;
    const uint16_t destinationPort = 4000;
    const char* myIP;
    const char *myNetMask = "255.255.255.0";
    const uint16_t receivePort = 8080;
    const uint16_t commandPort = 6000;
    Timer t_;

    EthernetInterface net;
    UDPSocket udp;
    Thread receiveThread;

    Thread odomReceiveThread;
    UDPSocket odom_socket;

    SocketAddress destination;

    void receive();
    void _odom_receive_task();
};

#endif
