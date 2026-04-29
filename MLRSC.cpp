#include "MLRCS.hpp"

const char* ODOMETRY_MULTICAST_GROUP = "239.0.0.1";

MLRCS::MLRCS(const char* toIP, const char* meIP)
    : destinationIP(toIP), myIP(meIP) {
}

int MLRCS::init(){
    net.set_dhcp(false);
    net.set_network(myIP, myNetMask, "");

    if (net.connect() != 0){
        printf("network connection Error\n");
        return -1;
    } else {
        printf("network connection success\n");
    }

    udp.open(&net);
    udp.bind(receivePort);

    destination.set_ip_address(destinationIP);
    destination.set_port(destinationPort);

    receiveThread.start(callback(this, &MLRCS::receive));

    t_.start();
    return 1;
}

void MLRCS::multicastOdometry(float odom[3]) {
    char buffer[13];
    buffer[0] = PacketType::ODOMETRY_DATA;
    memcpy(buffer + 1, odom, sizeof(float) * 3);

    SocketAddress multicast_addr;
    multicast_addr.set_ip_address(ODOMETRY_MULTICAST_GROUP);
    multicast_addr.set_port(destinationPort);

    udp.sendto(multicast_addr, buffer, sizeof(buffer));
}

void MLRCS::sendResetCommandToOdom(const char* odom_ip) {
    CommandPacket packet;
    packet.command = 1;

    SocketAddress addr;
    addr.set_ip_address(odom_ip);
    addr.set_port(commandPort);

    udp.sendto(addr, &packet, sizeof(packet));
}

void MLRCS::send_tof_correction(const char* odom_ip,int enable) {
    CommandPacket packet;
    packet.command = enable;

    SocketAddress addr;
    addr.set_ip_address(odom_ip);
    addr.set_port(commandPort);

    udp.sendto(addr, &packet, sizeof(packet));
}

int MLRCS::startOdometryReceiver() {
    if (net.get_connection_status() != NSAPI_STATUS_GLOBAL_UP) {
        printf("Error: Network not connected\n");
        return -1;
    }
    return odomReceiveThread.start(callback(this, &MLRCS::_odom_receive_task));
}

void MLRCS::_odom_receive_task() {

    odom_socket.open(&net);
    odom_socket.bind(destinationPort);

    odom_socket.join_multicast_group(SocketAddress(ODOMETRY_MULTICAST_GROUP));

    char buffer[13];
    SocketAddress sender;

    while(true) {
        auto result = odom_socket.recvfrom(&sender, buffer, sizeof(buffer));

        if (result == 13 && buffer[0] == PacketType::ODOMETRY_DATA) {
            memcpy(&current_pose, buffer + 1, sizeof(Pose));
        }
    }
}

EthernetInterface* MLRCS::get_net_interface() {
    return &net;
}

void MLRCS::receive() {

    SocketAddress source;
    char buffer[100];

    while (1){
        int result = udp.recvfrom(&source, buffer, sizeof(buffer));

        if (result > 0){
            uint8_t type = buffer[0];

            if (type == PacketType::GAMEPAD_DATA && result >= sizeof(GamepadData) + 1) {
                memcpy(&control_data, buffer + 1, sizeof(GamepadData));
                t_.reset();
            }
            else if (type == PacketType::PING) {
                buffer[0] = PacketType::PONG;
                udp.sendto(destination, buffer, result);
            }
        }

        if(t_.read_ms() > 100){
            memset(&control_data, 0, sizeof(GamepadData));
        }
    }
}
