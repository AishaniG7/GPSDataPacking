#pragma once 
#include <Arduino.h>

struct GPSPacket {
    uint8_t node_id;
    int32_t lat;
    int32_t lon;
};

static uint8_t NODE_ID = 0x01; // Hardcoded

inline void buildPacket(float latitude, float longitude, uint8_t *buffer) {
    GPSPacket packet;
    packet.node_id = NODE_ID;
    packet.lat = (int32_t)(latitude * 1e6);
    packet.lon = (int32_t)(longitude * 1e6);
    memcpy(buffer, &packet, sizeof(packet));
}

inline void parsePacket(uint8_t *buffer, float &latitude, float &longitude, uint8_t &node_id) {
    GPSPacket packet;
    memcpy(&packet, buffer, sizeof(packet));
    node_id = packet.node_id;
    latitude = packet.lat / 1e6;
    longitude = packet.lon / 1e6;
}