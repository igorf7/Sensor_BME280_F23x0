#include "packets.h"

void createRfPacket(PacketType_t pt, RF_Packet_t *packet)
{   
    // Header field
    packet->header.dst_addr = (METEO_WATCH_TYPE << 8) | METEO_WATCH_ID;
    packet->header.snd_addr = (RF_BME280_TYPE << 8) | SENDER_ID;
    packet->header.pack_type = pt;
}
//eof
