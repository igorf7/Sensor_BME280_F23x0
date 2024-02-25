#ifndef  __packets
#define  __packets

#include "globalls.h"

#define METEO_WATCH_ID  (uint8_t)1
#define SENDER_ID       (uint8_t)2

#define MAX_PACKET_SIZE (uint8_t)64 // Including Size Byte
#define MAX_DATA_SIZE   (uint8_t)58 // MAX_PACKET_SIZE - Header Size - Size Byte

/* Device types */
typedef enum {
    METEO_WATCH_TYPE = (uint8_t)9,
    RF_BME280_TYPE,
    RF_REMOTE_TYPE
} DevType_t;

/* Packet types */
typedef enum { /* Packet types */
    BME280_PACKET = (uint8_t)1,
    CMD_PACKET,
    ACK_PACKET,
    DIA_PACKET,
    CONNECT
} PacketType_t;

/* Packet header structure */
#pragma pack(push, 1)
typedef struct {
	uint16_t dst_addr,
             snd_addr;
    uint8_t pack_type;
} RF_Header_t;
#pragma pack(pop)

/* Packet structure */
#pragma pack(push, 1)
typedef struct {
	RF_Header_t header;
	uint8_t data[MAX_DATA_SIZE];
} RF_Packet_t;
#pragma pack(pop)

void createRfPacket(PacketType_t pt, RF_Packet_t *packet);

#endif /* packets.h */
/* eof */
