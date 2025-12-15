#ifndef CRTP_H_
#define CRTP_H_
#include <stdint.h>
#include <stdbool.h>
#define CRTP_MAX_DATA_SIZE 30
#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))
#define CRTP_IS_NULL_PACKET(P) ((P.header&0xF3)==0xF3)
typedef enum {
  CRTP_PORT_CONSOLE          = 0x00,
  CRTP_PORT_PARAM            = 0x02,
  CRTP_PORT_SETPOINT         = 0x03,
  CRTP_PORT_MEM              = 0x04,
  CRTP_PORT_LOG              = 0x05,
  CRTP_PORT_LOCALIZATION     = 0x06,
  CRTP_PORT_SETPOINT_GENERIC = 0x07,
  CRTP_PORT_SETPOINT_HL      = 0x08,
  CRTP_PORT_PLATFORM         = 0x0D,
  CRTP_PORT_LINK             = 0x0F,
} CRTPPort;
typedef struct _CRTPPacket
{
  uint8_t size;                         
  union {
    struct {
      union {
        uint8_t header;                 
        struct {
#ifndef CRTP_HEADER_COMPAT
          uint8_t channel     : 2;      
          uint8_t reserved    : 2;
          uint8_t port        : 4;      
#else
          uint8_t channel  : 2;
          uint8_t port     : 4;
          uint8_t reserved : 2;
#endif
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE]; 
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+1];  
  };
} __attribute__((packed)) CRTPPacket;
typedef void (*CrtpCallback)(CRTPPacket *);
void crtpInit(void);
bool crtpTest(void);
void crtpInitTaskQueue(CRTPPort taskId);
void crtpRegisterPortCB(int port, CrtpCallback cb);
int crtpSendPacket(CRTPPacket *p);
int crtpSendPacketBlock(CRTPPacket *p);
int crtpReceivePacket(CRTPPort taskId, CRTPPacket *p);
int crtpReceivePacketWait(CRTPPort taskId, CRTPPacket *p, int wait);
int crtpGetFreeTxQueuePackets(void);
int crtpReceivePacketBlock(CRTPPort taskId, CRTPPacket *p);
struct crtpLinkOperations
{
  int (*setEnable)(bool enable);
  int (*sendPacket)(CRTPPacket *pk);
  int (*receivePacket)(CRTPPacket *pk);
  bool (*isConnected)(void);
  int (*reset)(void);
};
void crtpSetLink(struct crtpLinkOperations * lk);
bool crtpIsConnected(void);
int crtpReset(void);
#endif 