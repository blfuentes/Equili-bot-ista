#include <cstdint>
#ifndef __LIBNOW_H__
#define __LIBNOW_H__


typedef struct {
    uint8_t mode;
    int move_x;
    int move_y;
    float param_p;
    float param_i;
    float param_d;
} message_control_status;

typedef enum{
    DST_ROBOT = 0,
    DST_MANDO
} LibNowDst;

typedef enum {
    MODE_STANDING = 0,
    MODE_WAR = 1
} ModeTypeTranslation;

void libnow_init();

void libnow_addPeer(LibNowDst dst);
void libnow_sendMessage(LibNowDst dst, message_control_status *msg);

#endif