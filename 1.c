#include "STC15F2K60S2.H"
#include "sys.H"
#include "displayer.H"
#include "uart1.h"

code unsigned long SysClock = 11059200;

#ifdef _displayer_H_
code char decode_table[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x00, 0x08, 0x40, 0x01, 0x41, 0x48,
    0x76, 0x38
};
#endif

#define MUSIC_FRAME_LEN 6
#define MUSIC_FRAME_TYPE 0x20
#define MUSIC_TIMEOUT_TICKS 50

unsigned char music_frame[MUSIC_FRAME_LEN];
unsigned char music_header[2] = {0xAA, 0x5A};
unsigned char music_bars = 0;
unsigned char music_fresh_ticks = MUSIC_TIMEOUT_TICKS;
unsigned char music_timeout_rendered = 1;

void RenderBars(unsigned char bars)
{
    unsigned char d0 = 10;
    unsigned char d1 = 10;
    unsigned char d2 = 10;
    unsigned char d3 = 10;
    unsigned char d4 = 10;
    unsigned char d5 = 10;
    unsigned char d6 = 10;
    unsigned char d7 = 10;

    if (bars > 8) bars = 8;
    if (bars > 0) d0 = 8;
    if (bars > 1) d1 = 8;
    if (bars > 2) d2 = 8;
    if (bars > 3) d3 = 8;
    if (bars > 4) d4 = 8;
    if (bars > 5) d5 = 8;
    if (bars > 6) d6 = 8;
    if (bars > 7) d7 = 8;
    Seg7Print(d0, d1, d2, d3, d4, d5, d6, d7);
}

void OnUart1Rxd(void)
{
    unsigned char checksum;

    checksum = music_frame[2] ^ music_frame[3] ^ music_frame[4];
    if ((music_frame[2] == MUSIC_FRAME_TYPE) &&
        (music_frame[4] <= 8) &&
        (music_frame[5] == checksum))
    {
        music_bars = music_frame[4];
        music_fresh_ticks = 0;
        music_timeout_rendered = 0;
        RenderBars(music_bars);
    }
}

void OnSys10mS(void)
{
    if (music_fresh_ticks < MUSIC_TIMEOUT_TICKS)
    {
        music_fresh_ticks++;
        if ((music_fresh_ticks >= MUSIC_TIMEOUT_TICKS) &&
            (music_timeout_rendered == 0))
        {
            music_timeout_rendered = 1;
            music_bars = 0;
            RenderBars(0);
        }
    }
}

void main(void)
{
    DisplayerInit();
    SetDisplayerArea(0, 7);
    RenderBars(0);

    Uart1Init(115200);
    SetUart1Rxd(music_frame, MUSIC_FRAME_LEN, music_header, 2);

    MySTC_Init();
    SetEventCallBack(enumEventUart1Rxd, OnUart1Rxd);
    SetEventCallBack(enumEventSys10mS, OnSys10mS);
    while (1)
    {
        MySTC_OS();
    }
}