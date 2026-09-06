#include "STC15F2K60S2.H"
#include "sys.H"
#include "displayer.H"
#include "uart1.h"
#include "adc.h"
#include "Key.H"

code unsigned long SysClock = 11059200;

#ifdef _displayer_H_
code char decode_table[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x00, 0x08, 0x40, 0x01, 0x41, 0x48,
    0x76, 0x38,
    0x3e, 0x5e, 0x79, 0x39, 0x58, 0x78, 0x6d
};
#endif
/* 追加字形：18=U 19=d 20=E 21=C 22=c 23=t 24=S */

#define MUSIC_FRAME_LEN 6
#define MUSIC_FRAME_TYPE 0x20
#define MUSIC_TIMEOUT_TICKS 50

/* 按键帧：AA 5A 21 key action chk，chk = 0x21 ^ key ^ action
   key: 1=上 2=下 3=左(Esc) 4=右(/) 5=中键(Enter) 6=K1(Ctrl+C) 7=K2(Tab) 8=K3(Enter)
   action: 1=按下 2=长按重复(仅上下) 3=抬起 */
#define KEY_FRAME_LEN 6
#define KEY_FRAME_TYPE 0x21

#define KEY_NONE   0
#define KEY_UP     1
#define KEY_DOWN   2
#define KEY_LEFT   3
#define KEY_RIGHT  4
#define KEY_CENTER 5
#define KEY_K1     6
#define KEY_K2     7
#define KEY_K3     8

#define ACT_PRESS   1
#define ACT_REPEAT  2
#define ACT_RELEASE 3

#define FEEDBACK_TICKS      30
#define REPEAT_DELAY_TICKS  45
#define REPEAT_PERIOD_TICKS 17

unsigned char music_frame[MUSIC_FRAME_LEN];
unsigned char music_header[2] = {0xAA, 0x5A};
unsigned char music_bars = 0;
unsigned char music_fresh_ticks = MUSIC_TIMEOUT_TICKS;
unsigned char music_timeout_rendered = 1;

unsigned char key_frame[KEY_FRAME_LEN] = {0xAA, 0x5A, KEY_FRAME_TYPE, 0, 0, 0};
unsigned char feedback_ticks = 0;
unsigned char held_key = KEY_NONE;
unsigned int hold_ticks = 0;

code unsigned char key_glyph[9] = {0, 18, 19, 22, 24, 20, 21, 23, 20};

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

void SendKeyFrame(unsigned char key, unsigned char action)
{
    key_frame[3] = key;
    key_frame[4] = action;
    key_frame[5] = KEY_FRAME_TYPE ^ key ^ action;
    Uart1Print(key_frame, KEY_FRAME_LEN);
}

void FlashKeyGlyph(unsigned char glyph)
{
    Seg7Print(glyph, 10, 10, 10, 10, 10, 10, 10);
    LedPrint(0x01);
    feedback_ticks = FEEDBACK_TICKS;
}

void SendKey(unsigned char key)
{
    SendKeyFrame(key, ACT_PRESS);
    FlashKeyGlyph(key_glyph[key]);
    if ((key == KEY_UP) || (key == KEY_DOWN))
    {
        held_key = key;
        hold_ticks = 0;
    }
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
        if (feedback_ticks == 0) RenderBars(music_bars);
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
            if (feedback_ticks == 0) RenderBars(0);
        }
    }

    if (feedback_ticks != 0)
    {
        feedback_ticks--;
        if (feedback_ticks == 0)
        {
            LedPrint(0x00);
            RenderBars(music_bars);
        }
    }

    if (held_key != KEY_NONE)
    {
        hold_ticks++;
        if ((hold_ticks >= REPEAT_DELAY_TICKS) &&
            (((hold_ticks - REPEAT_DELAY_TICKS) % REPEAT_PERIOD_TICKS) == 0))
        {
            SendKeyFrame(held_key, ACT_REPEAT);
            FlashKeyGlyph(key_glyph[held_key]);
        }
    }
}

/* K3 为独立按键，与导航摇杆共用 P1.7 的 ADC 分压通道，须经 GetAdcNavAct 读取 */
void OnNav(void)
{
    unsigned char act;

    act = GetAdcNavAct(enumAdcNavKeyUp);
    if (act == enumKeyPress) SendKey(KEY_UP);
    else if (act == enumKeyRelease)
    {
        if (held_key == KEY_UP) held_key = KEY_NONE;
    }

    act = GetAdcNavAct(enumAdcNavKeyDown);
    if (act == enumKeyPress) SendKey(KEY_DOWN);
    else if (act == enumKeyRelease)
    {
        if (held_key == KEY_DOWN) held_key = KEY_NONE;
    }

    act = GetAdcNavAct(enumAdcNavKeyLeft);
    if (act == enumKeyPress) SendKey(KEY_LEFT);

    act = GetAdcNavAct(enumAdcNavKeyRight);
    if (act == enumKeyPress) SendKey(KEY_RIGHT);

    act = GetAdcNavAct(enumAdcNavKeyCenter);
    if (act == enumKeyPress) SendKey(KEY_CENTER);

    act = GetAdcNavAct(enumAdcNavKey3);
    if (act == enumKeyPress) SendKey(KEY_K3);
}

void OnKey(void)
{
    unsigned char act;

    act = GetKeyAct(enumKey1);
    if (act == enumKeyPress) SendKey(KEY_K1);

    act = GetKeyAct(enumKey2);
    if (act == enumKeyPress) SendKey(KEY_K2);
}

void main(void)
{
    DisplayerInit();
    SetDisplayerArea(0, 7);
    RenderBars(0);

    Uart1Init(115200);
    SetUart1Rxd(music_frame, MUSIC_FRAME_LEN, music_header, 2);

    AdcInit(ADCexpEXT);
    KeyInit();

    MySTC_Init();
    SetEventCallBack(enumEventUart1Rxd, OnUart1Rxd);
    SetEventCallBack(enumEventSys10mS, OnSys10mS);
    SetEventCallBack(enumEventNav, OnNav);
    SetEventCallBack(enumEventKey, OnKey);
    while (1)
    {
        MySTC_OS();
    }
}
