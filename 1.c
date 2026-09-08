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
    0x3e, 0x5e, 0x79, 0x39, 0x58, 0x78, 0x6d,
    0x50, 0x54, 0x5c, 0x73, 0x1c,
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
    0x3f|0x80, 0x06|0x80, 0x5b|0x80, 0x4f|0x80, 0x66|0x80,
    0x6d|0x80, 0x7d|0x80, 0x07|0x80, 0x7f|0x80, 0x6f|0x80
};
#endif
/* 追加字形：18=U 19=d 20=E 21=C 22=c 23=t 24=S
   25=r 26=n 27=o 28=P 29=u 30..35=单段a/b/c/d/e/f（第8位转圈动画用）
   36..45=带小数点的0..9（温度显示用） */

#define MUSIC_FRAME_LEN 6
#define MUSIC_FRAME_TYPE 0x20
#define MUSIC_TIMEOUT_TICKS 50

/* 按键帧：AA 5A 21 key action chk，chk = 0x21 ^ key ^ action
   key: 1=上 2=下 3=左(Backspace) 4=右(/) 5=中键(Enter) 6=K1(Ctrl+C) 7=K2(Tab) 8=K3(Esc)
   action: 1=按下 2=长按重复(仅上下) 3=抬起 */
#define KEY_FRAME_LEN 6
#define KEY_FRAME_TYPE 0x21

/* 工作状态帧：AA 5A 22 0 status chk，chk = 0x22 ^ 0 ^ status（字节3保留0，字节4为状态）
   status: 0=空闲(显示Stop) 1=工作中(显示run，第8位转圈，LED来回扫描) */
#define STATUS_FRAME_LEN 6
#define STATUS_FRAME_TYPE 0x22

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

/* 显示模式：音乐帧与工作状态帧共用一个串口，后到者接管显示 */
#define MODE_MUSIC  0
#define MODE_STATUS 1
unsigned char disp_mode = MODE_MUSIC;
unsigned char omp_running = 0;

/* 第8位转圈：单段 a→b→c→d→e→f 顺时针，80ms/步 */
#define GLYPH_R        25
#define GLYPH_N        26
#define GLYPH_O        27
#define GLYPH_P        28
#define GLYPH_U        29
#define GLYPH_SPIN_A   30
#define SPINNER_STEPS        6
#define SPINNER_PERIOD_TICKS 8
unsigned char spinner_phase = 0;
unsigned char spinner_ticks = 0;

/* LED 往复扫描：3 颗连灯常亮，在 8 颗 LED 上 0..5..0 连续来回运动（无暗场停留），100ms/步 */
#define SCANNER_PERIOD_TICKS 10
#define SCANNER_BAR_WIDTH    3
#define SCANNER_MAX_POS      (8 - SCANNER_BAR_WIDTH)
unsigned char scanner_pos = 0;
unsigned char scanner_dir = 1; /* 1=正向 0=反向 */
unsigned char scanner_ticks = 0;

unsigned char LedScannerMask(void)
{
    return (unsigned char)(((1 << SCANNER_BAR_WIDTH) - 1) << scanner_pos);
}

/* 温度：板载 10K/3950 NTC（Rt 通道），官方 BSP 例程换算表（0.1°C 单位，查表+线性插值）
   源表 -11 与 -87 之间的 -4.7 按单调性修正为 -47 */
#define GLYPH_MINUS 12
#define GLYPH_DP0   36
int temp_value = 0;              /* 显示值：30s 平均（0.1°C，如 253 = 25.3°C） */
unsigned int temp_sum = 0;
unsigned char temp_i = 0;
long temp_acc = 0;               /* 30s 窗口累加（0.1°C） */
unsigned int temp_n = 0;         /* 窗口内样本数 */
unsigned char temp_have_avg = 0; /* 首个窗口未满前跟随原始值，保证上电即有显示 */
#define TEMP_WINDOW_SAMPLES 188  /* 188 × ~160ms ≈ 30s */

int rt_to_tem(unsigned int adc, unsigned char adcbit)
{
    code int temtable[32] = {2000, 1293, 1016, 866, 763, 685, 621, 567, 520, 477, 439, 403, 370, 338, 308, 278, 250, 222, 194, 167, 139, 111, 83, 53, 22, -11, -47, -87, -132, -186, -256, -364};
    unsigned char resh;
    unsigned int resl;
    long diff;

    resl = adc << (16 - adcbit);
    resh = resl >> 11;
    resl = resl & 0x07ff;
    /* 官方例程此处为16位乘法，diff*resl 最大约 1.45M 会回绕，改用 long */
    diff = (long)(temtable[resh] - temtable[resh + 1]);
    return (int)(temtable[resh] - (int)((diff * resl) >> 11));
}

void FillTempGlyphs(unsigned char *g)
{
    int t = temp_value;
    unsigned int tt;
    unsigned char buf[4];
    unsigned char n = 0;
    unsigned char intpart, h, tens, ones, i;

    /* 温度字形紧凑左对齐：正常两位室温占 d0..d2；出现符号或
       三位整数（≥100°C / 负温）时自然多占一格（即"移回来"） */
    if (t < 0) { buf[n++] = GLYPH_MINUS; tt = (unsigned int)(-t); }
    else tt = (unsigned int)t;

    intpart = (unsigned char)(tt / 10); /* 整数部分，最高 200 */
    h = intpart / 100;
    tens = intpart / 10 % 10;
    ones = intpart % 10;
    if (h != 0) buf[n++] = h;
    if (intpart >= 10) buf[n++] = tens;
    buf[n++] = (unsigned char)(GLYPH_DP0 + ones); /* 个位带小数点 */
    buf[n++] = (unsigned char)(tt % 10);          /* 十分位 */

    for (i = 0; i < 4; i++) g[i] = (i < n) ? buf[i] : 10;
}

void RenderStatus(void)
{
    unsigned char g[4];

    FillTempGlyphs(g);
    if (omp_running)
        Seg7Print(g[0], g[1], g[2], g[3], GLYPH_R, GLYPH_U, GLYPH_N,
                  GLYPH_SPIN_A + spinner_phase);
    else
        Seg7Print(g[0], g[1], g[2], g[3], 24, 23, GLYPH_O, GLYPH_P); /* Stop */
}

/* 空闲（无音乐帧且未接 omp）：高4位显示温度，低4位熄灭 */
void RenderIdle(void)
{
    unsigned char g[4];

    FillTempGlyphs(g);
    Seg7Print(g[0], g[1], g[2], g[3], 10, 10, 10, 10);
}

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
        if (disp_mode != MODE_MUSIC)
        {
            disp_mode = MODE_MUSIC;
            LedPrint(0x00);
        }
        music_bars = music_frame[4];
        music_fresh_ticks = 0;
        music_timeout_rendered = 0;
        if (feedback_ticks == 0) RenderBars(music_bars);
    }
    else if ((music_frame[2] == STATUS_FRAME_TYPE) &&
             (music_frame[3] == 0) &&
             (music_frame[4] <= 1) &&
             (music_frame[5] == checksum))
    {
        disp_mode = MODE_STATUS;
        omp_running = music_frame[4];
        spinner_phase = 0;
        spinner_ticks = 0;
        scanner_pos = 0;
        scanner_dir = 1;
        scanner_ticks = 0;
        if (feedback_ticks == 0)
        {
            RenderStatus();
            LedPrint(omp_running ? LedScannerMask() : 0x00);
        }
    }
}

void OnSys10mS(void)
{
    struct_ADC adcres;

    if (music_fresh_ticks < MUSIC_TIMEOUT_TICKS)
    {
        music_fresh_ticks++;
        if ((music_fresh_ticks >= MUSIC_TIMEOUT_TICKS) &&
            (music_timeout_rendered == 0))
        {
            music_timeout_rendered = 1;
            music_bars = 0;
            if ((feedback_ticks == 0) && (disp_mode == MODE_MUSIC)) RenderIdle();
        }
    }

    if (feedback_ticks != 0)
    {
        feedback_ticks--;
        if (feedback_ticks == 0)
        {
            LedPrint(0x00);
            if (disp_mode == MODE_STATUS) RenderStatus();
            else if (music_fresh_ticks < MUSIC_TIMEOUT_TICKS) RenderBars(music_bars);
            else RenderIdle();
        }
    }

    if ((disp_mode == MODE_STATUS) && (omp_running) && (feedback_ticks == 0))
    {
        spinner_ticks++;
        if (spinner_ticks >= SPINNER_PERIOD_TICKS)
        {
            spinner_ticks = 0;
            spinner_phase++;
            if (spinner_phase >= SPINNER_STEPS) spinner_phase = 0;
            RenderStatus();
        }

        scanner_ticks++;
        if (scanner_ticks >= SCANNER_PERIOD_TICKS)
        {
            scanner_ticks = 0;
            if (scanner_dir)
            {
                scanner_pos++;
                if (scanner_pos >= SCANNER_MAX_POS) scanner_dir = 0;
            }
            else
            {
                scanner_pos--;
                if (scanner_pos == 0) scanner_dir = 1;
            }
            LedPrint(LedScannerMask());
        }
    }

    /* 温度采样：16次 Rt 求和（10bit→14bit）得原始值（~160mS），
       累加进 30s 窗口；窗口满才更新显示值（近30s平均） */
    adcres = GetADC();
    if (temp_i < 15)
    {
        temp_sum += adcres.Rt;
        temp_i++;
    }
    else
    {
        unsigned char temp_dirty = 0;

        temp_acc += rt_to_tem(temp_sum, 14);
        temp_i = 0;
        temp_sum = adcres.Rt;
        temp_n++;
        if (!temp_have_avg)
        {
            temp_value = (int)(temp_acc / temp_n);
            temp_dirty = 1;
        }
        if (temp_n >= TEMP_WINDOW_SAMPLES)
        {
            temp_value = (int)(temp_acc / temp_n);
            temp_acc = 0;
            temp_n = 0;
            temp_have_avg = 1;
            temp_dirty = 1;
        }
        if ((temp_dirty) && (feedback_ticks == 0))
        {
            if (disp_mode == MODE_STATUS) RenderStatus();
            else if (music_fresh_ticks >= MUSIC_TIMEOUT_TICKS) RenderIdle();
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
    RenderIdle(); /* 上电无音乐帧，直接进空闲温度显示 */

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
