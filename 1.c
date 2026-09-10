#include "STC15F2K60S2.H"
#include "sys.H"
#include "displayer.H"
#include "uart1.h"
#include "adc.h"
#include "Key.H"
#include "Beep.H"

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
    0x6d|0x80, 0x7d|0x80, 0x07|0x80, 0x7f|0x80, 0x6f|0x80,
    0x63, 0x1c
};
#endif
/* 追加字形：18=U 19=d 20=E 21=C 22=c 23=t 24=S
   25=r 26=n 27=o 28=P 29=u 30..35=单段a/b/c/d/e/f（第8位转圈动画用）
   36..45=带小数点的0..9（温度显示用）
   46=上半段a/b/f/g 47=下半段c/d/e（双声道律动用） */

#define MUSIC_FRAME_LEN 6
#define MUSIC_FRAME_TYPE 0x20
#define MUSIC_TIMEOUT_TICKS 50

/* 双声道音乐帧：AA 5A 26 L R chk，chk = 0x26 ^ L ^ R（L/R=左右声道电平 0..8）
   每位数码管拆上下半段：上半段亮=左声道，下半段亮=右声道，上下都亮=整字 */
#define STEREO_FRAME_TYPE 0x26

/* 按键帧：AA 5A 21 key action chk，chk = 0x21 ^ key ^ action
   key: 1=上 2=下 3=左(Backspace) 4=右(/) 5=中键(Enter) 6=K1(Ctrl+C) 7=K2(Tab) 8=K3(Esc)
   action: 1=按下 2=长按重复(仅上下) 3=抬起 */
#define KEY_FRAME_LEN 6
#define KEY_FRAME_TYPE 0x21

/* 工作状态帧：AA 5A 22 0 status chk，chk = 0x22 ^ 0 ^ status（字节3保留0，字节4为状态）
   status: 0=空闲(显示Stop) 1=工作中(显示run，第8位转圈，LED来回扫描) */
#define STATUS_FRAME_LEN 6
#define STATUS_FRAME_TYPE 0x22

/* 提醒音帧：AA 5A 23 0 tune chk，chk = 0x23 ^ 0 ^ tune（字节3保留0，字节4为曲调号）
   tune: 1=C4-E4-G4-C5 上行琶音（任务完成/需手动操作提醒），每音 250ms */
#define TUNE_FRAME_LEN 6
#define TUNE_FRAME_TYPE 0x23
#define TUNE_ID_REMIND 1

/* SafeKey 帧（上位机保险箱的硬件 PIN 认证，参考 icecat897/SafeKey-STC-B 整合）：
   0x24 主机→板子：AA 5A 24 0 cmd chk，chk = 0x24 ^ 0 ^ cmd；cmd 1=进入PIN输入模式 0=退出
   0x25 板子→主机：AA 5A 25 ev payload chk，chk = 0x25 ^ ev ^ payload
   ev: 1=认证成功 2=认证失败(payload=已失败次数) 3=进入锁定(payload=30秒)
       4=输入进度(payload=已确认位数) 5=退出PIN模式
   PIN 模式下 K1=当前位数字+1 K2=确认进入下一位(第6位确认即本地校验) K3=回退一位，
   按键由板内消费、不转发按键帧；PIN 模式期间忽略音乐/状态/提醒音帧 */
#define SAFEKEY_FRAME_TYPE  0x24
#define SKAUTH_FRAME_TYPE   0x25
#define SK_EV_OK    1
#define SK_EV_FAIL  2
#define SK_EV_LOCK  3
#define SK_EV_INPUT 4
#define SK_EV_EXIT  5

#define PIN_LEN        6
#define PIN_MAX_FAIL   3
#define PIN_LOCK_TICKS 3000 /* 10ms 节拍 × 3000 = 30s */
/* 板上固定 PIN，存数字而非 ASCII（如需更换改此处后重新编译烧录） */
code unsigned char pin_code[PIN_LEN] = {1, 2, 3, 4, 5, 6};
unsigned char pin_mode = 0;
unsigned char pin_buf[PIN_LEN];
unsigned char pin_pos = 0;
unsigned char pin_digit = 0;
unsigned char pin_fails = 0;
unsigned char pin_locked = 0;
unsigned int pin_lock_ticks = 0;
unsigned char sk_tx[6];

#define NOTE_COUNT 4
#define NOTE_TICKS 25 /* SetBeep 时长=10×tick，25→250ms，与 10ms 系统节拍一致 */
code unsigned int remind_notes[NOTE_COUNT] = {262, 330, 392, 523}; /* C4 E4 G4 C5 */
unsigned char tune_playing = 0;
unsigned char tune_note = 0;
unsigned char tune_need_sound = 0;
unsigned char tune_ticks = 0;

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
unsigned char music_left = 0;
unsigned char music_right = 0;
unsigned char music_stereo = 0; /* 最近一帧音乐数据是否为双声道（0x26） */
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

#define GLYPH_BAR_TOP    46
#define GLYPH_BAR_BOTTOM 47

/* 双声道律动：第 i 位上/下半段按声道电平独立点亮，两级都亮则显示整字 8 */
void RenderBarsStereo(unsigned char left, unsigned char right)
{
    unsigned char g[8];
    unsigned char i;

    if (left > 8) left = 8;
    if (right > 8) right = 8;
    for (i = 0; i < 8; i++)
    {
        if ((left > i) && (right > i)) g[i] = 8;
        else if (left > i) g[i] = GLYPH_BAR_TOP;
        else if (right > i) g[i] = GLYPH_BAR_BOTTOM;
        else g[i] = 10;
    }
    Seg7Print(g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7]);
}

void SendKeyFrame(unsigned char key, unsigned char action)
{
    key_frame[3] = key;
    key_frame[4] = action;
    key_frame[5] = KEY_FRAME_TYPE ^ key ^ action;
    Uart1Print(key_frame, KEY_FRAME_LEN);
}

/* ---------------- SafeKey PIN 认证 ---------------- */

void SkSend(unsigned char ev, unsigned char payload)
{
    sk_tx[0] = 0xAA;
    sk_tx[1] = 0x5A;
    sk_tx[2] = SKAUTH_FRAME_TYPE;
    sk_tx[3] = ev;
    sk_tx[4] = payload;
    sk_tx[5] = SKAUTH_FRAME_TYPE ^ ev ^ payload;
    Uart1Print(sk_tx, 6);
}

/* 输入显示：d0..d5=已确认位（未确认处熄灭，当前编辑位实时显示正在输入的数字），
   d6=已确认位数 d7=当前编辑数字 */
void SkShow(void)
{
    unsigned char d0 = 10, d1 = 10, d2 = 10, d3 = 10, d4 = 10, d5 = 10;

    if (pin_pos > 0) d0 = pin_buf[0];
    if (pin_pos > 1) d1 = pin_buf[1];
    if (pin_pos > 2) d2 = pin_buf[2];
    if (pin_pos > 3) d3 = pin_buf[3];
    if (pin_pos > 4) d4 = pin_buf[4];
    if (pin_pos == 0) d0 = pin_digit;
    else if (pin_pos == 1) d1 = pin_digit;
    else if (pin_pos == 2) d2 = pin_digit;
    else if (pin_pos == 3) d3 = pin_digit;
    else if (pin_pos == 4) d4 = pin_digit;
    else d5 = pin_digit;
    Seg7Print(d0, d1, d2, d3, d4, d5, pin_pos, pin_digit);
}

/* 锁定倒计时：低两位显示剩余秒数 */
void SkShowLock(void)
{
    unsigned char sec = (unsigned char)(pin_lock_ticks / 100);

    Seg7Print(10, 10, 10, 10, 10, 10, sec / 10, sec % 10);
}

void SkClear(void)
{
    unsigned char i;

    for (i = 0; i < PIN_LEN; i++) pin_buf[i] = 0;
    pin_pos = 0;
    pin_digit = 0;
    SkShow();
}

void SkSuccess(void)
{
    pin_locked = 0;
    pin_fails = 0;
    LedPrint(0xFF);
    SetBeep(1800, 12); /* 120ms 长鸣 */
    SkSend(SK_EV_OK, 0);
    Seg7Print(10, 10, 10, 10, 10, 10, 10, 10);
}

void SkFail(void)
{
    pin_fails++;
    LedPrint(0x00);
    SetBeep(500, 22); /* 220ms 低鸣 */
    SkSend(SK_EV_FAIL, pin_fails);
    if (pin_fails >= PIN_MAX_FAIL)
    {
        pin_locked = 1;
        pin_lock_ticks = PIN_LOCK_TICKS;
        LedPrint(0x81);
        SkSend(SK_EV_LOCK, 30);
    }
    SkClear();
}

void SkCheck(void)
{
    unsigned char i;

    for (i = 0; i < PIN_LEN; i++)
    {
        if (pin_buf[i] != pin_code[i])
        {
            SkFail();
            return;
        }
    }
    SkSuccess();
}

/* PIN 模式下的 K1/K2/K3：由板内消费，不转发按键帧 */
void SkKey(unsigned char key)
{
    if (!pin_mode) return;
    if (pin_locked) return;
    if (key == KEY_K1)
    {
        pin_digit++;
        if (pin_digit > 9) pin_digit = 0;
        SetBeep(1200, 3);
        SkShow();
    }
    else if (key == KEY_K2)
    {
        pin_buf[pin_pos] = pin_digit;
        pin_pos++;
        pin_digit = 0;
        SetBeep(1400, 3);
        if (pin_pos >= PIN_LEN) SkCheck();
        else
        {
            SkShow();
            SkSend(SK_EV_INPUT, pin_pos);
        }
    }
    else /* KEY_K3 回退一位；第 0 位时只把当前数字归零 */
    {
        if (pin_pos > 0)
        {
            pin_pos--;
            pin_digit = pin_buf[pin_pos];
            pin_buf[pin_pos] = 0;
        }
        else
        {
            pin_digit = 0;
        }
        SetBeep(800, 5);
        SkShow();
        SkSend(SK_EV_INPUT, pin_pos);
    }
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
    if ((pin_mode == 0) &&
        (music_frame[2] == MUSIC_FRAME_TYPE) &&
        (music_frame[4] <= 8) &&
        (music_frame[5] == checksum))
    {
        if (disp_mode != MODE_MUSIC)
        {
            disp_mode = MODE_MUSIC;
            LedPrint(0x00);
        }
        music_bars = music_frame[4];
        music_stereo = 0;
        music_fresh_ticks = 0;
        music_timeout_rendered = 0;
        if (feedback_ticks == 0) RenderBars(music_bars);
    }
    else if ((pin_mode == 0) &&
             (music_frame[2] == STEREO_FRAME_TYPE) &&
             (music_frame[3] <= 8) &&
             (music_frame[4] <= 8) &&
             (music_frame[5] == checksum))
    {
        if (disp_mode != MODE_MUSIC)
        {
            disp_mode = MODE_MUSIC;
            LedPrint(0x00);
        }
        music_stereo = 1;
        music_left = music_frame[3];
        music_right = music_frame[4];
        music_fresh_ticks = 0;
        music_timeout_rendered = 0;
        if (feedback_ticks == 0) RenderBarsStereo(music_left, music_right);
    }
    else if ((pin_mode == 0) &&
             (music_frame[2] == STATUS_FRAME_TYPE) &&
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
    else if ((music_frame[2] == TUNE_FRAME_TYPE) &&
             (music_frame[3] == 0) &&
             (music_frame[4] == TUNE_ID_REMIND) &&
             (music_frame[5] == checksum))
    {
        /* 触发一次提醒音，不改变显示模式；PIN 模式下忽略以免与输入音冲突 */
        if (pin_mode == 0)
        {
            tune_playing = 1;
            tune_note = 0;
            tune_need_sound = 1;
            tune_ticks = 0;
        }
    }
    else if ((music_frame[2] == SAFEKEY_FRAME_TYPE) &&
             (music_frame[3] == 0) &&
             (music_frame[4] <= 1) &&
             (music_frame[5] == checksum))
    {
        if (music_frame[4])
        {
            /* 进入 PIN 模式：接管显示与 K1/K2/K3，丢弃未完成的按键反馈闪烁 */
            pin_mode = 1;
            pin_locked = 0;
            pin_fails = 0;
            feedback_ticks = 0;
            tune_playing = 0;
            LedPrint(0x18);
            SkClear();
        }
        else
        {
            pin_mode = 0;
            LedPrint(0x00);
            RenderIdle();
            SkSend(SK_EV_EXIT, 0);
        }
    }
}

void OnSys10mS(void)
{
    struct_ADC adcres;

    /* 提醒音序列：非阻塞，每音 250ms；上一音未结束(SetBeep 忙)时下个节拍重试 */
    if (tune_playing)
    {
        if (tune_need_sound)
        {
            if (SetBeep(remind_notes[tune_note], NOTE_TICKS) == enumSetBeepOK)
            {
                tune_need_sound = 0;
                tune_ticks = NOTE_TICKS;
            }
        }
        else
        {
            if (tune_ticks != 0) tune_ticks--;
            if (tune_ticks == 0)
            {
                tune_note++;
                if (tune_note >= NOTE_COUNT) tune_playing = 0;
                else tune_need_sound = 1;
            }
        }
    }

    if (music_fresh_ticks < MUSIC_TIMEOUT_TICKS)
    {
        music_fresh_ticks++;
        if ((music_fresh_ticks >= MUSIC_TIMEOUT_TICKS) &&
            (music_timeout_rendered == 0))
        {
            music_timeout_rendered = 1;
            music_bars = 0;
            music_left = 0;
            music_right = 0;
            if ((feedback_ticks == 0) && (disp_mode == MODE_MUSIC) && (pin_mode == 0)) RenderIdle();
        }
    }

    /* SafeKey 锁定倒计时：每秒刷新剩余秒数，归零后恢复输入 */
    if ((pin_mode) && (pin_locked) && (pin_lock_ticks != 0))
    {
        pin_lock_ticks--;
        if ((pin_lock_ticks % 100) == 0) SkShowLock();
        if (pin_lock_ticks == 0)
        {
            pin_locked = 0;
            pin_fails = 0;
            LedPrint(0x18);
            SkClear();
        }
    }

    if (feedback_ticks != 0)
    {
        feedback_ticks--;
        if (feedback_ticks == 0)
        {
            LedPrint(0x00);
            if (disp_mode == MODE_STATUS) RenderStatus();
            else if (music_fresh_ticks < MUSIC_TIMEOUT_TICKS)
            {
                if (music_stereo) RenderBarsStereo(music_left, music_right);
                else RenderBars(music_bars);
            }
            else RenderIdle();
        }
    }

    if ((disp_mode == MODE_STATUS) && (omp_running) && (feedback_ticks == 0) && (pin_mode == 0))
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
        if ((temp_dirty) && (feedback_ticks == 0) && (pin_mode == 0))
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
    if (act == enumKeyPress)
    {
        if (pin_mode) SkKey(KEY_K3);
        else SendKey(KEY_K3);
    }
}

void OnKey(void)
{
    unsigned char act;

    act = GetKeyAct(enumKey1);
    if (act == enumKeyPress)
    {
        if (pin_mode) SkKey(KEY_K1);
        else SendKey(KEY_K1);
    }

    act = GetKeyAct(enumKey2);
    if (act == enumKeyPress)
    {
        if (pin_mode) SkKey(KEY_K2);
        else SendKey(KEY_K2);
    }
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
    BeepInit();

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
