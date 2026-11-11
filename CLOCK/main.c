/* ============================================================
 * 基于 AT89C51 + LCD1602 的多功能电子钟
 * ------------------------------------------------------------
 * 功能：
 *   1. 时、分、秒显示与按键调整；日期显示与调整；
 *   2. 秒表：100ms 精度，支持启动、暂停、继续、停止、清零
 *   3. 整点响铃：五短一长循环鸣叫，直到按下最左边的按键（K1）才停止
 *   4. 上电/开始仿真时，时钟自动使用编译时刻的真实时间、日期作为初值
 * ------------------------------------------------------------
 * 硬件连接：
 *   LCD1602：数据口 P0，RS=P2.0，RW=P2.1，E=P2.2
 *   按键  ：K1~K4 = P2.3~P2.6，低电平有效（P2.7 未使用）
 *   蜂鸣器：P2.7，PNP 三极管，低电平响
 *   晶振   ：12MHz（如用 11.0592MHz，修改下面的 FOSC）
 * ------------------------------------------------------------
 * 按键说明：
 *   时钟显示：K1=秒表   K2=调时间  K3=调日期
 *   调时间  ：K1=保存返回  K2=选择时/分/秒  K3=加  K4=减
 *   调日期  ：K1=保存返回  K2=选择年/月/日  K3=加  K4=减
 *   秒表    ：K1=返回  K2=启动/暂停/继续  K3=停止  K4=清零
 *   整点响铃：只有最左边的按键 K1 能停止，其余按键在响铃期间被屏蔽
 * ============================================================ */

#include <reg52.h>
#include <intrins.h>

#define uchar unsigned char
#define uint  unsigned int

/* ---------------- 晶振与定时器 ---------------- */
#define FOSC        12000000UL      /* 12MHz；11.0592MHz 改为 11059200UL */
#define T0_50MS     (65536UL - (FOSC / 12UL * 50UL / 1000UL))
#define T0_H        ((uchar)(T0_50MS >> 8))
#define T0_L        ((uchar)(T0_50MS & 0xFF))

/* ---------------- LCD1602 引脚 ---------------- */
#define LCD_DATA    P0
sbit LCD_RS = P2^0;
sbit LCD_RW = P2^1;
sbit LCD_EN = P2^2;

/* ---------------- 按键 ---------------- */
/* K1~K4 = P2.3~P2.6，按下为低电平；P2.7 未接按键 */
#define KEY_MASK    0x78
#define KEY1        0x70
#define KEY2        0x68
#define KEY3        0x58
#define KEY4        0x38
/* 最左边的按键 K1（P2.3）是整点响铃的唯一停止键：
 * 若实际电路中按键排列相反（最左边是 K4），把下面改成 4 即可。*/
#define KEY_ALARM_STOP  1

/* ---------------- 蜂鸣器 ---------------- */
sbit BEEP = P2^7;                   /* PNP：0 响，1 停 */

/* ---------------- 工作模式 ---------------- */
#define MODE_CLOCK      0
#define MODE_SET_TIME   1
#define MODE_SET_DATE   2
#define MODE_STOPWATCH  3

/* ---------------- 秒表状态 ---------------- */
#define SW_STOP         0
#define SW_RUN          1
#define SW_PAUSE        2

/* ---------------- 上电初始时间/日期 ----------------
 * 默认用编译时刻（__TIME__ / __DATE__）作为初值：每次重新编译后再开始仿真，
 * 显示屏上显示的就是编译时的真实时间与日期，而不是固定的 12:00:00 / 2026-01-01。
 * 若需要固定初值，把下面的 USE_BUILD_TIME 改为 0 并修改 DEFAULT_* 即可*/
#define USE_BUILD_TIME  1
#define DEFAULT_HOUR    12
#define DEFAULT_MINUTE  0
#define DEFAULT_SECOND  0
#define DEFAULT_YEAR    26
#define DEFAULT_MONTH   1
#define DEFAULT_DAY     1

/* ---------------- 全局变量 ---------------- */
volatile uchar hour = DEFAULT_HOUR, minute = DEFAULT_MINUTE, second = DEFAULT_SECOND;
volatile uchar year = DEFAULT_YEAR, month = DEFAULT_MONTH, day = DEFAULT_DAY;   /* 20xx 年 */
volatile uchar clock_tick50 = 0;                /* 50ms 计数，0~19 */
volatile uchar display_div = 0;                 /* 100ms 显示分频 */
volatile uchar flag_100ms = 0;
volatile uchar key_tick = 0;                    /* 50ms 按键长按节拍 */

volatile uchar sw_state = SW_STOP;
volatile uint  sw_time = 0;                     /* 0.1s 为单位 */
volatile uchar sw_div = 0;                      /* 50ms 分频 */

volatile uchar alarm_active = 0;
volatile uchar alarm_seg = 0;
volatile uchar alarm_timer = 0;

uchar mode = MODE_CLOCK;
uchar set_index = 0;

uchar lcd_buf[16];


/* 月份天数，闰年 2 月单独处理 */
uchar code month_days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

/* 整点响铃：五短 + 一长，单位 50ms
 * 共 12 段：5 组“响 150ms / 停 150ms”，第 11 段为长鸣 800ms，
 * 第 12 段留 400ms 间隔，然后从头循环，直到按下 K1。*/
#define ALARM_SEGS  12
uchar code alarm_dur[ALARM_SEGS] = {3,3,3,3,3,3,3,3,3,3,16,8};
uchar code alarm_lvl[ALARM_SEGS] = {1,0,1,0,1,0,1,0,1,0,1,0};

/* ---------------- 函数声明 ---------------- */
void DelayMs(uint ms);
void LcdWriteCmd(uchar cmd);
void LcdWriteData(uchar dat);
void LcdInit(void);
void LcdSetCursor(uchar row, uchar col);
void LcdFlush(uchar row);
void BufClear(void);
void BufStr(uchar pos, char *s);
void BufNum2(uchar pos, uchar n);
uchar DaysInMonth(uchar m);
void DateNext(void);
void AdjustTime(uchar idx, uchar up);
void AdjustDate(uchar idx, uchar up);
void AlarmStart(void);
void AlarmStop(void);
uchar KeyScan(void);
void HandleKey(uchar key);
void DisplayClock(void);
void DisplaySetTime(void);
void DisplaySetDate(void);
void DisplayStopwatch(void);
void UpdateDisplay(void);
void DateTimeFromBuild(void);

/* ---------------- 上电时间/日期：取自编译时刻 ---------------- */
#if USE_BUILD_TIME
void DateTimeFromBuild(void)
{
    char code *t = __TIME__;        /* 编译时刻，格式 "HH:MM:SS" */
    char code *d = __DATE__;        /* 编译日期，格式 "Mmm dd yyyy"，如 "Nov 09 2026" */
    uchar i, dim;
    uint  v;                        /* 年份要按 2026 累加，必须用 16 位 */

    /* ---- 时间 ---- */
    hour   = (uchar)((t[0] - '0') * 10 + (t[1] - '0'));
    minute = (uchar)((t[3] - '0') * 10 + (t[4] - '0'));
    second = (uchar)((t[6] - '0') * 10 + (t[7] - '0'));
    clock_tick50 = 0;               /* 从整秒开始计时 */

    /* ---- 月份：按英文缩写比较 ---- */
    if (d[0] == 'J') {
        if (d[1] == 'a') month = 1;         /* Jan */
        else if (d[2] == 'n') month = 6;    /* Jun */
        else month = 7;                     /* Jul */
    } else if (d[0] == 'F') {
        month = 2;                          /* Feb */
    } else if (d[0] == 'M') {
        month = (d[2] == 'r') ? 3 : 5;      /* Mar / May */
    } else if (d[0] == 'A') {
        month = (d[1] == 'p') ? 4 : 8;      /* Apr / Aug */
    } else if (d[0] == 'S') {
        month = 9;                          /* Sep */
    } else if (d[0] == 'O') {
        month = 10;                         /* Oct */
    } else if (d[0] == 'N') {
        month = 11;                         /* Nov */
    } else {
        month = 12;                         /* Dec */
    }

    /* ---- 日：跳过空格后取 1~2 位数字（"09" 或 "19"）---- */
    i = 3;
    while (i < 10 && d[i] == ' ')
        i++;
    v = (uint)(d[i] - '0');
    if (d[i + 1] >= '0' && d[i + 1] <= '9') {
        v = v * 10 + (d[i + 1] - '0');
        i++;
    }
    day = (uchar)v;

    /* ---- 年：跳过空格后取四位数字，再取后两位 2026 -> 26 ---- */
    i++;
    while (i < 11 && d[i] == ' ')
        i++;
    v = 0;
    while (i < 11 && d[i] >= '0' && d[i] <= '9') {
        v = v * 10 + (d[i] - '0');
        i++;
    }
    year = (uchar)(v % 100);

    /* ---- 合法性检查 ---- */
    if (month < 1 || month > 12) month = 1;
    if (day < 1) day = 1;
    dim = DaysInMonth(month);
    if (day > dim) day = dim;
}
#endif

/* ---------------- 延时 ---------------- */
void DelayMs(uint ms)
{
    uint i, j;
    for (i = ms; i > 0; i--)
        for (j = 150; j > 0; j--);
}

/* ---------------- LCD1602 驱动 ---------------- */
void LcdWriteCmd(uchar cmd)
{
    LCD_RS = 0;
    LCD_RW = 0;
    LCD_DATA = cmd;
    LCD_EN = 1;
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    LCD_EN = 0;
    DelayMs(1);
}

void LcdWriteData(uchar dat)
{
    LCD_RS = 1;
    LCD_RW = 0;
    LCD_DATA = dat;
    LCD_EN = 1;
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    LCD_EN = 0;
    DelayMs(1);
}

void LcdInit(void)
{
    LCD_RS = 0;
    LCD_RW = 0;
    LCD_EN = 0;
    DelayMs(20);

    /* 上电后按数据手册进行三次功能设置 */
    LCD_DATA = 0x38;
    LCD_EN = 1; DelayMs(5); LCD_EN = 0; DelayMs(5);
    LCD_DATA = 0x38;
    LCD_EN = 1; DelayMs(1); LCD_EN = 0; DelayMs(5);
    LCD_DATA = 0x38;
    LCD_EN = 1; DelayMs(1); LCD_EN = 0; DelayMs(5);

    LcdWriteCmd(0x38);      /* 8 位数据，两行，5x7 点阵 */
    LcdWriteCmd(0x0C);      /* 开显示，关光标，关闪烁 */
    LcdWriteCmd(0x06);      /* 光标自动右移 */
    LcdWriteCmd(0x01);      /* 清屏 */
    DelayMs(2);
}

void LcdSetCursor(uchar row, uchar col)
{
    uchar addr;
    addr = (row == 0) ? col : (0x40 + col);
    LcdWriteCmd(0x80 | addr);
}

void LcdFlush(uchar row)
{
    uchar i;
    LcdSetCursor(row, 0);
    for (i = 0; i < 16; i++)
        LcdWriteData(lcd_buf[i]);
}

/* ---------------- 显示缓冲区 ---------------- */
void BufClear(void)
{
    uchar i;
    for (i = 0; i < 16; i++)
        lcd_buf[i] = ' ';
}

void BufStr(uchar pos, char *s)
{
    while (*s != '\0' && pos < 16)
        lcd_buf[pos++] = *s++;
}

void BufNum2(uchar pos, uchar n)
{
    lcd_buf[pos]     = (uchar)('0' + n / 10);
    lcd_buf[pos + 1] = (uchar)('0' + n % 10);
}

/* ---------------- 日期与时间 ---------------- */
uchar DaysInMonth(uchar m)
{
    if (m == 2 && (year & 0x03) == 0)   /* 2000~2099 年，能被 4 整除即闰年 */
        return 29;
    return month_days[m - 1];
}

void DateNext(void)
{
    uchar dim = DaysInMonth(month);
    day++;
    if (day > dim) {
        day = 1;
        month++;
        if (month > 12) {
            month = 1;
            year++;
            if (year > 99)
                year = 0;
        }
    }
}

void AdjustTime(uchar idx, uchar up)
{
    EA = 0;
    if (idx == 0) {
        if (up) {
            hour++;
            if (hour >= 24) hour = 0;
        } else {
            if (hour == 0) hour = 23;
            else hour--;
        }
    } else if (idx == 1) {
        if (up) {
            minute++;
            if (minute >= 60) minute = 0;
        } else {
            if (minute == 0) minute = 59;
            else minute--;
        }
    } else {
        if (up) {
            second++;
            if (second >= 60) second = 0;
        } else {
            if (second == 0) second = 59;
            else second--;
        }
        clock_tick50 = 0;   /* 调秒后从整秒重新计时 */
    }
    EA = 1;
}

void AdjustDate(uchar idx, uchar up)
{
    uchar dim;
    EA = 0;
    if (idx == 0) {                 /* 年 */
        if (up) {
            if (year >= 99) year = 0;
            else year++;
        } else {
            if (year == 0) year = 99;
            else year--;
        }
        dim = DaysInMonth(month);
        if (day > dim) day = dim;
    } else if (idx == 1) {          /* 月 */
        if (up) {
            if (month >= 12) month = 1;
            else month++;
        } else {
            if (month == 1) month = 12;
            else month--;
        }
        dim = DaysInMonth(month);
        if (day > dim) day = dim;
    } else {                        /* 日 */
        dim = DaysInMonth(month);
        if (up) {
            if (day >= dim) day = 1;
            else day++;
        } else {
            if (day == 1) day = dim;
            else day--;
        }
    }
    EA = 1;
}

/* ---------------- 整点响铃 ---------------- */
void AlarmStart(void)
{
    alarm_seg = 0;
    alarm_timer = alarm_dur[0];
    alarm_active = 1;
    BEEP = 0;
}

void AlarmStop(void)
{
    alarm_active = 0;
    BEEP = 1;
}

/* ---------------- 按键扫描 ---------------- */
uchar KeyScan(void)
{
    static uchar last = 0x78;
    static uchar held = 0;
    static uchar last_tick = 0;
    static uchar repeat_wait = 0;
    static uchar repeat_ok = 0;
    uchar cur;
    uchar key = 0;

    cur = P2 & KEY_MASK;
    if (cur != last) {
        DelayMs(10);                    /* 消抖 */
        cur = P2 & KEY_MASK;
        if (cur != last) {
            last = cur;
            last_tick = key_tick;

            if (cur == KEY1) {
                held = 1;
                repeat_ok = 0;
                key = 1;
            } else if (cur == KEY2) {
                held = 2;
                repeat_ok = 0;
                key = 2;
            } else if (cur == KEY3) {
                held = 3;
                repeat_ok = (mode == MODE_SET_TIME || mode == MODE_SET_DATE);
                repeat_wait = 10;       /* 长按 500ms 后开始连加连减 */
                key = 3;
            } else if (cur == KEY4) {
                held = 4;
                repeat_ok = (mode == MODE_SET_TIME || mode == MODE_SET_DATE);
                repeat_wait = 10;
                key = 4;
            } else {
                held = 0;
                repeat_ok = 0;
                repeat_wait = 0;
            }

            /* 响铃期间不允许长按连加连减 */
            if (alarm_active)
                repeat_ok = 0;
        }
        return key;
    }

    /* 只有调时/调日期时，K3、K4 支持长按连加/连减 */
    if (held != 0 && repeat_ok && key_tick != last_tick) {
        last_tick = key_tick;
        if (repeat_wait > 0) {
            repeat_wait--;
        } else {
            key = held;
            repeat_wait = 1;            /* 之后每 100ms 重复一次 */
        }
    }

    return key;
}

void HandleKey(uchar key)
{
    /* 整点响铃只能由最左边的按键 K1 停止，其他按键在响铃期间无效 */
    if (alarm_active) {
        if (key == KEY_ALARM_STOP)
            AlarmStop();
        return;
    }

    switch (mode) {
    case MODE_CLOCK:
        if (key == 1) {
            mode = MODE_STOPWATCH;
        } else if (key == 2) {
            mode = MODE_SET_TIME;
            set_index = 0;
        } else if (key == 3) {
            mode = MODE_SET_DATE;
            set_index = 0;
        }
        break;

    case MODE_SET_TIME:
        if (key == 1) {
            mode = MODE_CLOCK;          /* 保存并返回 */
        } else if (key == 2) {
            set_index++;
            if (set_index > 2) set_index = 0;
        } else if (key == 3) {
            AdjustTime(set_index, 1);
        } else if (key == 4) {
            AdjustTime(set_index, 0);
        }
        break;

    case MODE_SET_DATE:
        if (key == 1) {
            mode = MODE_CLOCK;          /* 保存并返回 */
        } else if (key == 2) {
            set_index++;
            if (set_index > 2) set_index = 0;
        } else if (key == 3) {
            AdjustDate(set_index, 1);
        } else if (key == 4) {
            AdjustDate(set_index, 0);
        }
        break;

    case MODE_STOPWATCH:
        if (key == 1) {
            mode = MODE_CLOCK;
        } else if (key == 2) {
            EA = 0;
            if (sw_state == SW_RUN) {
                sw_state = SW_PAUSE;
            } else if (sw_state == SW_PAUSE) {
                sw_state = SW_RUN;
            } else {
                sw_time = 0;
                sw_div = 0;
                sw_state = SW_RUN;
            }
            EA = 1;
        } else if (key == 3) {
            EA = 0;
            if (sw_state != SW_STOP)
                sw_state = SW_STOP;
            EA = 1;
        } else if (key == 4) {
            EA = 0;
            sw_state = SW_STOP;
            sw_time = 0;
            sw_div = 0;
            EA = 1;
        }
        break;

    default:
        mode = MODE_CLOCK;
        break;
    }
}

/* ---------------- 显示 ---------------- */
void DisplayClock(void)
{
    uchar h, m, s, y, mo, d;

    EA = 0;
    h = hour; m = minute; s = second;
    y = year; mo = month; d = day;
    EA = 1;

    BufClear();
    BufStr(0, "TIME ");
    BufNum2(5, h);
    lcd_buf[7] = ':';
    BufNum2(8, m);
    lcd_buf[10] = ':';
    BufNum2(11, s);
    LcdFlush(0);

    BufClear();
    if (alarm_active) {
        BufStr(0, "ALARM! PRESS K1 ");
    } else {
        BufStr(0, "DATE ");
        BufStr(5, "20");
        BufNum2(7, y);
        lcd_buf[9] = '-';
        BufNum2(10, mo);
        lcd_buf[12] = '-';
        BufNum2(13, d);
    }
    LcdFlush(1);
}

void DisplaySetTime(void)
{
    uchar h, m, s;

    EA = 0;
    h = hour; m = minute; s = second;
    EA = 1;

    BufClear();
    if (set_index == 0)      BufStr(0, "TIME SET: HOUR");
    else if (set_index == 1) BufStr(0, "TIME SET: MINUTE");
    else                     BufStr(0, "TIME SET: SECOND");
    LcdFlush(0);

    BufClear();
    BufNum2(0, h);
    lcd_buf[2] = ':';
    BufNum2(3, m);
    lcd_buf[5] = ':';
    BufNum2(6, s);
    LcdFlush(1);
}

void DisplaySetDate(void)
{
    uchar y, mo, d;

    EA = 0;
    y = year; mo = month; d = day;
    EA = 1;

    BufClear();
    if (set_index == 0)      BufStr(0, "DATE SET: YEAR  ");
    else if (set_index == 1) BufStr(0, "DATE SET: MONTH ");
    else                     BufStr(0, "DATE SET: DAY   ");
    LcdFlush(0);

    BufClear();
    BufStr(0, "20");
    BufNum2(2, y);
    lcd_buf[4] = '-';
    BufNum2(5, mo);
    lcd_buf[7] = '-';
    BufNum2(8, d);
    LcdFlush(1);
}

void DisplayStopwatch(void)
{
    uint t;
    uchar st, mi, se, th;

    EA = 0;
    t = sw_time;
    st = sw_state;
    EA = 1;

    BufClear();
    if (st == SW_RUN)        BufStr(0, "STOPWATCH  RUN  ");
    else if (st == SW_PAUSE) BufStr(0, "STOPWATCH PAUSE ");
    else                     BufStr(0, "STOPWATCH  STOP ");
    LcdFlush(0);

    mi = (uchar)(t / 600);          /* 0.1s -> 分 */
    se = (uchar)((t / 10) % 60);    /* 秒 */
    th = (uchar)(t % 10);           /* 0.1s */

    BufClear();
    lcd_buf[4]  = (uchar)('0' + mi / 10);
    lcd_buf[5]  = (uchar)('0' + mi % 10);
    lcd_buf[6]  = ':';
    lcd_buf[7]  = (uchar)('0' + se / 10);
    lcd_buf[8]  = (uchar)('0' + se % 10);
    lcd_buf[9]  = '.';
    lcd_buf[10] = (uchar)('0' + th);
    LcdFlush(1);
}

void UpdateDisplay(void)
{
    /* 响铃期间无论处于哪个界面，都提示按 K1 停止 */
    if (alarm_active) {
        DisplayClock();
        return;
    }

    switch (mode) {
    case MODE_CLOCK:     DisplayClock();     break;
    case MODE_SET_TIME:  DisplaySetTime();   break;
    case MODE_SET_DATE:  DisplaySetDate();   break;
    case MODE_STOPWATCH: DisplayStopwatch(); break;
    default:             DisplayClock();     break;
    }
}

/* ---------------- 定时器 ---------------- */
void Timer0Init(void)
{
    TMOD &= 0xF0;
    TMOD |= 0x01;       /* Timer0 方式1，16 位定时 */
    TH0 = T0_H;
    TL0 = T0_L;
    ET0 = 1;
    EA = 1;
    TR0 = 1;
}

void Timer0_ISR(void) interrupt 1
{
    TH0 = T0_H;
    TL0 = T0_L;
    key_tick++;             /* 50ms 按键长按节拍 */

    /* ---- 整点响铃状态机：五短 + 一长，循环直到按下 K1 ---- */
    if (alarm_active) {
        if (alarm_timer > 0)
            alarm_timer--;
        if (alarm_timer == 0) {
            alarm_seg++;
            if (alarm_seg >= ALARM_SEGS)
                alarm_seg = 0;      /* 一轮结束，立即重新开始下一轮 */
            alarm_timer = alarm_dur[alarm_seg];
            BEEP = alarm_lvl[alarm_seg] ? 0 : 1;
        }
    }

    /* ---- 100ms 显示刷新 ---- */
    if (++display_div >= 2) {
        display_div = 0;
        flag_100ms = 1;
    }

    /* ---- 秒表：100ms 精度 ---- */
    if (sw_state == SW_RUN) {
        if (++sw_div >= 2) {
            sw_div = 0;
            if (sw_time < 59999)
                sw_time++;
            else
                sw_state = SW_STOP;
        }
    } else {
        sw_div = 0;
    }

    /* ---- 时钟：50ms × 20 = 1s ---- */
    if (++clock_tick50 >= 20) {
        clock_tick50 = 0;

        if (++second >= 60) {
            second = 0;
            if (++minute >= 60) {
                minute = 0;
                if (++hour >= 24) {
                    hour = 0;
                    DateNext();
                }
                AlarmStart();       /* 整点报时 */
            }
        }
    }
}

/* ---------------- 主函数 ---------------- */
void main(void)
{
    uchar key;

    BEEP = 1;
    DelayMs(50);
    LcdInit();

    /* 上电短鸣，便于确认蜂鸣器硬件是否工作 */
    BEEP = 0;
    DelayMs(1000);
    BEEP = 1;

    /* 用编译时刻作为时钟初值：开始测试/仿真时显示的就是当前真实时间与日期 */
#if USE_BUILD_TIME
    DateTimeFromBuild();
#endif

    Timer0Init();
    UpdateDisplay();

    /* 主循环：扫描按键 + 每 100ms 刷新一次显示 */
    while (1) {
        key = KeyScan();
        if (key != 0) {
            HandleKey(key);
            flag_100ms = 1;         /* 按键后立即刷新，响应更及时 */
        }
        if (flag_100ms) {
            flag_100ms = 0;
            UpdateDisplay();
        }
    }
}
