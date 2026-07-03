/**********************************************
 * 按键扫描 — 非阻塞版本
 *
 * 所有按键使用统一的消抖计数器, 由主循环定时 (~100ms) 驱动
 * 无内部 Delay_ms, 不阻塞任何其他操作
 **********************************************/
#include "key.h"

/* ---- 消抖参数 ---- */
#define DEBOUNCE_CNT  2    /* 连续2次(约200ms)按下=确认按下 */
#define HOLD_CNT      10   /* 连续10次(约1秒)=长按 */
#define REPEAT_CNT    15   /* 长按后每~1.5秒连发 */

/* ---- 统一按键状态 ---- */
typedef struct {
    uint8_t  debounce;    /* 消抖计数器 (按下时递增) */
    uint8_t  triggered;   /* 本次已触发? 1=是 (防止重复触发) */
    uint8_t  hold_fired;  /* 长按已触发? */
    uint16_t hold_timer;  /* 长按计时 */
    uint8_t  last_state;  /* 上次消抖后状态: 0=松开 1=按下 */
} KeyState;

static KeyState ks_mode, ks_add, ks_ok, ks_max30102;

/*==========================================================================
 * 通用按键检测 — 非阻塞
 * state:   按键状态结构
 * pin:     引脚号
 * port:    GPIO端口
 * is_add:  1=ADD键(支持长按连发), 0=普通键
 * 返回:    0=无事件, 1=短按触发, 2=长按连发
 *==========================================================================*/
static uint8_t key_scan(KeyState *ks, uint16_t pin, GPIO_TypeDef *port, uint8_t is_add)
{
    uint8_t raw = (GPIO_ReadInputDataBit(port, pin) == 0) ? 1 : 0;  /* 1=按下 */

    if (raw) {
        /* 按下 → 消抖计数递增 */
        if (ks->debounce < 255) ks->debounce++;
    } else {
        /* 松开 → 全部复位 */
        ks->debounce   = 0;
        ks->triggered  = 0;
        ks->hold_fired = 0;
        ks->hold_timer = 0;
        ks->last_state = 0;
        return 0;
    }

    /* 消抖未完成 */
    if (ks->debounce < DEBOUNCE_CNT) return 0;

    /* ---- 消抖完成, 按键确认按下 ---- */

    if (!is_add) {
        /* 普通键: 只触发一次 (下降沿) */
        if (!ks->triggered) {
            ks->triggered  = 1;
            ks->last_state = 1;
            return 1;
        }
        return 0;
    }

    /* ADD键: 首次触发起效 */
    if (!ks->triggered) {
        ks->triggered  = 1;
        ks->hold_timer = DEBOUNCE_CNT;
        ks->last_state = 1;
        return 1;
    }

    /* ADD键: 长按检测 */
    ks->hold_timer++;
    if (!ks->hold_fired && ks->hold_timer >= HOLD_CNT) {
        ks->hold_fired = 1;
        ks->hold_timer = HOLD_CNT;
        return 2;  /* 长按首次触发 */
    }
    if (ks->hold_fired && ks->hold_timer >= REPEAT_CNT) {
        ks->hold_timer = HOLD_CNT;
        return 2;  /* 长按连发 */
    }

    return 0;
}

/*==========================================================================
 * 对外接口
 *==========================================================================*/
uint8_t Key_Mode_Scan(void)
{
    return (key_scan(&ks_mode, KEY_MODE, GPIOA, 0) == 1) ? 1 : 0;
}

uint8_t Key_Mode_Hold(void)
{
    return (key_scan(&ks_mode, KEY_MODE, GPIOA, 1) == 2) ? 1 : 0;
}

uint8_t Key_Add_Scan(void)
{
    uint8_t r = key_scan(&ks_add, KEY_ADD, GPIOA, 1);
    return (r >= 1) ? 1 : 0;  /* 短按和长按连发都当作有效 */
}

uint8_t Key_Add_Hold(void)
{
    uint8_t r = key_scan(&ks_add, KEY_ADD, GPIOA, 1);
    return (r == 2) ? 1 : 0;  /* 仅长按连发 */
}

uint8_t Key_OK_Scan(void)
{
    return (key_scan(&ks_ok, KEY_OK, GPIOA, 0) == 1) ? 1 : 0;
}

uint8_t Key_Max30102_Scan(void)
{
    return (key_scan(&ks_max30102, KEY_MAX30102, GPIOA, 0) == 1) ? 1 : 0;
}
