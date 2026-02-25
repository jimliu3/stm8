/* MAIN.C file
 *
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8s.h"
#include "stm8s_conf.h"
#include "uart.h"
#include "rc522.h"
#include "lf_send.h"

/* ================= State ================= */

enum pke_oper_state {
    PKE_OPER_STA_POWER_OFF = 0,
    PKE_OPER_STA_POWER_ON,
    PKE_OPER_STA_IDLE,
    PKE_OPER_STA_LEARN,
    PKE_OPER_STA_IGN_ON,
    PKE_OPER_STA_BUSY,
    PKE_OPER_STA_ERROR
};

volatile struct PKE_config {
    uint8_t key_num;
    uint8_t rc522_num;
    volatile uint8_t oper_state;
    volatile uint8_t power_event_flag;
    volatile uint8_t learn_event_flag;
} TJTW_PKE;

/* ================= GPIO Macros ================= */

#define BR_LIGHT_ON()       GPIO_WriteHigh(GPIOD, GPIO_PIN_3)
#define BR_LIGHT_OFF()      GPIO_WriteLow(GPIOD, GPIO_PIN_3)

#define BZ_ON()             GPIO_WriteHigh(GPIOB, GPIO_PIN_1)
#define BZ_OFF()            GPIO_WriteLow(GPIOB, GPIO_PIN_1)

#define LP_RIGHT_ON()       GPIO_WriteHigh(GPIOB, GPIO_PIN_3)
#define LP_RIGHT_OFF()      GPIO_WriteLow(GPIOB, GPIO_PIN_3)

#define MOTOR2_ON()         GPIO_WriteHigh(GPIOC, GPIO_PIN_2)
#define MOTOR2_OFF()        GPIO_WriteLow(GPIOC, GPIO_PIN_2)

#define MOTOR1_ON()         GPIO_WriteHigh(GPIOB, GPIO_PIN_0)
#define MOTOR1_OFF()        GPIO_WriteLow(GPIOB, GPIO_PIN_0)

#define MOTOR_FWD()         do{ MOTOR1_ON();  MOTOR2_OFF(); }while(0)
#define MOTOR_REV()         do{ MOTOR1_OFF(); MOTOR2_ON();  }while(0)
#define MOTOR_STOP()        do{ MOTOR1_OFF(); MOTOR2_OFF(); }while(0)

/* ================= EEPROM Layout ================= */

#define RC522KEY_COUNT_ADDR   0x00004100
#define RC522KEY_START_ADDR   0x00004110
#define RC522KEY_SIZE         4
#define MAX_RC522KEY_NUM      5

/* ================= UART Test Buffer ================= */

u8 Tx_Buffer[] = "RFID---test";
#define BufferSize (countof(Tx_Buffer)-1)

/* ================= Unlock / Auth Window ================= */

/* 1 = motor has moved forward (unlocked), so reverse is required on power off */
static uint8_t g_is_unlocked = 0;

/* 10s authentication window (10ms tick based) */
static volatile uint8_t  g_authwin_active = 0;   /* 1 = window active */
static volatile uint16_t g_authwin_cnt10ms = 0;  /* remaining 10ms ticks */

/* ================= Clock ================= */

static void Clock_Config(void)
{
    /* enable internal HSI clock(16MHZ) */
    CLK_HSICmd(ENABLE);

    /* make sure internal clock(HSI) is stable */
    while (CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == RESET);

    /* set HSI DIV(High speed internal clock prescaler: 1) */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

    /* SPI clock */
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
}

/* ================= EEPROM Helpers ================= */

static void Clear_PKE_EEPROM(void)
{
    uint8_t i;

    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    for (i = 0; i < 16; i++)
        FLASH_ProgramByte(0x00004000 + i, 0x00);

    FLASH_ProgramByte(RC522KEY_COUNT_ADDR, 0x00);

    for (i = 0; i < 16; i++)
        FLASH_ProgramByte(RC522KEY_START_ADDR + i, 0x00);

    FLASH_Lock(FLASH_MEMTYPE_DATA);
}

static u8 Rebuild_KeyCount(void)
{
    u8 i, real_count = 0;
    uint16_t addr;

    for (i = 0; i < MAX_RC522KEY_NUM; i++)
    {
        addr = RC522KEY_START_ADDR + (i * RC522KEY_SIZE);

        FLASH_Unlock(FLASH_MEMTYPE_DATA);
        if (FLASH_ReadByte(addr)     != 0x00 ||
            FLASH_ReadByte(addr + 1) != 0x00 ||
            FLASH_ReadByte(addr + 2) != 0x00 ||
            FLASH_ReadByte(addr + 3) != 0x00)
        {
            real_count++;
        }
        FLASH_Lock(FLASH_MEMTYPE_DATA);
    }

    return real_count;
}

static void Write_EEpeomData(unsigned char *rc522)
{
    unsigned char i;
    u8 num;
    u8 match;
    uint16_t write_addr;
    uint16_t read_addr;

    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    num = FLASH_ReadByte(RC522KEY_COUNT_ADDR);
    if (num >= MAX_RC522KEY_NUM)
    {
        num = Rebuild_KeyCount();
        FLASH_ProgramByte(RC522KEY_COUNT_ADDR, num);
        FLASH_Lock(FLASH_MEMTYPE_DATA);
        return;
    }

    match = 0;
    for (i = 0; i < num; i++)
    {
        read_addr = RC522KEY_START_ADDR + (i * RC522KEY_SIZE);
        if ((FLASH_ReadByte(read_addr)     == rc522[0]) &&
            (FLASH_ReadByte(read_addr + 1) == rc522[1]) &&
            (FLASH_ReadByte(read_addr + 2) == rc522[2]) &&
            (FLASH_ReadByte(read_addr + 3) == rc522[3]))
        {
            match = 1;
            (void)match;
            FLASH_Lock(FLASH_MEMTYPE_DATA);
            return;
        }
    }

    write_addr = RC522KEY_START_ADDR + (num * RC522KEY_SIZE);
    for (i = 0; i < RC522KEY_SIZE; i++)
        FLASH_ProgramByte(write_addr + i, rc522[i]);

    num++;
    FLASH_ProgramByte(RC522KEY_COUNT_ADDR, num);

    FLASH_Lock(FLASH_MEMTYPE_DATA);
}

static u8 Check_RC522Key(unsigned char *rc522)
{
    u8 i, j;
    uint16_t read_addr;
    u8 num;
    u8 match;

    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    num = FLASH_ReadByte(RC522KEY_COUNT_ADDR);
    if (num > MAX_RC522KEY_NUM)
        num = Rebuild_KeyCount();

    for (i = 0; i < num; i++)
    {
        match = 1;
        read_addr = RC522KEY_START_ADDR + (i * RC522KEY_SIZE);

        for (j = 0; j < RC522KEY_SIZE; j++)
        {
            if (FLASH_ReadByte(read_addr + j) != rc522[j])
            {
                match = 0;
                break;
            }
        }

        if (match == 1)
        {
            FLASH_Lock(FLASH_MEMTYPE_DATA);
            return 1; /* found */
        }
    }

    FLASH_Lock(FLASH_MEMTYPE_DATA);
    return 0; /* not found */
}

/* ================= Motor Pulse ================= */

static void Motor_Pulse_Fwd_20ms(void)
{
    MOTOR_FWD();
    Delay_ms(20);
    MOTOR_STOP();
}

static void Motor_Pulse_Rev_20ms(void)
{
    MOTOR_REV();
    Delay_ms(20);
    MOTOR_STOP();
}

/* ================= IGN ================= */

static uint8_t IGN_IsOn(void)
{
    return (uint8_t)GPIO_ReadInputPin(GPIOB, GPIO_PIN_5); /* 1=ON */
}

/* ================= PowerOff Unified Exit ================= */

static void Do_PowerOff(uint8_t *pIdleFlag)
{
    /* Reverse motor ONLY if it was previously unlocked */
    if (g_is_unlocked)
    {
        Motor_Pulse_Rev_20ms();
        g_is_unlocked = 0;
    }

    /* Disable breathing PWM and restore GPIO mode */
    TIM2_CCxCmd(TIM2_CHANNEL_2, DISABLE);
    GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
    BR_LIGHT_OFF();

    /* Clear 10s authentication window */
    g_authwin_active  = 0;
    g_authwin_cnt10ms = 0;

    /* Reset IDLE entry flag */
    if (pIdleFlag)
        *pIdleFlag = 0;

    /* Put outputs to safe states */
    BZ_OFF();
    LP_RIGHT_OFF();
    MOTOR_STOP();

    TJTW_PKE.oper_state = PKE_OPER_STA_POWER_OFF;
}

/* ================= GPIO / EXTI ================= */

static void GPIO_Config(void)
{
    GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST); /* D3 BR_LIGHT */
    GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST); /* B3 LP_RIGHT */
    GPIO_Init(GPIOC, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_FAST); /* Motor IN2 */
    GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST); /* Motor IN1 */
    GPIO_Init(GPIOB, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST); /* BZ */

    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_PU_IT);        /* POWER KEY */
    GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);     /* LEARN KEY */
    GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);     /* IGN KEY */

    /* init gpio status */
    BR_LIGHT_OFF();
    LP_RIGHT_OFF();
    MOTOR_STOP();
    BZ_OFF();
}

static void EXTI_Config(void)
{
    /* NOTE:
     * If your power key is "press-to-GND", change to EXTI_SENSITIVITY_FALL_ONLY.
     * If your power key is "press-to-VCC", EXTI_SENSITIVITY_RISE_ONLY is OK.
     */
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
}

/* ================= ISR ================= */

INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
    TJTW_PKE.power_event_flag = 1;
}

/* ================= MAIN ================= */

int main(void)
{
    int i;
    u8 set = 0;
    uint16_t brightness = 0;     /* 0..999? depends on your BR_PWM implementation */
    uint8_t up = 1;
    unsigned char rc522_SN[4];

    uint8_t idle = 0;

    TJTW_PKE.oper_state       = PKE_OPER_STA_POWER_OFF;
    TJTW_PKE.power_event_flag = 0;
    TJTW_PKE.learn_event_flag = 0;

    Clock_Config();
    GPIO_Config();
    EXTI_Config();

    TIM4_DeInit();
    TIM4_Init();

    Uart_Init();
    InitRc522();

    Delay_ms(100);

    TIM2_PWM_Config();

    /* Clear_PKE_EEPROM(); */

    enableInterrupts();

    UART2_SendStr("system start!\r\n");

    while (1)
    {
        switch (TJTW_PKE.oper_state)
        {
            case PKE_OPER_STA_POWER_OFF:
            {
                UART2_SendStr("PKE_OPER_STA_POWER_OFF in!\r\n");

                /* ensure no pending window */
                g_authwin_active  = 0;
                g_authwin_cnt10ms = 0;

                /* safe off before halt */
                BR_LIGHT_OFF();
                LP_RIGHT_OFF();
                BZ_OFF();
                MOTOR_STOP();

                enableInterrupts();
                halt();

                /* wake-up */
                Clock_Config();
                Uart_Init();
                UART2_SendStr("PKE_OPER_STA_POWER_OFF out!\r\n");

                if (TJTW_PKE.power_event_flag)
                {
                    TJTW_PKE.power_event_flag = 0;

                    if (GPIO_ReadInputPin(GPIOA, GPIO_PIN_2))
                        TJTW_PKE.oper_state = PKE_OPER_STA_LEARN;
                    else
                        TJTW_PKE.oper_state = PKE_OPER_STA_POWER_ON;
                }
                break;
            }

            case PKE_OPER_STA_POWER_ON:
            {
                uint8_t seen_key = 0;
                uint8_t auth_ok  = 0;

                UART2_SendStr("PKE_OPER_STA_POWER_ON in!\r\n");

                /* scan card window (keep your original 5s behavior) */
                set = 0;
                for (i = 0; i < 50; i++)
                {
                    Delay_ms(100);

                    showcard(Tx_Buffer, &set, rc522_SN);
                    Reset_RC522();

                    if (set == 1)
                    {
                        seen_key = 1;
                        set = 0;

                        auth_ok = Check_RC522Key(rc522_SN);
                        break;
                    }
                }

                if (!seen_key)
                {
                    /* No key detected -> totally no reaction -> back to POWER_OFF */
                    Do_PowerOff(&idle);
                    break;
                }

                if (!auth_ok)
                {
                    /* Key detected but not in EEPROM -> back to POWER_OFF */
                    Do_PowerOff(&idle);
                    break;
                }

                /* Key OK -> Motor FWD 20ms + start 10s window */
                Motor_Pulse_Fwd_20ms();
                g_is_unlocked = 1;

                g_authwin_active  = 1;
                g_authwin_cnt10ms = 1000; /* 10s / 10ms = 1000 ticks */

                TJTW_PKE.oper_state = PKE_OPER_STA_IDLE;

                UART2_SendStr("KEY OK -> IDLE (authwin=10s)\r\n");
                break;
            }

            case PKE_OPER_STA_IDLE:
            {
                if (idle == 0)
                {
                    UART2_SendStr("PKE_OPER_STA_IDLE in!\r\n");
                    idle = 1;
                    TIM2_CCxCmd(TIM2_CHANNEL_2, ENABLE);
                }

                BR_PWM(&brightness, &up);
                Delay_ms(10);

                /* ===== 10s window check (ONLY after KEY OK) ===== */
                if (g_authwin_active)
                {
                    if (IGN_IsOn())
                    {
                        /* Enter IGN within 10s -> freeze window, go IGN_ON */
                        g_authwin_active  = 0;
                        g_authwin_cnt10ms = 0;
                        TJTW_PKE.oper_state = PKE_OPER_STA_IGN_ON;
                        UART2_SendStr("AuthWin: IGN ON -> IGN_ON\r\n");
                        break;
                    }

                    if (g_authwin_cnt10ms > 0)
                        g_authwin_cnt10ms--;

                    if (g_authwin_cnt10ms == 0)
                    {
                        UART2_SendStr("AuthWin: timeout -> POWER_OFF\r\n");
                        Do_PowerOff(&idle);
                        break;
                    }
                }

                /* manual power key: immediate power off (but only reverse if unlocked) */
                if (TJTW_PKE.power_event_flag)
                {
                    TJTW_PKE.power_event_flag = 0;
                    UART2_SendStr("IDLE: power key -> POWER_OFF\r\n");
                    Do_PowerOff(&idle);
                    break;
                }

                break;
            }

            case PKE_OPER_STA_LEARN:
            {
                UART2_SendStr("PKE_OPER_STA_LEARN in!\r\n");

                /* Stop breathing while learning */
                TIM2_CCxCmd(TIM2_CHANNEL_2, DISABLE);
                GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
                BR_LIGHT_OFF();
                idle = 0;

                for (i = 0; i < 50; i++)
                {
                    BZ_ON();
                    LP_RIGHT_ON();
                    BR_LIGHT_ON();
                    Delay_ms(100);

                    showcard(Tx_Buffer, &set, rc522_SN);
                    Reset_RC522();

                    if (set == 1)
                    {
                        UART2_SendString(Tx_Buffer, 17);
                        set = 0;
                        Write_EEpeomData(rc522_SN);
                        break;
                    }

                    BZ_OFF();
                    LP_RIGHT_OFF();
                    BR_LIGHT_OFF();
                    Delay_ms(100);
                }

                /* Learning done -> go POWER_OFF (do not force motor reverse) */
                g_authwin_active  = 0;
                g_authwin_cnt10ms = 0;

                /* leave outputs safe */
                BZ_OFF();
                LP_RIGHT_OFF();
                BR_LIGHT_OFF();
                MOTOR_STOP();

                TJTW_PKE.oper_state = PKE_OPER_STA_POWER_OFF;
                UART2_SendStr("PKE_OPER_STA_LEARN out!\r\n");
                break;
            }

            case PKE_OPER_STA_IGN_ON:
            {
                /* Keep breathing / system alive while IGN is ON */
                BR_PWM(&brightness, &up);
                Delay_ms(10);

                /* IGN OFF -> POWER_OFF immediately (reverse only if unlocked) */
                if (!IGN_IsOn())
                {
                    UART2_SendStr("IGN_ON: IGN OFF -> POWER_OFF\r\n");
                    Do_PowerOff(&idle);
                    break;
                }

                /* block manual power key while IGN ON */
                if (TJTW_PKE.power_event_flag)
                    TJTW_PKE.power_event_flag = 0;

                break;
            }

            default:
                UART2_SendStr("Default state -> POWER_OFF\r\n");
                Do_PowerOff(&idle);
                break;
        }
    }
}