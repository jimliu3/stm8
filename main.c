/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8s.h"
#include "stm8s_conf.h"
#include "uart.h"
#include "rc522.h"
#include "lf_send.h"
#include <string.h>
#include <stdlib.h>

#define TEST_MODE_RFID   0      // RFID test mode
#define TEST_MODE_LF     1      // LF (125kHz / 433MHz) test mode

#if TEST_MODE_RFID
u8 Tx_Buffer[] = "RFID---test";
#endif

#define MCU_REG_NUM      26
const uint8_t mcu_user_config[MCU_REG_NUM] =
{
    0xA5,0x5A,0x01,0x7D,0x80,0x00,0x00,0x01,
    0xC3,0x3A,0x0C,0x01,0x00,0x15,0x00,0x01,
    0x01,0x81,0x32,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,
};

unsigned char Time_1ms = 0;


void Clock_Config(void)
{

    CLK_HSICmd(ENABLE);

    while (CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == RESET);

    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI,    ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
}

static void GPIO_Config(void)
{
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST); 
    GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);

    GPIO_WriteLow(GPIOD, GPIO_PIN_2);
}

void TIM2_Init(void)
{
    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM2_PRESCALER_16, 100);
    TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
    TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_HIGH);
    TIM2_OC2PreloadConfig(ENABLE);
    TIM2_SetCounter(0x0000);
    TIM2_Cmd(ENABLE);


    ITC->ISPR4 &= (uint8_t)(~0x0C);
}



@far @interrupt void tim2_irqhandler(void)
{
    TIM2_ClearITPendingBit(TIM2_IT_UPDATE);

    Time_1ms++;
    if (Time_1ms >= 10)
    {
        Time_1ms = 0;

        
        if ((LF_ENABLE == 1) && (LF_Send_Tim > 0))
        {
            LF_Send_Tim--;
        }
        else
        {
            LF_Send_Tim = 0;
        }
    }
}


#if TEST_MODE_RFID
void Poll_RFID(void)
{
    uint8_t rfid_set = 0;
    unsigned char rc522_SN[4];
    const char hex_chars[] = "0123456789ABCDEF";
    char hex_out[4];
    uint8_t i;

    showcard(Tx_Buffer, &rfid_set, rc522_SN);
    Reset_RC522();

    if (rfid_set == 1) {
        UART2_SendString("\nRFID UID: ", 11);
        for (i = 0; i < 4; i++) {
            hex_out[0] = hex_chars[(rc522_SN[i] >> 4) & 0x0F];
            hex_out[1] = hex_chars[rc522_SN[i] & 0x0F];
            hex_out[2] = ' ';
            UART2_SendString((unsigned char*)hex_out, 3);
        }
        UART2_SendString("\t", 1);
    }

    Delay_ms(200);
}
#endif

#if TEST_MODE_LF
void Test_LF_Simple(void)
{	
    /* sent 125kHz */
    LF_SendData(0xc3, 0x3a, PATTREN_BIT, LF_SEND_CH1, 0x01, 0x01);
    Delay_ms(100);
}
#endif


void main(void)
{
    uint8_t cfg_idx;

    Clock_Config();
    GPIO_Config();
    TIM4_DeInit();
    TIM4_Init();
    Uart_Init();
    InitRc522();
    Delay_ms(100);

    LF_ClockOccurs(125);
    for (cfg_idx = 0; cfg_idx < MCU_REG_NUM; cfg_idx++) {
        Set_Buff[cfg_idx] = mcu_user_config[cfg_idx];
    }
    LF_PLL_SET(LF_PLL);

    Delay_InIt(16);
    TIM2_Init();
    enableInterrupts();

    UART2_SendStr("system start!");

    while (1)
    {
#if TEST_MODE_RFID
        Poll_RFID();
#elif TEST_MODE_LF
        Test_LF_Simple();
#else
        #error "No test mode selected"
#endif
    }
}