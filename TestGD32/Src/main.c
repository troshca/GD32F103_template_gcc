#include "gd32f10x.h"
#include "systick.h"
#include <stdio.h>

#include <SEGGER_RTT.h>


static uint8_t logBuffer[32];
char str1[30];

void usart_string_transmit(uint32_t usart_periph, uint8_t* pData, uint16_t Size);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{ 
    systick_config();
    //RTT
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(
    1, "Log", logBuffer, sizeof(logBuffer), 
    SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    printf("Hello, It's Segger RTT\n");

    /* enable the LED clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* configure LED GPIO port */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

    gpio_bit_reset(GPIOC, GPIO_PIN_13);
    gpio_bit_set(GPIOC, GPIO_PIN_13);

    usart_config();

    uint32_t data = 0;
    while(1)
    {
        printf("data: %d", data);
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        printf("set\n\r");
        delay_1ms(500);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        printf("reset\n\r");
        delay_1ms(500);
    }
}

/*! 
    \brief      USART1 configure 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usart_config(void)
{
    usart_deinit(USART1);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    
    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    //usart_halfduplex_enable(USART1);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);
}

/*!
    \brief      USART transmit data function
    \param[in]  usart_periph: USARTx(x=0,1,2)/UARTx(x=3,4)
    \param[in]  pData: Pointer to data buffer.
    \param[in]  Size: Amount of data to be sent.
    \param[out] none
    \retval     none
*/
void usart_string_transmit(uint32_t usart_periph, uint8_t* pData, uint16_t Size)
{
    uint16_t ind = 0;
    while (ind < Size)
    {
        usart_data_transmit(usart_periph, *(uint8_t*)(pData + ind));
        ind++;
        while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
    }
}

