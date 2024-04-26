#include "gd32f10x.h"
#include "systick.h"
#include <stdio.h>

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    systick_config();
    /* enable the LED clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    /* configure LED GPIO port */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
    gpio_bit_set(GPIOC, GPIO_PIN_13);

    while(1)
    {
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        delay_1ms(1000);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(1000);
    }
}