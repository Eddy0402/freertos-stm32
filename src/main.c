#include "main.h"
#include "stm32f4xx_gpio.h"

extern const unsigned char _sromfs;

#define LED_GPIO_PORT GPIOG
#define LED_GPIO_PIN  GPIO_Pin_13

void RCC_Configuration(void)
{
    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);

}

void prvInit(){
    //LCD init
    LCD_Init();
    IOE_Config();
    LTDC_Cmd( ENABLE );

    LCD_LayerInit();
    LCD_SetLayer( LCD_FOREGROUND_LAYER );
    LCD_Clear( LCD_COLOR_BLACK );
    LCD_SetTextColor( LCD_COLOR_WHITE );

    //Button
    STM_EVAL_PBInit( BUTTON_USER, BUTTON_MODE_GPIO );

    RCC_Configuration();
    GPIO_Configuration();
}

//Main Function
int main(void)
{
    prvInit();
    while(1){
        LED_GPIO_PORT->ODR   ^= LED_GPIO_PIN;
        for(int i = 1;i < 10000000;++i);
    }

//    LED_GPIO_PORT->BSRRL  = LED_GPIO_PIN;   // On : Bit set reset register low
//    LED_GPIO_PORT->BSRRH  = LED_GPIO_PIN; // Off : Bit set reset register high
//    LED_GPIO_PORT->ODR   ^= LED_GPIO_PIN; // Toggle: Output data register
}

