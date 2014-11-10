#include "main.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

extern const unsigned char _sromfs;

#define LED_GPIO_PORT GPIOG
#define LED_GPIO_PIN  GPIO_Pin_13

void RCC_Configuration(void)
{
    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure2;
    GPIO_InitTypeDef GPIO_InitStructure3;

    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure2);

    GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure3.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure3.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure3);
}

void IT_Configuration(void){
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0); // PA0

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;

    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void){
    if(EXTI_GetFlagStatus(EXTI_Line0) != RESET){
        LED_GPIO_PORT->ODR ^= LED_GPIO_PIN;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
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
    IT_Configuration();
}

//Main Function
int main(void)
{
    prvInit();
    while(1){
        for(int i = 0;i < 1000000;++i);
        GPIOA -> ODR ^= GPIO_Pin_1;
    }

//    LED_GPIO_PORT->BSRRL  = LED_GPIO_PIN;   // On : Bit set reset register low
//    LED_GPIO_PORT->BSRRH  = LED_GPIO_PIN; // Off : Bit set reset register high
//    LED_GPIO_PORT->ODR   ^= LED_GPIO_PIN; // Toggle: Output data register
}

