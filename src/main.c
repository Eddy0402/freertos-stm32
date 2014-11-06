#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "game_task.h"
#include "shell.h"
#include "fio.h"
#include "filesystem.h"
#include "romfs.h"
#include "clib.h"

/* Default user name */
#define USER_NAME "Eddy"

extern const unsigned char _sromfs;

void RCC_Configuration(void)
{
    /* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);//USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);//USART1_RX
}

void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configuration 
     * USARTx configured as follow:
     *- BaudRate = 9600 baud
     *- Word Length = 8 Bits
     *- One Stop Bit
     *- No parity
     *- Hardware flow control disabled (RTS and CTS signals)
     *- Receive and transmit enabled
     **/
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
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
    USART1_Configuration();
}

void send_byte(char ch){
    // TODO: implementation
}

char recv_byte(){
    // TODO: implementation
}

void command_prompt(void *pvParameters){
    char buf[128];
    char *argv[20];
    char hint[] = USER_NAME "@STM32:~$ ";

    fio_printf(1,"\rWelcome to FreeRTOS Shell\r\n");
    while(1){
        fio_printf(1, "%s", hint);
        fio_read(0, buf, 127);
        int n = parse_command(buf, argv);

        /* will return pointer to the command function */
        cmdfunc *fptr=do_command(argv[0]);
        if(fptr != NULL){
            fptr(n, argv);
        }else{
            fio_printf(2, "\r\n\"%s\" command not found.\r\n", argv[0]);
        }
    }
}

//Main Function
int main(void)
{
    prvInit();
    fs_init();
    fio_init();

    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);

    xTaskCreate(command_prompt, (char *) "cli", 512,
            NULL, tskIDLE_PRIORITY + 1, NULL);
    register_romfs("romfs", &_sromfs);
    register_devfs();

    create_game_tasks();

    //Call Scheduler
    vTaskStartScheduler();
}

