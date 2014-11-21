#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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

    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}

void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void USART_IT_CONFIG(){
    NVIC_InitTypeDef NVIC_InitStructure;

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

volatile xSemaphoreHandle serial_tx_wait_sem = NULL;
volatile xQueueHandle serial_rx_queue = NULL;

void USART1_IRQHandler()
{
    static signed portBASE_TYPE xHigherPriorityWoken;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
        char msg = USART_ReceiveData(USART1);
        if(!xQueueSendToBack(serial_rx_queue, &msg, portMAX_DELAY));
    }
}

void send_byte(char ch){
    while(!xSemaphoreTake(serial_tx_wait_sem, portMAX_DELAY));
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
    xSemaphoreGive(serial_tx_wait_sem);
}

char recv_byte(){
    char msg;
    USART_IT_CONFIG(USART1, USART_IT_RXNE, ENABLE);
    while(!xQueueReceive(serial_rx_queue,&msg,portMAX_DELAY));
    return msg;
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

    vSemaphoreCreateBinary(serial_tx_wait_sem);
    serial_rx_queue = xQueueCreate(1,sizeof(char));

    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    USART_IT_CONFIG();
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

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;

    GPIO_Init(GPIOG, &GPIO_InitStructure);
    prvInit();
    fs_init();
    fio_init();

    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);

    xTaskCreate(command_prompt, (char *) "cli", 512,
            NULL, tskIDLE_PRIORITY + 2, NULL);
    register_romfs("romfs", &_sromfs);
    register_devfs();

    create_game_tasks();
    //Call Scheduler
    vTaskStartScheduler();
}

