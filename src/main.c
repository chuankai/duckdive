/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "main.h"
#include <stdbool.h>
#include "stm32f4_discovery.h"
#include "adc.h"
#include <stdlib.h>
#include <sys/socket.h>
#include <netdb.h>

static void SystemClock_Config(void);
			
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
    if (HAL_GetREVID() == 0x1001)
    {
      /* Enable the Flash prefetch */
      __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}

/*
 * Network Thread
 */
#define SERVER_PORT 13469
#define SERVER_HOST "test.ring.com"

osThreadId networkThread;

#define HEART_BEAT_SIGNAL		(0x1)
#define CONNECT_ON_SIGNAL		(0x1 << 1)
#define CONNECT_OFF_SIGNAL		(0x1 << 2)

void Heart_Beat_Signal(void const *arg)
{
	osSignalSet(networkThread, HEART_BEAT_SIGNAL);
}

osSemaphoreId connectAckSem;
int connect_socket;

static void Connect_Thread(void const *argument)
{
	unsigned short seq = *(uint16_t *) argument;

    unsigned short seq_n;
    struct sockaddr_in destAddr;
    struct sockaddr_in fromAddr;
    struct hostent *hostInfo;
    int err;

    hostInfo = gethostbyname("test.ring.com");
    if (hostInfo == NULL) {
    	goto ERROR;
    }

    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(SERVER_PORT);
    destAddr.sin_addr = *(struct in_addr *) hostInfo->h_addr;

    connect_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (connect_socket < 0) {
            goto ERROR;
    }

    seq_n = htons(seq);
    err = sendto (connect_socket, &seq_n, sizeof(seq_n), 0, (struct sockaddr*) &destAddr, sizeof(destAddr));
    if (err < 0) {
            goto ERROR;
    } else {
            err = recvfrom(connect_socket, &seq_n, sizeof(seq_n), 0, NULL, 0);
            if (err < 0) {
                    goto ERROR;
            } else {
                    if (ntohs(seq_n) == seq) {
                    	osSemaphoreRelease(connectAckSem);
                    }
            }
    }

ERROR:
	close(connect_socket);
	osThreadTerminate(osThreadGetId());
}

static void Network_Thread(void const *argument)
{
	osEvent evt;
	static uint16_t seq;

	BSP_LED_Init(LED5);
	BSP_LED_Off(LED5);

	osTimerDef(NET_TIMER, Heart_Beat_Signal);
	osTimerId netTimer;
	netTimer = osTimerCreate (osTimer(NET_TIMER), osTimerPeriodic, NULL);

	osThreadId connectThread;
	osThreadDef(CONNECT, Connect_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	osSemaphoreDef(CONNECT_ACK_SEM);
	connectAckSem = osSemaphoreCreate(osSemaphore(CONNECT_ACK_SEM) , 1);
	osSemaphoreWait(connectAckSem , osWaitForever);	//Decrement the semaphore

	while (1) {
		evt = osSignalWait(HEART_BEAT_SIGNAL | CONNECT_ON_SIGNAL | CONNECT_OFF_SIGNAL, osWaitForever);
		if (evt.status == osEventSignal) {
			if (evt.value.signals & HEART_BEAT_SIGNAL) {
				BSP_LED_Toggle(LED5);
				connectThread = osThreadCreate(osThread(CONNECT), &seq);
				if(osSemaphoreWait(connectAckSem , 500) == 0) {
					seq++;
				} else {
					close(connect_socket);
					osThreadTerminate(connectThread);
				}
			}

			if (evt.value.signals & CONNECT_ON_SIGNAL) {
				seq = 0;
				osTimerStart (netTimer, 1000);
			}

			if (evt.value.signals & CONNECT_OFF_SIGNAL) {
				osTimerStop (netTimer);
			}
		}
	}
}

/*
 * Battery Monitor Thread
 */
typedef enum {
	BATTERY_VOLTAGE_LOW = 0,
	BATTERY_VOLTAGE_OK = 1,
} battery_voltage_t;

battery_voltage_t bat_vol_state;
osThreadId batteryMonitorThread;

#define BAT_VOL_CHECK_SIGNAL 	(0x1)
#define BAT_VOL_LOW_IND_SIGNAL	(0x1 << 1)

void Bat_Voltage_Check_Signal(void const *arg)
{
	osSignalSet(batteryMonitorThread, BAT_VOL_CHECK_SIGNAL);
}

void Bat_Voltage_Low_Indicator_Signal(void const *arg)
{
	osSignalSet(batteryMonitorThread, BAT_VOL_LOW_IND_SIGNAL);
}

static void Battery_Monitor_Thread(void const *argument)
{
	osEvent evt;
	uint32_t mV;

	osTimerDef(BAT_TIMER, Bat_Voltage_Check_Signal);
	osTimerId batTimer;
	batTimer = osTimerCreate (osTimer(BAT_TIMER), osTimerPeriodic, NULL);
	osTimerStart (batTimer, 6000);

	osTimerDef(BAT_LOW_INDICATOR, Bat_Voltage_Low_Indicator_Signal);
	osTimerId batLowIndTimer;
	batLowIndTimer = osTimerCreate (osTimer(BAT_LOW_INDICATOR), osTimerPeriodic, NULL);

	bat_vol_state = BATTERY_VOLTAGE_OK;

	while (1) {
		static uint32_t bat_low_ind_count = 0;
		evt = osSignalWait(BAT_VOL_LOW_IND_SIGNAL | BAT_VOL_CHECK_SIGNAL , osWaitForever);
		if (evt.status == osEventSignal) {
			if (evt.value.signals & BAT_VOL_LOW_IND_SIGNAL) {
				bat_low_ind_count++;
				if (bat_low_ind_count % 4 == 0) {
					BSP_LED_On(LED3);
				} else if (bat_low_ind_count % 4 == 1) {
					BSP_LED_Off(LED3);
				}
			}

			if (evt.value.signals & BAT_VOL_CHECK_SIGNAL) {
				ADC_ConfigChannel();
				ADC_Start();
				ADC_PollForConversion();
				mV = ADC_GetValue();
				if (mV < 3500) {
					if (bat_vol_state == BATTERY_VOLTAGE_OK) {
						osTimerStart(batLowIndTimer, 125);
						bat_vol_state = BATTERY_VOLTAGE_LOW;
					}
				} else {
					if (bat_vol_state == BATTERY_VOLTAGE_LOW) {
						osTimerStop(batLowIndTimer);
						BSP_LED_Off(LED3);
						bat_vol_state = BATTERY_VOLTAGE_OK;
					}
				}
			}
		}
	}
}

/*
 * User Input Thread
 */

void Button_Init(Button_TypeDef Button)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	BUTTONx_GPIO_CLK_ENABLE(Button);

    GPIO_InitStruct.Pin = KEY_BUTTON_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    HAL_GPIO_Init(KEY_BUTTON_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority((IRQn_Type)KEY_BUTTON_EXTI_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)KEY_BUTTON_EXTI_IRQn);
}

osThreadId userInputThread;
osSemaphoreId buttonISRSem;

static void User_Input_Thread(void const *argument)
{
	BSP_LED_Init(LED4);
	BSP_LED_Off(LED4);
	while (true) {
		if (buttonISRSem != NULL) {
			if(osSemaphoreWait(buttonISRSem , osWaitForever) == 0) {
				uint32_t count = 5;
				GPIO_PinState pinState;

				pinState = HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN);
				do {
					if (count-- <= 0) {
						if (pinState == GPIO_PIN_SET && bat_vol_state == BATTERY_VOLTAGE_OK) {
							BSP_LED_On(LED4);
							osSignalSet(networkThread, CONNECT_ON_SIGNAL);
						} else {
							BSP_LED_Off(LED4);
							osSignalSet(networkThread, CONNECT_OFF_SIGNAL);
						}
						break;
					}
					osDelay(10);
				} while (pinState == HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN));
	  		}
	    }
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == KEY_BUTTON_PIN) {
		osSemaphoreRelease(buttonISRSem);
	}
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	BSP_LED_Init(LED3);
	BSP_LED_Off(LED3);

	Button_Init(BUTTON_KEY);

	osSemaphoreDef(BSEM);
	buttonISRSem = osSemaphoreCreate(osSemaphore(BSEM) , 1);

	osThreadDef(BAT, Battery_Monitor_Thread, osPriorityHigh, 0, configMINIMAL_STACK_SIZE);
	batteryMonitorThread = osThreadCreate(osThread(BAT), NULL);
	osThreadDef(UI, User_Input_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	userInputThread = osThreadCreate(osThread(UI), NULL);
	osThreadDef(NET, Network_Thread, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
	networkThread = osThreadCreate(osThread(NET), NULL);

	osKernelStart();

	while(1) {
	}
}
