/*
* Example RTOS Atmel Studio
*/


/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

// Tasks
#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_LED1_STACK_SIZE 			   (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY		   (tskIDLE_PRIORITY)

#define TASK_LED2_STACK_SIZE 			   (1024/sizeof(portSTACK_TYPE))
#define TASK_LED2_STACK_PRIORITY		   (tskIDLE_PRIORITY)

#define TASK_LED3_STACK_SIZE 			   (1024/sizeof(portSTACK_TYPE))
#define TASK_LED3_STACK_PRIORITY		   (tskIDLE_PRIORITY)

#define TASK_EXECUTE_STACK_SIZE 	       (1024/sizeof(portSTACK_TYPE))
#define TASK_EXECUTE_STACK_PRIORITY		   (tskIDLE_PRIORITY)

/**
* LEDs OLED
*/

// LED PLACA
#define LED_PIO_PLACA      PIOC
#define LED_PIO_ID_PLACA   ID_PIOC
#define LED_IDX_PLACA      8
#define LED_IDX_MASK_PLACA (1 << LED_IDX_PLACA)

// LED1 OLED
#define LED_PIO_1 PIOA
#define LED_PIO_ID_1 ID_PIOA
#define LED_IDX_1 0
#define LED_IDX_MASK_1 (1 << LED_IDX_1)

// LED2 OLED
#define LED_PIO_2 PIOC
#define LED_PIO_ID_2 ID_PIOC
#define LED_IDX_2 30
#define LED_IDX_MASK_2 (1 << LED_IDX_2)

// LED3 OLED
#define LED_PIO_3 PIOB
#define LED_PIO_ID_3 ID_PIOB
#define LED_IDX_3 2
#define LED_IDX_MASK_3 (1 << LED_IDX_3)

// Button 1 OLED
#define BUT_PIO_1      PIOD
#define BUT_PIO_ID_1   ID_PIOD
#define BUT_IDX_1  28
#define BUT_IDX_MASK_1 (1 << BUT_IDX_1)

// Button 2 OLED
#define BUT_PIO_2      PIOC
#define BUT_PIO_ID_2   ID_PIOC
#define BUT_IDX_2  31
#define BUT_IDX_MASK_2 (1 << BUT_IDX_2)

// Button 3 OLED
#define BUT_PIO_3      PIOA
#define BUT_PIO_ID_3   ID_PIOA
#define BUT_IDX_3  19
#define BUT_IDX_MASK_3 (1 << BUT_IDX_3)

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
// Flag button 1 
volatile char flag_led_1 = 0;
// Flag button 2
volatile char flag_led_2 = 0;
// Flag button 3
volatile char flag_led_3 = 0;

// Semaphores
SemaphoreHandle_t semaphore_button_1;
SemaphoreHandle_t semaphore_button_2;
SemaphoreHandle_t semaphore_button_3;

// Queues
QueueHandle_t queue_uart;
QueueHandle_t queue_command;

/************************************************************************/
/* Prototyopes                                                          */
/************************************************************************/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* Functions                                                            */
/************************************************************************/

/* INIT                                                                 */
/************************************************************************/

/* CALLBACKS                                                            */
/************************************************************************/
/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
void but1_callback(void){
	// Sinaliza que o butão foi pressionado
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// Manda uma alteração no semáforo
	xSemaphoreGiveFromISR(semaphore_button_1, &xHigherPriorityTaskWoken);
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
void but2_callback(void){
	// Sinaliza que o butão foi pressionado
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// Manda uma alteração no semáforo
	xSemaphoreGiveFromISR(semaphore_button_2, &xHigherPriorityTaskWoken);
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
void but3_callback(void){
	// Sinaliza que o butão foi pressionado
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// Manda uma alteração no semáforo
	xSemaphoreGiveFromISR(semaphore_button_3, &xHigherPriorityTaskWoken);
}

void init(void){
	
	sysclk_init();
	
	// Disable watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Configure LED1
	pmc_enable_periph_clk(LED_PIO_ID_1);
	pio_configure(LED_PIO_1, PIO_OUTPUT_1, LED_IDX_MASK_1, PIO_DEFAULT);
	
	// Configure LED2
	pmc_enable_periph_clk(LED_PIO_ID_2);
	pio_configure(LED_PIO_2, PIO_OUTPUT_1, LED_IDX_MASK_2, PIO_DEFAULT);
		
	// Configure LED3
	pmc_enable_periph_clk(LED_PIO_ID_3);
	pio_configure(LED_PIO_3, PIO_OUTPUT_1, LED_IDX_MASK_3, PIO_DEFAULT);
	
	// Initialize clock of PIO responsible for buttons 1,2 and 3
	pmc_enable_periph_clk(BUT_PIO_ID_1);
	pmc_enable_periph_clk(BUT_PIO_ID_2);
	pmc_enable_periph_clk(BUT_PIO_ID_3);
	
	// Configure PIO for buttons 1,2 and 3
	pio_configure(BUT_PIO_1, PIO_INPUT, BUT_IDX_MASK_1, PIO_PULLUP);
	pio_configure(BUT_PIO_2, PIO_INPUT, BUT_IDX_MASK_2, PIO_PULLUP);
	pio_configure(BUT_PIO_3, PIO_INPUT, BUT_IDX_MASK_3, PIO_PULLUP);
	
	// PIO 1 handler
	pio_handler_set(BUT_PIO_1,
	BUT_PIO_ID_1,
	BUT_IDX_MASK_1,
	PIO_IT_FALL_EDGE,
	but1_callback);
	
	// Set PIO 2 handler
	pio_handler_set(BUT_PIO_2,
	BUT_PIO_ID_2,
	BUT_IDX_MASK_2,
	PIO_IT_FALL_EDGE,
	but2_callback);
	
	// Set PIO 3 handler
	pio_handler_set(BUT_PIO_3,
	BUT_PIO_ID_3,
	BUT_IDX_MASK_3,
	PIO_IT_FALL_EDGE,
	but3_callback);
	
	// Enable interruption
	pio_enable_interrupt(BUT_PIO_1, BUT_IDX_MASK_1);
	pio_enable_interrupt(BUT_PIO_2, BUT_IDX_MASK_2);
	pio_enable_interrupt(BUT_PIO_3, BUT_IDX_MASK_3);

	// Configure interruption button PIO 1
	NVIC_EnableIRQ(BUT_PIO_ID_1);
	NVIC_SetPriority(BUT_PIO_ID_1, 4); // Prioridade 4
	
	// Configure interruption button PIO 2
	NVIC_EnableIRQ(BUT_PIO_ID_2);
	NVIC_SetPriority(BUT_PIO_ID_2, 4); // Prioridade 4
	
	// Configure interruption button PIO 3
	NVIC_EnableIRQ(BUT_PIO_ID_3);
	NVIC_SetPriority(BUT_PIO_ID_3, 4); // Prioridade 4
}

/* PIN TOGGLE                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask) {
	if (pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio, mask);
}

/* UART                                                                 */
/************************************************************************/
void USART1_Handler(void){
	uint32_t ret = usart_get_status(USART1);

	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	char c;

	// Verifica por qual motivo entrou na interrupçcao?
	// RXRDY ou TXRDY

	//  Dados disponível para leitura
	if(ret & US_IER_RXRDY){
		usart_serial_getchar(USART1, &c);
		xQueueSendFromISR(queue_uart, &c, 0);

		// -  Transmissoa finalizada
		} else if(ret & US_IER_TXRDY){

	}
}

uint32_t usart1_puts(uint8_t *pstring){
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART1))
	usart_serial_putchar(USART1, *(pstring+i++));
}

static void USART1_init(void){
	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4); // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

	/* Configura opcoes USART */
	const sam_usart_opt_t usart_settings = {
		.baudrate       = 115200,
		.char_length    = US_MR_CHRL_8_BIT,
		.parity_type    = US_MR_PAR_NO,
		.stop_bits    = US_MR_NBSTOP_1_BIT    ,
		.channel_mode   = US_MR_CHMODE_NORMAL
	};

	/* Ativa Clock periferico USART0 */
	sysclk_enable_peripheral_clock(ID_USART1);

	stdio_serial_init(CONF_UART, &usart_settings);

	/* Enable the receiver and transmitter. */
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);

	/* map printf to usart */
	ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

	/* ativando interrupcao */
	usart_enable_interrupt(USART1, US_IER_RXRDY);
	NVIC_SetPriority(ID_USART1, 4);
	NVIC_EnableIRQ(ID_USART1);
}


/* RTOS                                                                 */
/************************************************************************/
/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/* TASKS                                                                */
/************************************************************************/
/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	while(1) {
		printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(3000);
	}
}

static void task_uart(void *pvParameters){
	char buffer_uart[32];
	char msg_from_uart;
	int msg_counter = 0 ;
	while(1){		
		 if (xQueueReceive( queue_uart, &(msg_from_uart), ( TickType_t )  100 / portTICK_PERIOD_MS)) {
			if(msg_from_uart != '\n'){
				buffer_uart[msg_counter] = msg_from_uart;
				msg_counter++;
			} else {
				buffer_uart[msg_counter] = 0;
				msg_counter = 0 ;
				xQueueSend(queue_command, &buffer_uart, 0);
			}
		}
	}
}

static void task_execute(void *pvParameters){
	char s[64];
	while(1){		
		if (xQueueReceive( queue_command, &(s), ( TickType_t )  100 / portTICK_PERIOD_MS)) {
			if(!strcmp(s, "led1on")){
                pin_toggle(LED_PIO_1, LED_IDX_MASK_1);
// 			else if (!strcmp(s, "led1on"))
// 			} else if (s == "led 3 toggle") {
// 				if (flag_led_3 == 1) {
// 	      			pio_clear(LED3_PIO, LED3_IDX_MASK);
// 					flag_led_3 = 0;
// 				} else {
// 					pio_set(LED3_PIO, LED3_IDX_MASK);
// 	      			flag_led_3 = 1;
// 				}
			}
		}
	}
}

static void task_led(void *pvParameters)
{
	/* Block for 2000ms. */
	const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;

	while (1) {
		LED_Toggle(LED0);
		vTaskDelay(xDelay);
	}
}

static void task_led1(void *pvParameters)
{
	semaphore_button_1 = xSemaphoreCreateBinary();

	if (semaphore_button_1 == NULL)
	printf("falha em criar o semaforo \n");

	while (1) {
		if( xSemaphoreTake(semaphore_button_1, ( TickType_t ) 500) == pdTRUE ){
            pin_toggle(LED_PIO_1, LED_IDX_MASK_1);
		}
	}
}

static void task_led2(void *pvParameters)
{
	semaphore_button_2 = xSemaphoreCreateBinary();

	if (semaphore_button_2 == NULL)
	printf("falha em criar o semaforo \n");

	while (1) {
		if( xSemaphoreTake(semaphore_button_2, ( TickType_t ) 500) == pdTRUE ){
            pin_toggle(LED_PIO_2, LED_IDX_MASK_2);
		}
	}
}

static void task_led3(void *pvParameters)
{
	/* Block for 2000ms. */
	const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
	semaphore_button_3 = xSemaphoreCreateBinary();

	if (semaphore_button_3 == NULL)
	printf("falha em criar o semaforo \n");

	while (1) {
		if( xSemaphoreTake(semaphore_button_3, ( TickType_t ) 500) == pdTRUE ){
            pin_toggle(LED_PIO_3, LED_IDX_MASK_3);
		}
	}
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	init();
	board_init();

	/* Initialize the console uart */
	queue_uart  = xQueueCreate( 32, sizeof( char ) );
    queue_command = xQueueCreate(5, sizeof(char[64]));

	USART1_init();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_led1, "Led1", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led2, "Led2", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_led3, "Led3", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Create task to make uart communication */
	if (xTaskCreate(task_uart, "uart", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make execute uart command */
	if (xTaskCreate(task_execute, "execute", TASK_EXECUTE_STACK_SIZE, NULL,
	TASK_EXECUTE_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
