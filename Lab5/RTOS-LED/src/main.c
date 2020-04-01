/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

// LED1 Task
#define TASK_LED1_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)

// LED
#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1 << LED1_PIO_IDX)

// Button
#define BUT_PIO_1      PIOD
#define BUT_PIO_ID_1   ID_PIOD
#define BUT_IDX_1  28
#define BUT_IDX_MASK_1 (1 << BUT_IDX_1)

// Button
#define BUT_PIO_2      PIOC
#define BUT_PIO_ID_2   ID_PIOC
#define BUT_IDX_2  31
#define BUT_IDX_MASK_2 (1 << BUT_IDX_2)

// Button
#define BUT_PIO_3      PIOA
#define BUT_PIO_ID_3   ID_PIOA
#define BUT_IDX_3  19
#define BUT_IDX_MASK_3 (1 << BUT_IDX_3)

/** Semaforo a ser usado pela task led 
    tem que ser var global! */

SemaphoreHandle_t xSemaphore_1;

SemaphoreHandle_t xSemaphore_2;

SemaphoreHandle_t xSemaphore_3;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

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
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
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


/**
* callback do botao
* libera semaforo: xSemaphore
*/
void but1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore_1, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

void but2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore_2, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

void but3_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore_3, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}


/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(3000);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led(void *pvParameters) {
	/* We are using the semaphore for synchronisation so we create a binary
	semaphore rather than a mutex.  We must make sure that the interrupt
	does not attempt to use the semaphore before it is created! */
	xSemaphore_1 = xSemaphoreCreateBinary();
	xSemaphore_2 = xSemaphoreCreateBinary();
	xSemaphore_3 = xSemaphoreCreateBinary();

	/* devemos iniciar a interrupcao no pino somente apos termos alocado
	os recursos (no caso semaforo), nessa funcao inicializamos 
	o botao e seu callback*/
	/* init botão */
	pmc_enable_periph_clk(BUT_PIO_ID_1);
	pio_configure(BUT_PIO_1, PIO_INPUT, BUT_IDX_MASK_1, PIO_PULLUP);
	pio_handler_set(BUT_PIO_1, BUT_PIO_ID_1, BUT_IDX_MASK_1, PIO_IT_FALL_EDGE, but1_callback);
	pio_enable_interrupt(BUT_PIO_1, BUT_IDX_MASK_1);
	NVIC_EnableIRQ(BUT_PIO_ID_1);
	NVIC_SetPriority(BUT_PIO_ID_1, 4); // Prioridade 4
	  
	pmc_enable_periph_clk(BUT_PIO_ID_2);
	pio_configure(BUT_PIO_2, PIO_INPUT, BUT_IDX_MASK_2, PIO_PULLUP);
	pio_handler_set(BUT_PIO_2, BUT_PIO_ID_2, BUT_IDX_MASK_2, PIO_IT_FALL_EDGE, but2_callback);
	pio_enable_interrupt(BUT_PIO_2, BUT_IDX_MASK_2);
	NVIC_EnableIRQ(BUT_PIO_ID_2);
	NVIC_SetPriority(BUT_PIO_ID_2, 4); // Prioridade 4
	
	pmc_enable_periph_clk(BUT_PIO_ID_3);
	pio_configure(BUT_PIO_3, PIO_INPUT, BUT_IDX_MASK_3, PIO_PULLUP);
	pio_handler_set(BUT_PIO_3, BUT_PIO_ID_3, BUT_IDX_MASK_3, PIO_IT_FALL_EDGE, but3_callback);
	pio_enable_interrupt(BUT_PIO_3, BUT_IDX_MASK_3);
	NVIC_EnableIRQ(BUT_PIO_ID_3);
	NVIC_SetPriority(BUT_PIO_ID_3, 4); // Prioridade 4

	  if (xSemaphore_1 == NULL)
	  printf("falha em criar o semaforo \n");
	  
	  if (xSemaphore_2 == NULL)
	  printf("falha em criar o semaforo \n");
	  
	  if (xSemaphore_3 == NULL)
	  printf("falha em criar o semaforo \n");
	  
	  for (;;) {
		  if( xSemaphoreTake(xSemaphore_1, ( TickType_t ) 500) == pdTRUE ){
			  LED_Toggle(LED0);
		  }
		  if( xSemaphoreTake(xSemaphore_2, ( TickType_t ) 500) == pdTRUE ){
			pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		  } else {
			  pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		  }
	  }
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led1(void *pvParameters){  
	pmc_enable_periph_clk(LED1_PIO_ID);
    pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);

    /* Block for 3000ms. */
    const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;
	
	/* Block for 200ms. */
	const TickType_t xDelay_Led = 200 / portTICK_PERIOD_MS;

    for (;;) {
		for (int i=0; i<3; i++){
			pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
			vTaskDelay(xDelay_Led);
			pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			vTaskDelay(xDelay_Led);
		};
      vTaskDelay(xDelay);
    }
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

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
	
	/* Create task to make led1 blink */
	if (xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL,
	 TASK_LED1_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led1 task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}