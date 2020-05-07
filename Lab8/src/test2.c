/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_LED1_STACK_SIZE 				(1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY			(tskIDLE_PRIORITY)

#define TASK_EXECUTE_STACK_SIZE 			(1024/sizeof(portSTACK_TYPE))
#define TASK_EXECUTE_STACK_PRIORITY			(tskIDLE_PRIORITY)

// Flag led 1
volatile char flag_led_1 = 0;

// Flag led 3
volatile char flag_led_3 = 0;

/**
* LEDs OLED
*/

// Led 1 OLED
#define LED1_PIO_ID	    ID_PIOA
#define LED1_PIO         PIOA
#define LED1_PIN		      0
#define LED1_IDX_MASK    (1<<LED1_PIN)

// Led 2 OLED
#define LED2_PIO_ID	    ID_PIOC
#define LED2_PIO         PIOC
#define LED2_PIN		      30
#define LED2_IDX_MASK    (1<<LED2_PIN)

// Led 3 OLED
#define LED3_PIO_ID	    ID_PIOB
#define LED3_PIO         PIOB
#define LED3_PIN		      2
#define LED3_IDX_MASK    (1<<LED3_PIN)

// Button 1 OLED
#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         16
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

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


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

// Semaphores
SemaphoreHandle_t semaphore_button_1;
SemaphoreHandle_t semaphore_button_2;
SemaphoreHandle_t semaphore_button_3;

// Queues
QueueHandle_t queue_uart;
QueueHandle_t queue_command;

/* PIN TOGGLE                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask) {
	if (pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio, mask);
}

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
            printf("Comando: %s\n", s);
			printf("CM: led 1 toggle\n");
			printf("%d", strcmp(s, "l"));
			if("led 1 toggle" == s){
				printf("OHTYEAYY");
				
			}
			if(!strcmp(s, "led1on")){
                pin_toggle(LED1_PIO, LED1_IDX_MASK);
			} else if (s == "led 3 toggle") {
				if (flag_led_3 == 1) {
	      			pio_clear(LED3_PIO, LED3_IDX_MASK);
					flag_led_3 = 0;
				} else {
					pio_set(LED3_PIO, LED3_IDX_MASK);
	      			flag_led_3 = 1;
				}
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

	while(1) {
		/* We are using the semaphore for synchronisation so we create a binary
		semaphore rather than a mutex.  We must make sure that the interrupt
		does not attempt to use the semaphore before it is created! */
		semaphore_button_1 = xSemaphoreCreateBinary();

		/* devemos iniciar a interrupcao no pino somente apos termos alocado
		os recursos (no caso semaforo), nessa funcao inicializamos 
		o botao e seu callback*/
		/* init botão */
		pmc_enable_periph_clk(BUT1_PIO_ID);
		pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
		pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
		pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
		NVIC_EnableIRQ(BUT1_PIO_ID);
		NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4

		if (semaphore_button_1 == NULL)
		printf("falha em criar o semaforo \n");

		while (1) {
			if( xSemaphoreTake(semaphore_button_1, ( TickType_t ) 500) == pdTRUE ){
            	pin_toggle(LED1_PIO, LED1_IDX_MASK);
			}
		}
	}
}

static void task_led2(void *pvParameters)
{

	while(1) {
		/* We are using the semaphore for synchronisation so we create a binary
		semaphore rather than a mutex.  We must make sure that the interrupt
		does not attempt to use the semaphore before it is created! */
		semaphore_button_1 = xSemaphoreCreateBinary();

		/* devemos iniciar a interrupcao no pino somente apos termos alocado
		os recursos (no caso semaforo), nessa funcao inicializamos 
		o botao e seu callback*/
		/* init botão */
		pmc_enable_periph_clk(BUT_PIO_2);
		pio_configure(BUT_PIO_2, PIO_INPUT, BUT_IDX_MASK_2, PIO_PULLUP);
		pio_handler_set(BUT_PIO_2, BUT_PIO_ID_2, BUT_IDX_MASK_2, PIO_IT_FALL_EDGE, but2_callback);
		pio_enable_interrupt(BUT_PIO_2, BUT_IDX_MASK_2);
		NVIC_EnableIRQ(BUT_PIO_ID_2);
		NVIC_SetPriority(BUT_PIO_ID_2, 4); // Prioridade 4

		if (semaphore_button_1 == NULL)
		printf("falha em criar o semaforo \n");

		while (1) {
			if( xSemaphoreTake(semaphore_button_1, ( TickType_t ) 500) == pdTRUE ){
            	pin_toggle(LED2_PIO, LED2_IDX_MASK);
			}
		}
	}
}

static void task_led3(void *pvParameters)
{

	while(1) {
		/* We are using the semaphore for synchronisation so we create a binary
		semaphore rather than a mutex.  We must make sure that the interrupt
		does not attempt to use the semaphore before it is created! */
		semaphore_button_1 = xSemaphoreCreateBinary();

		/* devemos iniciar a interrupcao no pino somente apos termos alocado
		os recursos (no caso semaforo), nessa funcao inicializamos 
		o botao e seu callback*/
		/* init botão */
		pmc_enable_periph_clk(BUT_PIO_3);
		pio_configure(BUT_PIO_3, PIO_INPUT, BUT_IDX_MASK_3, PIO_PULLUP);
		pio_handler_set(BUT_PIO_3, BUT_PIO_ID_3, BUT_IDX_MASK_3, PIO_IT_FALL_EDGE, but3_callback);
		pio_enable_interrupt(BUT_PIO_3, BUT_IDX_MASK_3);
		NVIC_EnableIRQ(BUT_PIO_ID_3);
		NVIC_SetPriority(BUT_PIO_ID_3, 4); // Prioridade 4

		if (semaphore_button_1 == NULL)
		printf("falha em criar o semaforo \n");

		while (1) {
			if( xSemaphoreTake(semaphore_button_1, ( TickType_t ) 500) == pdTRUE ){
            	pin_toggle(LED3_PIO, LED3_IDX_MASK);
			}
		}
	}
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

void init (void){	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_IDX_MASK, PIO_DEFAULT);
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
	queue_uart  = xQueueCreate( 32, sizeof( char ) );
    queue_command = xQueueCreate(5, sizeof(char[64]));

	USART1_init();

	init();

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
	if (xTaskCreate(task_led2, "Led2", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led3, "Led3", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led1, "Led1", TASK_LED_STACK_SIZE, NULL,
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
