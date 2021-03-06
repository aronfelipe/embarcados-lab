/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// Configuracoes do LED
#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        12                   // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// Configuracoes do botao
#define BUT_PIO           PIOA                 // periferico que controla o LED
#define BUT_PIO_ID        10                   // ID do periférico PIOC (controla LED)
#define BUT_PIO_IDX       11                   // ID do LED no PIO
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)   // Mascara para CONTROLARMOS o LED

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
  // Função de inicialização do uC
 void init(void){
	  // Initialize the board clock
	  sysclk_init();

	  // Desativa WatchDog Timer
	  WDT->WDT_MR = WDT_MR_WDDIS;
	  
	  // Ativa o PIO na qual o LED foi conectado
	  // para que possamos controlar o LED.
	  pmc_enable_periph_clk(LED_PIO_ID);
	  
	  //Inicializa PC8 como saída
	  pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	  
	  // Inicializa PIO do botao
	  pmc_enable_periph_clk(BUT_PIO_ID);
	  
	  // configura pino ligado ao botão como entrada com um pull-up.
	  pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
	  
	  // configura pino para valor padrão seja o energizado.
	  pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
  }
  
  /**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,const uint32_t ul_pull_up_enable){
	
	
 }

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{

	init();
  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  int but;
  while (1)
  {	
	but = pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK);
	if(!but){
		for(int i = 0; i < 5; i++){
			_pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do 
			delay_ms(100);                        // Delay por software de 200 ms
			_pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
			delay_ms(100);
		}
	}
  }
  return 0;
}
