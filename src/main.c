#include <inttypes.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "pulses.h"

#define BUZZER_GPIO_RCC RCC_GPIOA  // BUZZER PA8
#define BUZZER_GPIO_PORT GPIOA
#define BUZZER_GPIO_PIN GPIO8
#define BUZZER_GPIO_AF GPIO_AF2  // TIM1_CH1

#define PWM_TIMER         TIM1
#define PWM_TIMER_RCC     RCC_TIM1
#define PWM_TIMER_CHANNEL TIM_OC1
#define PWM_DMA_REQUEST   TIM_DIER_CC1DE

#define SAM_TIMER         TIM2
#define SAM_TIMER_RCC     RCC_TIM2

#define PWM_DMA             DMA1
#define PWM_DMA_RCC         RCC_DMA1
#define PWM_DMA_CHANNEL     DMA_CHANNEL2
#define PWM_DMA_PERIPH      TIM_CCR1(PWM_TIMER)
#define PWM_DMA_PERIPH_SIZE DMA_CCR_PSIZE_16BIT

#define LED_GPIO_PORT GPIOF
#define LED_GPIO_PIN  GPIO3
#define LED_GPIO_RCC  RCC_GPIOF

/*
 * This example outputs a dynamic pwm signal on a gpio
 *
 * The pwm pulse width changes after every compare-match event
 * when the compare matches (pulse falls), the next pulse width is
 * loaded into the compare register (ccr) via dma
 *
 * dma reads the pulse widths sequentially from a buffer
 *
 * This method can be used to efficiently implement one wire signals
 * like ws2812b leds, dshot ESCs, and even pulse-count modulated
 * (PCM) wav audio
 */

void clock_setup() {
  //rcc_clock_setup_in_hsi_out_48mhz();
rcc_clock_setup_in_hse_8mhz_out_48mhz();
}

void gpio_setup() {
  // PWM GPIO
  rcc_periph_clock_enable(BUZZER_GPIO_RCC);
  gpio_mode_setup(BUZZER_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BUZZER_GPIO_PIN);
  gpio_set_af(BUZZER_GPIO_PORT, BUZZER_GPIO_AF, BUZZER_GPIO_PIN);
  gpio_set_output_options(BUZZER_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BUZZER_GPIO_PIN);

  //gpio_mode_setup(BUZZER_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BUZZER_GPIO_PIN);

  // LED GPIO
  rcc_periph_clock_enable(LED_GPIO_RCC);
  gpio_mode_setup(LED_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_GPIO_PIN);
}
/**
 * For PWM DMA PCM 
 **/
void setupPWMTimer() {
  // First timer outputs PWM to de buzzer
  rcc_periph_clock_enable(PWM_TIMER_RCC);  // enable timer clock
  rcc_periph_reset_pulse(PWM_TIMER);
  // prueba
  // timer_set_mode(PWM_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  // timer_continuous_mode(PWM_TIMER);

  // stretch clock with larger dividers in order to time longer signals without overruns
  timer_set_prescaler(PWM_TIMER, 0);
  timer_set_period(PWM_TIMER, 255);                               // set ARR
  timer_set_oc_value(PWM_TIMER, PWM_TIMER_CHANNEL, 128);
  timer_set_oc_mode(PWM_TIMER, PWM_TIMER_CHANNEL, TIM_OCM_PWM1);  // set OCM
  timer_enable_oc_preload(PWM_TIMER, PWM_TIMER_CHANNEL);          // set OCPE
  timer_enable_oc_output(PWM_TIMER, PWM_TIMER_CHANNEL);           // set CCxE
  // shouldn't do/be part of normal timer api (this only applies to advanced control timers)?
  timer_enable_break_main_output(PWM_TIMER);
  //timer_enable_irq(PWM_TIMER, PWM_DMA_REQUEST);  // set CCxDE (enable dma request on CCx event)
  timer_enable_counter(PWM_TIMER);  // set CEN
}

void setupSampleTimer() {
  // Second timer triggers DMA transfers (at sample rate)
  // enable the clock for Timer1 (f_PCLK2=72MHz)
  rcc_periph_clock_enable(SAM_TIMER_RCC);
  rcc_periph_reset_pulse(SAM_TIMER);
  // probamos esto
  //timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  // timer_continuous_mode(TIM3);
  //timer_set_oc_value(TIM3, TIM_OC3, 128);
  //timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);  // set OCM

  // use a prescaler of 1, i.e. f_TIM1=f_PCLK2/(0+1)=72MHz
  timer_set_prescaler(SAM_TIMER, 0);
  // set timer1 auto-reload value
  // so the timer overflow will occur with a frequency of 72MHz/1633=approx. 44.1kHz
  // 48Mhz/1088=44.1kHz
  timer_set_period(SAM_TIMER, 1088);
  // enable timer update DMA request
  // i.e. at every counter update/overflow, a DMA request will be issued
  // which triggers a DMA data transfer


	//timer_enable_preload(SAM_TIMER);
  //timer_continuous_mode(SAM_TIMER);


  //timer_set_dma_on_update_event(SAM_TIMER);
  //timer_enable_update_event(SAM_TIMER);
  timer_enable_irq(SAM_TIMER, TIM_DIER_UDE);

  // start timer
  timer_enable_counter(SAM_TIMER);
}
/**
 * For PWM DMA PCM https://marcelmg.github.io/pwm_dac_sound/
 **/
void setupDma() {
  rcc_periph_clock_enable(PWM_DMA_RCC);
  dma_channel_reset(PWM_DMA, PWM_DMA_CHANNEL);                                      // enable dma clock
  dma_set_peripheral_address(PWM_DMA, PWM_DMA_CHANNEL, (uint32_t)&PWM_DMA_PERIPH);  // set CPAR
  dma_set_memory_address(PWM_DMA, PWM_DMA_CHANNEL, (uint32_t)&pulses);              // set CMAR
  dma_set_read_from_memory(PWM_DMA, PWM_DMA_CHANNEL);                               // set DIR
  dma_set_memory_size(PWM_DMA, PWM_DMA_CHANNEL, DMA_CCR_MSIZE_8BIT);                // set MSIZE
  dma_set_peripheral_size(PWM_DMA, PWM_DMA_CHANNEL, PWM_DMA_PERIPH_SIZE);           // set PSIZE
  dma_enable_memory_increment_mode(PWM_DMA, PWM_DMA_CHANNEL);                       // set MINC
  dma_disable_peripheral_increment_mode(PWM_DMA, PWM_DMA_CHANNEL);
  dma_set_number_of_data(PWM_DMA, PWM_DMA_CHANNEL, sizeof(pulses));                 // set CNDTR
  dma_enable_circular_mode(PWM_DMA, PWM_DMA_CHANNEL);                               // set CIRC
  dma_set_priority(PWM_DMA, PWM_DMA_CHANNEL, DMA_CCR_PL_HIGH);
  dma_enable_channel(PWM_DMA, PWM_DMA_CHANNEL);                                     // set EN
}

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

/* Called when systick fires */
void sys_tick_handler(void) {
  system_millis++;
}

/* sleep for delay milliseconds */
static void msleep(uint32_t delay) {
  uint32_t wake = system_millis + delay;
  while (wake > system_millis)
    ;
}

/* Set up a timer to create 1mS ticks. */
static void systick_setup(void) {
  /* clock rate / 1000 to get 1mS interrupt rate */
  systick_set_reload(48000);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();
  /* this done last */
  systick_interrupt_enable();
}

void buzzer_init() {
  rcc_periph_clock_enable(PWM_TIMER_RCC);  // enable timer clock

  timer_set_mode(PWM_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_set_prescaler(PWM_TIMER, 48);
  timer_set_repetition_counter(PWM_TIMER, 0);
  timer_enable_preload(PWM_TIMER);
  timer_enable_break_main_output(PWM_TIMER);
  timer_continuous_mode(PWM_TIMER);

  timer_disable_oc_output(PWM_TIMER, PWM_TIMER_CHANNEL);
  timer_set_oc_mode(PWM_TIMER, PWM_TIMER_CHANNEL, TIM_OCM_PWM1);
  timer_set_oc_value(PWM_TIMER, PWM_TIMER_CHANNEL, 0);
  timer_enable_oc_output(PWM_TIMER, PWM_TIMER_CHANNEL);
}

void buzzer_enable() {
  timer_enable_counter(PWM_TIMER);
}

void buzzer_disable() {
  timer_disable_counter(PWM_TIMER);
}

void buzzer_set_freq(uint32_t freq) {
  uint32_t period = 1000000 / freq;

  timer_set_period(PWM_TIMER, period);
  timer_set_oc_value(PWM_TIMER, PWM_TIMER_CHANNEL, period / 2);
}
// Uncomment next line for PCM audio test. 
// Sadly volume is too low with factory buzzer.
// Comment for simple buzzer alarm test
#define PCM

int main() {
  clock_setup();
  systick_setup();
  gpio_setup();
#if defined(PCM)
  setupPWMTimer();
  setupDma();
  setupSampleTimer();
#else
  buzzer_init();
  buzzer_enable();
#endif

  gpio_set(LED_GPIO_PORT, LED_GPIO_PIN);

  /* Blink the LEDs (PD12, PD13, PD14 and PD15) on the board. */
  uint32_t freq = 200;

  while (1) {
#if !defined(PCM)
    buzzer_set_freq(freq);
    freq++;
    msleep(1);
#endif
  }

}