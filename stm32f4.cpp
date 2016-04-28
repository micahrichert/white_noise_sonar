#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <algorithm>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>

#define NR_OUTPUTS 2 // can't be more than 8
#define NR_INPUTS 1
#define MAX_DISTANCE 1.0//5.0 // m
#define XCOR_DECAY_TIME 1.000 // s
#define MAX_SAMPLE_RATE 76000.0/11.1 // Hz
#define SPEED_OF_SOUND 340.3 // m/s
#define SAMPLE_RATE (MAX_SAMPLE_RATE/NR_INPUTS)  // if there are more than one a2d that can run in parallel then this equation needs to be updated
#define DISTANCE_PER_SAMPLE (SPEED_OF_SOUND/SAMPLE_RATE)
#define NR_SAMPLES (int(MAX_DISTANCE/DISTANCE_PER_SAMPLE))
#define XCOR_DECAY_SAMPLES (int(SAMPLE_RATE * XCOR_DECAY_TIME))
#define XCOR_DECAY_RATE (1.0-1.0/XCOR_DECAY_SAMPLES)

uint8_t rand_hist[NR_SAMPLES+1][NR_OUTPUTS]; // need one extra slot due to the analog value being delayed by 1 cycle
int16_t rand_hist_pos;
int32_t xcors[NR_INPUTS][NR_SAMPLES][NR_OUTPUTS];
uint32_t xs[NR_OUTPUTS]= {230489920, 43587912};//, 23498165, 56816325, 369823, 9274971, 2846911, 6376929};  // 8 random seeds
int16_t prev_input[NR_INPUTS];
int16_t t = 0;
uint32_t start_time = 0;

uint32_t fast_rand(uint32_t x) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

/// maximum length of single message sent via SERIAL_send_string or SERIAL_printf
#define SERIAL_MESSAGE_SIZE 64
/// maximum amount of characters that can be buffered between SERIAL_printf_poll() calls
#define SERIAL_MESSAGE_BUFFER_SIZE 512

static char            tx_buffer[2][SERIAL_MESSAGE_BUFFER_SIZE];
static uint32_t        num_chars_to_send = 0;
static uint32_t        available_buffer  = 0;
static volatile bool   transfer_complete = true;
static char            printf_buffer[SERIAL_MESSAGE_SIZE];

void dma1_stream3_isr()
{
    // check if transfer is complete
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM3, DMA_LISR_TCIF0))
    {
        // turn off transfer
        usart_disable_tx_dma(USART3);
        dma_disable_stream(DMA1, DMA_STREAM3);

        // clear interrupt flag, so interrupt won't fire again
        dma_clear_interrupt_flags(DMA1, DMA_STREAM3, DMA_LISR_TCIF0);
        transfer_complete = true;
    }
}

// Initialize COM1 port as serial output device
void serial_init(uint32_t baud)
{
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_DMA1);

    // Setup GPIO pins for USART3 transmit.
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
    // Setup USART3 TX pin as alternate function.
    gpio_set_af(GPIOD, GPIO_AF7, GPIO8);

    // configure USART3
    usart_set_baudrate(USART3, baud);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_enable(USART3);

    // configure DMA
    dma_stream_reset(DMA1, DMA_STREAM3);
    dma_set_transfer_mode(DMA1,
                          DMA_STREAM3,
                          DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA1, DMA_STREAM3, (uint32_t)&USART3_DR);
    // use channel, connected to USART3
    dma_channel_select(DMA1,
                       DMA_STREAM3,
                       DMA_SxCR_CHSEL_4);
    // increment a memory pointer after each transfer
    dma_enable_memory_increment_mode(DMA1,
                                     DMA_STREAM3);
    // do not increment peripheral pointer, all ADC regular channel data is stored in single register
    dma_disable_peripheral_increment_mode(DMA1,
                                          DMA_STREAM3);
    dma_set_memory_size(DMA1, DMA_STREAM3, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM3, DMA_SxCR_PSIZE_8BIT);
    dma_set_priority(DMA1,
                     DMA_STREAM3,
                     DMA_SxCR_PL_HIGH);
    // fire dma1_stream3_isr when transfer is complete
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM3);

    // setup some priority for DMA interrupt, not very important interrupt
    nvic_set_priority(NVIC_DMA1_STREAM3_IRQ, 5);
    nvic_enable_irq(NVIC_DMA1_STREAM3_IRQ);

    // reset transfer flag, so we won't lock in SERIAL_printf_poll
    available_buffer  = 0;
    num_chars_to_send = 0;
    transfer_complete = true;
}


void serial_printf_poll()
{
    // lock until previous transfer is complete
    if (num_chars_to_send > 0)
    {
        while (!transfer_complete);

        // configure dma transfer
        dma_set_memory_address(DMA1, DMA_STREAM3, (uint32_t)tx_buffer[available_buffer]);
        dma_set_number_of_data(DMA1, DMA_STREAM3, num_chars_to_send);

        // from now on write to another tx_buffer
        available_buffer  = 1 - available_buffer;
        num_chars_to_send = 0;

        // start transfer
        transfer_complete = false;
        dma_enable_stream(DMA1, DMA_STREAM3);
        usart_enable_tx_dma(USART3);
    }
}

void serial_printf_helper(const char* pFormat, ...)
{
    va_list ap;
    uint32_t     num_written;

    va_start(ap, pFormat);
    num_written = vsnprintf(printf_buffer, SERIAL_MESSAGE_SIZE, pFormat, ap);
    va_end(ap);
    if (num_written > 0)
    {
        // copy data to available tx_buffer
        uint32_t num_chars_available = SERIAL_MESSAGE_BUFFER_SIZE - num_chars_to_send;
        uint32_t num_chars_to_copy   = std::min(num_written, num_chars_available);
        memcpy(tx_buffer[available_buffer] + num_chars_to_send, printf_buffer, num_chars_to_copy);
        num_chars_to_send += num_chars_to_copy;
    }
}

template <typename ... Ts>
inline void serial_printf(const char* pFormat, Ts ... ts){
    serial_printf_helper(pFormat, ts...);
}

// =====================
//         Clock
// =====================

constexpr uint32_t ahb_clock         = 168000000;
constexpr uint32_t systick_clock     = ahb_clock / 8;
/// how long interval we want for between sys_tick_handler interrupts
constexpr uint32_t systick_period_ms = 1;
constexpr uint32_t systick_period    = systick_clock / 1000 * systick_period_ms;
constexpr uint32_t systick_reload    = systick_period - 1;

void clock_setup()
{
    // NOTE: if you change clock settings, also change the constants above
    // 168MHz processor/AHB clock
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    // 168MHz / 8 => 21 million counts per second
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    // set reload to every 1 millisecond
    systick_set_reload(systick_reload);
    systick_interrupt_enable();
    systick_counter_enable();
}

/// this variable is used to create a time reference incremented by systick_period_ms in sys_tick_handler
volatile uint32_t time_ms = 0;

// systick interrupt, fired on overload
extern "C" void sys_tick_handler(void)
{
    time_ms += 1;
}

uint32_t get_ms_from_start()
{
    return time_ms;
}


void setup()
{
    clock_setup();

    serial_init(115200L);
    serial_printf("Setting up\n");
    serial_printf("%d %d\n", NR_SAMPLES, XCOR_DECAY_SAMPLES);

    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
    {
        gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dpin); // Use PB0...
    }
    
    rcc_periph_clock_enable(RCC_ADC1);
    adc_off(ADC1);

    for (uint8_t apin=0; apin<NR_INPUTS; apin++)
    {
        gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, apin); // Use PA0...
    }

	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_15CYC); // need to play with this value and the prescaler
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2); // 1Mhz sampling rate?
    adc_power_on(ADC1);
}

int main(void)
{
    uint8_t channel_array[16];

    setup();
    
    while (1)
    {
        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            xs[dpin] = fast_rand(xs[dpin]);
            rand_hist[rand_hist_pos][dpin] = xs[dpin] & 0x1;

            if (xs[dpin]&0x1) gpio_set(GPIOB, dpin);
            else gpio_clear(GPIOB, dpin);
        }

        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
        {
            int16_t input = prev_input[apin];
            
            // select a2d input pin and start the conversion
	        channel_array[0] = apin;
	        adc_set_regular_sequence(ADC1, 1, channel_array);
        	adc_start_conversion_regular(ADC1);
	
            for (int16_t i=NR_SAMPLES-1, pos=rand_hist_pos-1; i>=0; i--,pos--)
            {
                if (pos < 0) pos = NR_SAMPLES;
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                {
                    int32_t tmp_xcor = xcors[apin][i][dpin];
                    if (rand_hist[pos][dpin]) tmp_xcor += input;
                    else tmp_xcor -= input;
                    xcors[apin][i][dpin] = tmp_xcor;
                }
            }
            
            // wait for a2d to finish and read sample
	        if (!adc_eoc(ADC1))
	            serial_printf("ADC conversion took too long!\n");
            prev_input[apin] = adc_read_regular(ADC1);
        }

        rand_hist_pos += 1;
        if (rand_hist_pos >= NR_SAMPLES+1) rand_hist_pos = 0;

        t += 1;
        if (t >= XCOR_DECAY_SAMPLES)
        {
            t = 0;
            serial_printf_poll();
            serial_printf("%d\n", get_ms_from_start() - start_time);
            start_time = get_ms_from_start();
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                {
                    serial_printf("%d %d: ", apin, dpin);
                    for (int16_t i=NR_SAMPLES-1;i>=0;i--)
                    {
                        serial_printf("%d ", xcors[apin][i][dpin]>>16);
                        xcors[apin][i][dpin] = 0; // set to 0 or decay
                    }
                    serial_printf("\n");
                }
            }
        }
    }
}

