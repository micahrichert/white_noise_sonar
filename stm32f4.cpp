#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <algorithm>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f4/rng.h>
#include <stdio.h>

#define NR_OUTPUTS 1 // can't be more than 8
#define NR_INPUTS 1
#define MAX_DISTANCE 1.5//5.0 // m
#define SAMPLE_STEP_SIZE 1 // integer
#define TIME_LAG 0.0000 // s, time to shift xcor analysis by to compensate for speaker lag
#define XCOR_DECAY_TIME 1.000 // s
#define SIGMA 3 // how many standard deviations greater than chance should be considered significant
//#define MAX_SAMPLE_RATE 35933 // Hz //2in, 2out at 0.5m
//#define MAX_SAMPLE_RATE 43206 // Hz //1in, 2out at 0.5m
//#define MAX_SAMPLE_RATE 47175 // Hz //3in, 1out at 0.5m
//#define MAX_SAMPLE_RATE 50666 // Hz //2in, 1out at 0.5m
#define MAX_SAMPLE_RATE 56000 // Hz //1in, 1out at 0.5m
#define SPEED_OF_SOUND (340.3/2.0) // m/s, divide by 2 because sound travels out and back
#define SAMPLE_RATE (MAX_SAMPLE_RATE)
#define DISTANCE_PER_SAMPLE (SPEED_OF_SOUND/SAMPLE_RATE)
#define NR_SAMPLES (int(MAX_DISTANCE/DISTANCE_PER_SAMPLE)/SAMPLE_STEP_SIZE)
#define XCOR_DECAY_SAMPLES (int(SAMPLE_RATE * XCOR_DECAY_TIME))
#define XCOR_DECAY_RATE (1.0-1.0/XCOR_DECAY_SAMPLES)
#define SIGMA2 (SIGMA*SIGMA) // the code works with variance instead of standard deviation
#define SAMPLE_LAG (int(TIME_LAG*SAMPLE_RATE)+1) // add 1 because inputs are always sampled time step behind
#define HISTORY_LEN (NR_INPUTS/32+1)

/// maximum length of single message sent via SERIAL_send_string or SERIAL_printf
#define SERIAL_MESSAGE_SIZE (1024*3)
/// maximum amount of characters that can be buffered between SERIAL_printf_poll() calls
#define SERIAL_MESSAGE_BUFFER_SIZE (SERIAL_MESSAGE_SIZE)

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


void serial_printf_flush()
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

void serial_printf_helper(bool block, const char* pFormat, ...)
{
    va_list ap;
    uint32_t     num_written;

    va_start(ap, pFormat);
    num_written = vsnprintf(printf_buffer, SERIAL_MESSAGE_SIZE, pFormat, ap);
    va_end(ap);
    if (num_written > 0)
    {
        if (block) // if blocking
        {
            serial_printf_flush();
            while (!transfer_complete);
            for (size_t i = 0; i<num_written; ++i)
                usart_send_blocking(USART3, (uint8_t)printf_buffer[i]);
        } else {
            // copy data to available tx_buffer
            uint32_t num_chars_available = SERIAL_MESSAGE_BUFFER_SIZE - num_chars_to_send;
            
            if (num_chars_available < num_written) serial_printf_flush();
            
            uint32_t num_chars_to_copy   = std::min(num_written, num_chars_available);
            memcpy(tx_buffer[available_buffer] + num_chars_to_send, printf_buffer, num_chars_to_copy);
            num_chars_to_send += num_chars_to_copy;
        }
    }
}

template <typename ... Ts>
inline void serial_printf(bool block, const char* pFormat, Ts ... ts){
    serial_printf_helper(block, pFormat, ts...);
}

// =====================
//         Clock
// =====================

constexpr uint32_t ahb_clock      = 168000000;
constexpr uint32_t systick_clock  = ahb_clock / 8;

// time to reload systick timer in milliseconds & ticks
static uint16_t systick_period_ms;
static uint32_t systick_period;
// variable used as time reference
static uint32_t time_ms;

void clock_setup()
{
    systick_period_ms = 1;
    systick_period    = systick_clock / 1000 * systick_period_ms;

    // NOTE: if you change clock settings, also change the constants above
    // 168MHz processor/AHB clock
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    // 168MHz / 8 => 21 million counts per second
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    // set reload to every 1 milliseconds
    systick_set_reload(systick_period - 1);
    systick_interrupt_enable();
    systick_clear();
    systick_counter_enable();
}

// systick interrupt, fired on overload
extern "C" void sys_tick_handler(void)
{
    time_ms += systick_period_ms;
}

uint32_t get_ms_from_start()
{
    return time_ms;
}

void setup()
{
    clock_setup();
    serial_init(115200L*4);
    serial_printf(true, "Starting up\r\n");

    rcc_periph_clock_enable(RCC_GPIOB);

    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
    {
        gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 1<<dpin); // Use PB0...
    }
    
	rcc_periph_clock_enable(RCC_GPIOA);

	for (uint8_t apin=0; apin<NR_INPUTS; apin++)
	{
        gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, 1<<apin); // Use PA0...
    }
    
	rcc_periph_clock_enable(RCC_RNG);
	RNG_CR |= RNG_CR_RNGEN;
}

uint32_t real_rand()
{
    while (!(RNG_SR & RNG_SR_DRDY));
    return RNG_DR;
}

inline void shift_left_or(uint32_t* buf, int or_val, int len)
{
    for (int i=len-1;i>=0;i--)
    {
        int next_val = i>0?(buf[i-1]>>31):or_val;
        buf[i] <<= 1;
        buf[i] |= next_val;
    }
}

inline void shift_right(uint32_t* buf, int bits, int len)
{
    for (int i=0;i<len;i++)
    {
        int next_val = (i<(len-1))?buf[i+1]:0;
        buf[i] >>= bits;
        buf[i] |= (next_val<<(32-bits));
    }
}

int main(void)
{
// p = desired significance threshold for statistical significance, should be determined from desired false positive rate corrected for number of comparisons (NR_INPUTS*NR_SAMPLES*NR_OUTPUTS)
// sigma = norm_inverse_cdf(p/2), divide by 2 because we are using a single-tailed test.
// significant if xcors**2 > sum((input-mean_input)**2)*sigma**2

    uint32_t rand_hist[NR_OUTPUTS][HISTORY_LEN] = {0}; // need one extra slot due to the analog value being delayed by 1 cycle
    uint32_t shifted_rand_hist[NR_OUTPUTS][HISTORY_LEN];
    int32_t xcors[NR_INPUTS][NR_SAMPLES][NR_OUTPUTS] = {0};
    uint32_t input[NR_INPUTS] = {0};
    int32_t t = 0;
    uint32_t start_time = 0;
    uint32_t rnd = 0;
    uint8_t calc_phase = 0;

    setup();
    
    while (true)
    {
        int start=time_ms;
        uint32_t rnd = real_rand();

        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            int r = (rnd&(1<<dpin))>>dpin;
            shift_left_or(rand_hist[dpin], r, HISTORY_LEN);
            if (r) gpio_set(GPIOB, 1<<dpin);
            else gpio_clear(GPIOB, 1<<dpin);
        }

        memcpy(shifted_rand_hist, rand_hist, sizeof(rand_hist));
        int i = calc_phase;
        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            shift_right(shifted_rand_hist[dpin], calc_phase+SAMPLE_LAG, HISTORY_LEN);
        }
        do
        {
            //compute actual xcross-correlation
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                {
                    uint32_t matches = shifted_rand_hist[dpin][0] ^ input[apin];
                    xcors[apin][i][dpin] = int32_t(0.999 * xcors[apin][i][dpin]) + __builtin_popcount(matches) - 16; // count the number of 1s in the 32 bits and subtract the mean
                }
            }

            for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
            {
                shift_right(shifted_rand_hist[dpin], 32, HISTORY_LEN);
            }
            
            i += 32;
        } while (i<NR_SAMPLES);
        calc_phase++;
        calc_phase &= 31;

        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
        {
            input[apin] <<= 1;
            input[apin] |= gpio_get(GPIOA, 1<<apin);
        }

        t += 1;
        if (t >= XCOR_DECAY_SAMPLES)
        {
            int time = get_ms_from_start();
            t = 0;
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                {
                    serial_printf(false, "%d %d: ", apin, dpin);//-(12*2+16-31)));
                    for (int16_t i=0;i<NR_SAMPLES;i++)
                    {
                        int32_t tmp = xcors[apin][i][dpin];
                        serial_printf(false, "%5d", (tmp*tmp)/SIGMA2/XCOR_DECAY_SAMPLES);
                    }
                    serial_printf(false, "\r\n");
                }
            }
            serial_printf_flush();
            serial_printf(false, "%d %d\r\n", time - start_time, get_ms_from_start() - time);
            start_time = get_ms_from_start();
        }
    }
}

