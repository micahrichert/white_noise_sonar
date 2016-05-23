#include <string.h>
#include <stdarg.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>



// serial printf
const int SERIAL_MESSAGE_BUFFER_SIZE = 1024;
static char            tx_buffer[2][SERIAL_MESSAGE_BUFFER_SIZE];
static volatile uint32_t        num_chars_to_send = 0;
static volatile uint32_t        available_buffer  = 0;
static volatile bool   transfer_complete = true;

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
    usart_set_stopbits(USART3, USART_STOPBITS_2); // increase stop bits to improve reliability
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
        usart_enable_tx_dma(USART3);
        dma_enable_stream(DMA1, DMA_STREAM3);
    }
}

void serial_printf_helper(bool block, const char* pFormat, ...)
{
    va_list ap;
    uint32_t     num_written, len;

    if (block) // if blocking
    {
        serial_printf_flush();
        while (!transfer_complete);
    }

    va_start(ap, pFormat);
    len = vsnprintf(NULL, 0, pFormat, ap);
    va_end(ap);

    if (len > SERIAL_MESSAGE_BUFFER_SIZE - num_chars_to_send) serial_printf_flush();

    nvic_disable_irq(NVIC_DMA1_STREAM3_IRQ);

    va_start(ap, pFormat);
    num_written = vsnprintf(tx_buffer[available_buffer] + num_chars_to_send, SERIAL_MESSAGE_BUFFER_SIZE - num_chars_to_send, pFormat, ap);
    va_end(ap);

    if (num_written > 0)
    {
        // copy data to available tx_buffer
        num_chars_to_send += num_written;
    }
    
    nvic_enable_irq(NVIC_DMA1_STREAM3_IRQ);

    if (block) // if blocking
    {
        serial_printf_flush();
        while (!transfer_complete);
    }
}

template <typename ... Ts>
inline void serial_printf(bool block, const char* pFormat, Ts ... ts){
    serial_printf_helper(block, pFormat, ts...);
}

