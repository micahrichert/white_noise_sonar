#include <stdint.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f4/rng.h>
#include <libopencm3/stm32/adc.h>

#include "serial_printf.cpp"
#include "clock.cpp"

#include "config.h"

const int NOISE_LEVEL = 0; // add hysteresis to input when using analog/ADC

inline uint32_t pin_to_ADC(uint8_t apin)
{
    if (apin == 0) return ADC1;
    else if (apin == 1) return ADC2;
    return ADC3;
}

void setup()
{
    clock_setup();
    serial_init(2500000);

    rcc_periph_clock_enable(RCC_GPIOB);

    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
    {
        gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 1<<dpin); // Use PB0...
    }
    
	rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_ADC2);
    rcc_periph_clock_enable(RCC_ADC3);

	for (uint8_t apin=0; apin<NR_INPUTS; apin++)
	{
        uint32_t adc = pin_to_ADC(apin);

        gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, 1<<apin); // Use PA0...

        adc_off(adc);

        adc_disable_external_trigger_regular(adc);
        adc_disable_scan_mode(adc);
        adc_disable_eoc_interrupt(adc);
	    adc_set_single_conversion_mode(adc);
        adc_set_continuous_conversion_mode(adc);
        adc_set_sample_time_on_all_channels(adc, ADC_SMPR_SMP_3CYC); // need to play with this value and the prescaler
        adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);  // prescaler defaults to 2 at startup
        adc_power_on(adc);
    }
    
    // enable the RNG
	rcc_periph_clock_enable(RCC_RNG);
	RNG_CR |= RNG_CR_RNGEN;
}

uint32_t real_rand()
{
    while (!(RNG_SR & RNG_SR_DRDY));
    return RNG_DR;
}

typedef struct
{
    uint8_t magic;
    uint8_t count;
    uint8_t nr_inputs;
    uint8_t nr_outputs;
    uint32_t inputs[NR_INPUTS];
    uint32_t outputs[NR_OUTPUTS];
} __attribute__((packed)) packet_t;

int main(void)
{
    // use triple buffering because the dma transfer takes a little time to initiate
    packet_t packets[3] = {
        {.magic=0xA1, .count=0, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS},
        {.magic=0xA1, .count=0, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS},
        {.magic=0xA1, .count=0, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS},
    };
    int packet_id = 0;
    int previous_input[NR_INPUTS];
    uint32_t start_time;
    uint32_t NOP_count = 0;
    uint32_t t = 0;
    uint32_t transmit_count = 0;
    
    setup();
    
    // select a2d input pin and start the conversion
    for (uint8_t apin=0; apin<NR_INPUTS; apin++)
    {
        uint32_t adc = pin_to_ADC(apin);
        adc_set_regular_sequence(adc, 1, &apin);
    	adc_start_conversion_regular(adc);
    }

    start_time = get_time_us();
    while (true)
    {
        uint32_t rnd = real_rand();

        uint32_t set_bits=0, clear_bits=0;
        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            bool r = (rnd&(1<<(dpin)))>>dpin;
            packets[packet_id].outputs[dpin] <<= 1;
            packets[packet_id].outputs[dpin] |= r;
            if (r) set_bits |= (1<<dpin);
            else clear_bits |= (1<<dpin);
        }
        gpio_set(GPIOB, set_bits);
        gpio_clear(GPIOB, clear_bits);

        for (int tmp=0;tmp<NOP_count;tmp++) asm("nop");

        // read input
        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
        {
    	    uint32_t adc = pin_to_ADC(apin);

            // wait for ADC
            while (!adc_eoc(adc));
            
            int tmp_in = adc_read_regular(adc);

            packets[packet_id].inputs[apin] <<= 1;

            if ((tmp_in - previous_input[apin] < NOISE_LEVEL) & (tmp_in - previous_input[apin] > -NOISE_LEVEL))
                // if we don't exceed the required change threshold then just repeat the previous input value
                packets[packet_id].inputs[apin] |= (packets[packet_id].inputs[apin]&0x2)>>1;
            else
                packets[packet_id].inputs[apin] |= tmp_in > previous_input[apin];

            previous_input[apin] = tmp_in;
        }
        
        t += 1;
        // transmit every 32 steps because we are using variables of type uint32_t to store input and output data
        if (t % 32 == 0)
        {
            //transfer merged_xcors over serial using dma
            packets[packet_id].count = transmit_count;
            transmit_count += 1;
            dma_set_memory_address(DMA1, DMA_STREAM3, (uint32_t)&packets[packet_id]);
            dma_set_number_of_data(DMA1, DMA_STREAM3, sizeof(packet_t));

            usart_enable_tx_dma(USART3);
            dma_enable_stream(DMA1, DMA_STREAM3);

            packet_id += 1; // switch buffers
            packet_id %= sizeof(packets)/sizeof(*packets);

            // delta_time_us/1000000.0 < 32.0/SAMPLE_RATE
            if (((get_time_us() - start_time)*(SAMPLE_RATE/1000)) < (32*(1000000/1000))) NOP_count += 1;
//            else NOP_count -= 1;
            start_time = get_time_us();
        }
    }
}

