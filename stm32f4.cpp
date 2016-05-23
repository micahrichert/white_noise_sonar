#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f4/rng.h>
#include <libopencm3/stm32/adc.h>

#include "serial_printf.cpp"
#include "clock.cpp"

#define USE_BUILTIN 0  // slow on stm32 but may be fast on other chips
#define USE_PARALLEL_BITS 0  // slower than lookup but doesn't use any memory
#define USE_LOOKUP 1  // fastest solution on stm32 but uses 64K of memory
#include "popcount.cpp"

/*
Chi2 significance threshold table generated with the following python code:

import scipy.stats
import numpy as np
cum_p_thresh = 1.0-(1.0-scipy.stats.norm.cdf(3.0))*2  # 3.0 sigmas (1 in 370 false positive) works pretty well
thresh = scipy.stats.chi2.ppf(cum_p_thresh, np.arange(1, 100))
np.set_printoptions(precision=2)
print repr(thresh)

*/
const float chi2_threshs[100] = {  9.0 ,   11.83,   14.16,   16.25,   18.21,   20.06,   21.85,
         23.57,   25.26,   26.9 ,   28.51,   30.1 ,   31.66,   33.2 ,
         34.71,   36.22,   37.7 ,   39.17,   40.63,   42.08,   43.52,
         44.94,   46.36,   47.76,   49.16,   50.55,   51.93,   53.31,
         54.68,   56.04,   57.4 ,   58.75,   60.1 ,   61.44,   62.77,
         64.1 ,   65.43,   66.75,   68.07,   69.38,   70.69,   71.99,
         73.3 ,   74.6 ,   75.89,   77.18,   78.47,   79.76,   81.04,
         82.32,   83.6 ,   84.87,   86.14,   87.41,   88.68,   89.94,
         91.2 ,   92.46,   93.72,   94.98,   96.23,   97.48,   98.73,
         99.98,  101.22,  102.46,  103.7 ,  104.94,  106.18,  107.42,
        108.65,  109.89,  111.12,  112.35,  113.57,  114.8 ,  116.03,
        117.25,  118.47,  119.69,  120.91,  122.13,  123.35,  124.56,
        125.77,  126.99,  128.2 ,  129.41,  130.62,  131.83,  133.03,
        134.24,  135.44,  136.65,  137.85,  139.05,  140.25,  141.45,
        142.65};

#define HIST_TYPE uint32_t
#define HIST_TYPE_BITS (sizeof(HIST_TYPE)*8)

#define RECORD_INPUT 0

const unsigned int NOISE_LEVEL = 30; // add hysteresis to input when using analog/ADC
const bool USE_ANALOG_INPUT = true;
const unsigned int NR_OUTPUTS = 1;
const unsigned int NR_INPUTS = 1;
const float MAX_DISTANCE = 1.5;//1.5; // m
const int SAMPLE_LAG = 0;  // must be a multiple of HIST_TYPE_BITS
const unsigned int MERGE_BIN_CNT = 1;
const float MAX_SPEED = 1.1;  // m/s
const unsigned int NR_SPEEDS = 10;
#if (RECORD_INPUT)
    const float XCOR_DECAY_TIME = 1.0;
    const unsigned int NOP_LOOP = 270;
#else
    const float XCOR_DECAY_TIME = 0.10;//0.05;//1.0; // s
    const unsigned int NOP_LOOP = 0;
#endif
//const float SAMPLE_RATE = 75000; // Hz //3in, 2out at 1.5m
//const float SAMPLE_RATE = 75000; // Hz //3in, 2out at 1.5m
//const float SAMPLE_RATE = 88000; // Hz //2in, 2out at 1.5m
//const float SAMPLE_RATE = 111500; // Hz //1in, 2out at 1.5m
//const float SAMPLE_RATE = 47000; // Hz //1in, 2out at 1.5m, 10speeds
//const float SAMPLE_RATE = 89500; // Hz //4in, 1out at 1.5m
//const float SAMPLE_RATE = 96500; // Hz //3in, 1out at 1.5m
//const float SAMPLE_RATE = 121000; // Hz //2in, 1out at 1.5m
//const float SAMPLE_RATE = 140000; // Hz //1in, 1out at 1.5m
const float SAMPLE_RATE = 77000; // Hz //1in, 1out at 1.5m, 10speeds
const float SPEED_OF_SOUND = 340.3/2.0; // m/s, divide by 2 because sound travels out and back

// dependant variables
const float DISTANCE_PER_SAMPLE = SPEED_OF_SOUND/SAMPLE_RATE;
const float ACTUAL_OUTPUT_BIN_DISTANCE = MERGE_BIN_CNT*DISTANCE_PER_SAMPLE; // m, see DESIRED_OUTPUT_BIN_DISTANCE
const unsigned int NR_BINS = (int(MAX_DISTANCE/DISTANCE_PER_SAMPLE)+31)/32*32-1; // should be multiple of 32-1, round up //int(MAX_DISTANCE/DISTANCE_PER_SAMPLE)/MERGE_BIN_CNT*MERGE_BIN_CNT; // make sure NR_BINS is a multple of merge
const int XCOR_DECAY_SAMPLES = SAMPLE_RATE * XCOR_DECAY_TIME;
const float XCOR_DECAY_RATE = 1.0 - 1.0/XCOR_DECAY_SAMPLES;
const float MAX_SPEED_DILATION = MAX_SPEED/DISTANCE_PER_SAMPLE/SAMPLE_RATE;
const float SPEED_DILATION_STEP = MAX_SPEED_DILATION/NR_SPEEDS;
const unsigned int EXTRA_DILATION_BINS = int(MAX_SPEED_DILATION*XCOR_DECAY_SAMPLES);
const unsigned int HISTORY_LEN = NR_BINS/32 + (SAMPLE_LAG+EXTRA_DILATION_BINS+31)/32;

inline uint32_t pin_to_ADC(uint8_t apin)
{
    if (apin == 0) return ADC1;
    else if (apin == 1) return ADC2;
    return ADC3;
}

void setup()
{
    clock_setup();
    serial_init(115200*16);
//    serial_printf(true, "Starting up\r\n");

    rcc_periph_clock_enable(RCC_GPIOB);

    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
    {
        gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 1<<dpin); // Use PB0...
    }
    
	rcc_periph_clock_enable(RCC_GPIOA);
    if (USE_ANALOG_INPUT)
    {
	    rcc_periph_clock_enable(RCC_ADC1);
	    rcc_periph_clock_enable(RCC_ADC2);
	    rcc_periph_clock_enable(RCC_ADC3);
    }

	for (uint8_t apin=0; apin<NR_INPUTS; apin++)
	{
	    if (USE_ANALOG_INPUT)
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
        } else {
            gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, 1<<apin); // Use PA0...
        }
    }
    
    // enable the RNG
	rcc_periph_clock_enable(RCC_RNG);
	RNG_CR |= RNG_CR_RNGEN;
	
	popcount_setup();
}

uint32_t real_rand()
{
    while (!(RNG_SR & RNG_SR_DRDY));
    return RNG_DR;
}

inline void insert_bit(HIST_TYPE* buf, HIST_TYPE bit, int16_t loc)
{
    int i = loc / HIST_TYPE_BITS;
    int offset = loc % HIST_TYPE_BITS;
    buf[i] = (buf[i]&(~(HIST_TYPE(1)<<offset))) | (bit << offset);
}

typedef struct
{
    uint32_t magic;
    uint32_t magic2;
    uint32_t nr_samples;
    uint32_t nr_bins;
    uint32_t merge_bin_cnt;
    uint32_t nr_inputs;
    uint32_t nr_outputs;
    uint32_t cycle_time;
    uint32_t io_time;
#if (RECORD_INPUT)
    uint32_t merged_xcors[NR_BINS/MERGE_BIN_CNT][NR_INPUTS][NR_OUTPUTS];
#else
    float merged_xcors[NR_BINS/MERGE_BIN_CNT][NR_INPUTS][NR_OUTPUTS];
#endif
} __attribute__((packed)) merged_xcors_struct_t;

int main(void)
{
// p = desired significance threshold for statistical significance, should be determined from desired false positive rate corrected for number of comparisons (NR_INPUTS*NR_BINS*NR_OUTPUTS)
// sigma = norm_inverse_cdf(p/2), divide by 2 because we are using a single-tailed test.
// significant if xcors**2 > sum((input-mean_input)**2)*sigma**2

    int32_t xcors[NR_SPEEDS][NR_BINS+EXTRA_DILATION_BINS][NR_INPUTS][NR_OUTPUTS] = {0};
    merged_xcors_struct_t merged_xcors_structs[2] = {
        {.magic=0x12345678, .magic2=0x9ABCDEF0, .nr_samples=XCOR_DECAY_SAMPLES, .nr_bins=NR_BINS, .merge_bin_cnt=MERGE_BIN_CNT, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS},
        {.magic=0x12345678, .magic2=0x9ABCDEF0, .nr_samples=XCOR_DECAY_SAMPLES, .nr_bins=NR_BINS, .merge_bin_cnt=MERGE_BIN_CNT, .nr_inputs=NR_INPUTS, .nr_outputs=NR_OUTPUTS}};
    int merged_xcors_ind = 0;
    HIST_TYPE rand_hist[NR_OUTPUTS][HISTORY_LEN] = {0};
    HIST_TYPE input[NR_INPUTS] = {0};
    int previous_input[NR_INPUTS] = {0};
    uint32_t t = 0;
    uint32_t start_time = 0;
    uint32_t rnd = 0;
    int calc_phase;
    float dilations[NR_SPEEDS] = {0};
    int16_t bit_pos = 0;
    int hist_i;
    int xcor_i[NR_SPEEDS];
    int decay_i = 0;
    int r;
    bool adc_too_slow = false;
    int tmp_in;
    int cycle_time=0, io_time=0;
    bool first_time = true;
    bool do_reset = true;
    bool reset_done = true;
    int reset_cnt = 0;
    
    setup();
    
    calc_phase = (-1 & (HIST_TYPE_BITS-1)); // & with 31 to ensure it is always between 0 and 31, even when negative; % keeps sign.

    // select a2d input pin and start the conversion
    for (uint8_t apin=0; apin<NR_INPUTS; apin++)
    {
        if (USE_ANALOG_INPUT)
        {
            uint32_t adc = pin_to_ADC(apin);
            adc_set_regular_sequence(adc, 1, &apin);
        	adc_start_conversion_regular(adc);
        }
    }

    while (true)
    {
        int start=get_time_us();
        rnd = real_rand();

        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            r = (rnd&(1<<dpin))>>dpin;
            insert_bit(rand_hist[dpin], r, bit_pos);
            if (r) gpio_set(GPIOB, 1<<dpin);
            else gpio_clear(GPIOB, 1<<dpin);
        }

        hist_i = (bit_pos+(HIST_TYPE_BITS-1)+1+SAMPLE_LAG)/HIST_TYPE_BITS; // the 32bits we want start 31 bits back (positive values are back in time)
        for (int s=0; s<NR_SPEEDS;s++) xcor_i[s] = calc_phase + dilations[s];
#if (!RECORD_INPUT)
        do
        {
            if (hist_i>=HISTORY_LEN) hist_i-=HISTORY_LEN;
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                HIST_TYPE input_apin = input[apin];
                for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
                {
                    HIST_TYPE shift_hist = rand_hist[dpin][hist_i];
                    int xcor = popcount(shift_hist ^ input_apin)*2 - HIST_TYPE_BITS; // count the number of 1s in the 32 bits *2 (for unit variance) and subtract the mean
                    
                    //compute actual xcross-correlation
                    for (int s=0; s<NR_SPEEDS;s++) xcors[s][xcor_i[s]][apin][dpin] += xcor;
                }
            }
            
            for (int s=0; s<NR_SPEEDS;s++) xcor_i[s] += HIST_TYPE_BITS;
            hist_i++;
        } while (xcor_i[0]<NR_BINS);

        calc_phase += 1;
        calc_phase %= 32;
        for (int s=0; s<NR_SPEEDS;s++)
        {
            dilations[s] += SPEED_DILATION_STEP*s;
        }
#endif
        t += 1;
        if (decay_i == 0)
        {
            if (t >= XCOR_DECAY_SAMPLES)
            {
                merged_xcors_structs[merged_xcors_ind].nr_samples = t;
                t = 0;
                for (int s=0; s<NR_SPEEDS;s++) dilations[s] = 0;
                do_reset = true;
            } else {
                if (do_reset) reset_done = true;
                do_reset = false;
            }
        }

#if (!RECORD_INPUT)
        // gets run once every NR_BINS
        for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
        {
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                if (do_reset)
                {
                    if (MERGE_BIN_CNT == 1)
                        merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] = xcors[0][decay_i][apin][dpin]*xcors[0][decay_i][apin][dpin];
                        for (int s=1; s<NR_SPEEDS;s++)
                            merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] += xcors[s][decay_i][apin][dpin]*xcors[s][decay_i][apin][dpin];
//                    else
//                        merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] += xcors[decay_i][apin][dpin]*xcors[decay_i][apin][dpin];
                    for (int s=0; s<NR_SPEEDS;s++) xcors[s][decay_i][apin][dpin] = 0;
                } else {
                    // a little wasteful to put this assignment here but makes the loop times more consistent
                    merged_xcors_structs[merged_xcors_ind].merged_xcors[decay_i/MERGE_BIN_CNT][apin][dpin] = 0;
                }
            }
        }
        decay_i++;
        if (decay_i >= NR_BINS) decay_i = 0;
#endif
        bit_pos--;
        if (bit_pos<0) bit_pos += HISTORY_LEN*HIST_TYPE_BITS;
        
#if (RECORD_INPUT)
        if (t%32 == 31 && t/32<NR_BINS/MERGE_BIN_CNT)
        {
            for (uint8_t apin=0; apin<NR_INPUTS; apin++)
            {
                merged_xcors_structs[merged_xcors_ind].merged_xcors[t/32][apin][0] = input[apin];
            }
        }
#endif
        
        for (int tmp=0;tmp<NOP_LOOP;tmp++) asm("nop");

        // read input
        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
        {
            input[apin] <<= 1;
            if (USE_ANALOG_INPUT)
            {
        	    uint32_t adc = pin_to_ADC(apin);
                while (!adc_eoc(adc)) adc_too_slow = true;
                
                tmp_in = adc_read_regular(adc);
                if ((tmp_in - previous_input[apin] < NOISE_LEVEL) & (tmp_in - previous_input[apin] > -NOISE_LEVEL))
                    // if we don't exceed the required change threshold then just repeat the previous input value
                    input[apin] |= ((input[apin]&0x2)>>1)*0x1;
                else
                    input[apin] |= (tmp_in > previous_input[apin])*0x1;
                previous_input[apin] = tmp_in;
            } else {
                input[apin] |= (gpio_get(GPIOA, 1<<apin)>>apin)*0x1;
            }
        }

        if (reset_done)
        {
            int start_io_time = get_time_us();
            reset_done = false;
            reset_cnt += 1;
            
            //transfer merged_xcors over serial using dma
            merged_xcors_structs[merged_xcors_ind].cycle_time = cycle_time;
            merged_xcors_structs[merged_xcors_ind].io_time = io_time;
            
            dma_set_memory_address(DMA1, DMA_STREAM3, (uint32_t)&merged_xcors_structs[merged_xcors_ind]);
            dma_set_number_of_data(DMA1, DMA_STREAM3, sizeof(merged_xcors_struct_t));

            usart_enable_tx_dma(USART3);
            dma_enable_stream(DMA1, DMA_STREAM3);
            
            merged_xcors_ind = 1-merged_xcors_ind;

            cycle_time = start_io_time - start_time;
            io_time = get_time_us() - start_io_time;
            start_time = get_time_us();
        }
    }
}

