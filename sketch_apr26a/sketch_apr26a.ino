#include <stdint.h>

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
uint32_t start_time = millis();

uint32_t fast_rand(uint32_t x) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

void setup()
{
    Serial.begin(115200L);
    Serial.println("Setting up");
    Serial.print(NR_SAMPLES);
    Serial.print(" ");
    Serial.println(XCOR_DECAY_SAMPLES);

    for (uint8_t apin=0; apin<NR_INPUTS; apin++)
    {
        pinMode(A0+apin, INPUT);
    }

    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
    {
        pinMode(2+dpin, OUTPUT);  // starts with pin 2
    }
    
    ADCSRA = bit (ADEN) | bit (ADPS1) | bit (ADPS2); //64 prescaler because we are running the xcor loop so slow.  //bit (ADPS2);  // set our own prescaler to 16 to run ADC at 1Mhz, gets us ~76Khz sampling

}

void loop()
{
    uint8_t dout = 0;
    for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
    {
        xs[dpin] = fast_rand(xs[dpin]);
        rand_hist[rand_hist_pos][dpin] = xs[dpin] & 0x1;
        dout |= (xs[dpin]&0x1)<<(dpin+2);
    }
    PORTD = dout;  // set the output pins to new random values

    for (uint8_t apin=0; apin<NR_INPUTS; apin++)
    {
        int16_t input = prev_input[apin];
        uint8_t low, high;
        
        // select a2d input pin and start the conversion
        ADMUX  = bit (REFS0) | apin;
        bitSet (ADCSRA, ADSC);  // start a conversion

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
        while (bit_is_set(ADCSRA, ADSC));
        low  = ADCL;
        high = ADCH;
        prev_input[apin] = (high << 8) | low;
    }

    rand_hist_pos += 1;
    if (rand_hist_pos >= NR_SAMPLES+1) rand_hist_pos = 0;

    t += 1;
    if (t >= XCOR_DECAY_SAMPLES)
    {
        t = 0;
        Serial.println(millis() - start_time);
        start_time = millis();
        for (uint8_t apin=0; apin<NR_INPUTS; apin++)
        {
            for (uint8_t dpin=0; dpin<NR_OUTPUTS; dpin++)
            {
                Serial.print(apin, 1);
                Serial.print(" ");
                Serial.print(dpin, 1);
                Serial.print(": ");
                for (int16_t i=NR_SAMPLES-1;i>=0;i--)
                {
                    Serial.print(xcors[apin][i][dpin]>>16);
                    Serial.print(" ");
                    xcors[apin][i][dpin] = 0;
                }
                Serial.println();
            }
        }
    }
}

