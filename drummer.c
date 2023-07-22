///////////////////////////////////////////////////////////////////////
// drummer.c : a simple Drum machine for the Raspberry Pi Pico
//
// (c) 2021 Mike Field <hamster@snap.net.nz>
//
// Really just a test of making an I2S interface. Plays a few different
// bars of drum patterns
//
///////////////////////////////////////////////////////////////////////
// This section is all the hardware specific DMA stuff
///////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include "pico/stdlib.h"
#include <memory.h>
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "pio_i2s.pio.h"
#include "math.h"

#define PIO_I2S_CLKDIV 44.25F
#define N_BUFFERS 4
#define BUFFER_SIZE 49

static int dma_chan;
static uint32_t buffer[N_BUFFERS][BUFFER_SIZE];
static volatile int buffer_playing = 0;
static int buffer_to_fill = 0;

static void dma_handler() {
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_chan;
    // Give the channel a new wave table entry to read from, and re-trigger it
    dma_channel_set_read_addr(dma_chan, buffer[buffer_playing], true);
    if(buffer_playing == N_BUFFERS-1) 
       buffer_playing = 0;
    else
       buffer_playing++;
}


int setup_dma(void) {
     
    //////////////////////////////////////////////////////
    // Set up a PIO state machine to serialise our bits
    uint offset = pio_add_program(pio0, &pio_i2s_program);
    pio_i2s_program_init(pio0, 0, offset, 26, PIO_I2S_CLKDIV);

    //////////////////////////////////////////////////////
    // Configure a channel to write the buffers to PIO0 SM0's TX FIFO, 
    // paced by the data request signal from that peripheral.
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, 1); 
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);

    dma_channel_configure(
        dma_chan,
        &c,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        NULL,
        BUFFER_SIZE,        
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

}


///////////////////////////////////////////////////////////////////////////////////
// Everything from here down is non-h/w specific
///////////////////////////////////////////////////////////////////////////////////
// Drum samples
#include "samples/drum_clap.h"
#include "samples/drum_hihat.h"
#include "samples/drum_kick.h"
#include "samples/drum_perc.h"
#include "samples/drum_snare.h"

#define N_VOICES 6

#define BPM            155
#define SAMPLE_RATE    ((int)(125000000/PIO_I2S_CLKDIV/32/2))
#define BEATS_PER_BAR  4
#define BAR_LEN        72
#define LOOP_BARS      10

// Counters for where we are in time
static int bar = 0;
static int tickOfBar = 0;
static int sampleOfTick = 0;
static int samplesPerBeat;
static int samplesPerTick;


const struct Sounds {
    const int16_t *samples;
    const size_t  len;
} sounds[] = {
   {drum_kick,  sizeof(drum_kick)/sizeof(int16_t)},
   {drum_clap,  sizeof(drum_clap)/sizeof(int16_t)},
   {drum_snare, sizeof(drum_snare)/sizeof(int16_t)},
   {drum_hihat, sizeof(drum_hihat)/sizeof(int16_t)},
   {drum_perc,  sizeof(drum_perc)/sizeof(int16_t)}
};

// Parameters for volume and pan
static struct voice {
    int pan;
    int volume;
    int sample;
    int pos;
    int emph;
} voices[N_VOICES] = {
   { 16,  192, 0, 0, 0},
   {  8,    0, 1, 0, 0},
   { 16,   40, 2, 0, 0},
   { 18,   80, 3, 0, 0},
   { 28,   30, 4, 0, 0},
   {  4,   30, 0, 0, 0}
};


struct Pattern {
   char pattern[N_VOICES][BAR_LEN];
};


const static struct Pattern pattern0 = {{
//"012345678901234567890123456789012345678901234567890123456789012345678901"
  "1        1                     1    1        1                          ",
  "                                                                        ",
  "                  1                                   1                 ",
  "                                                                        ",
  "1        1        1        1        1        1        1        1        ",
  "                                                                        "
}};

const static struct Pattern pattern1 = {{
//"012345678901234567890123456789012345678901234567890123456789012345678901"
  "9                                   1                                   ",
  "                                                                        ",
  "4        1        1        1        3        1        1        1        ",
  "                                                                        ",
  "                                                                        ",
  "                                                                        "
}};

const static struct Pattern pattern2 = {{
//"012345678901234567890123456789012345678901234567890123456789012345678901"
  "9                                   1                                   ",
  "         1                 1                 1                 1        ",
  "1                 1                 1                 1                 ",
  "5        1        1        1        1        1                 1        ",
  "5                          1                 1                          ",
  "         1                          4                          1        "
}};


static const struct Pattern *patterns[LOOP_BARS] = {
   &pattern0,
   &pattern1,
   &pattern2
};


static const int bars[LOOP_BARS] = { 0,0,0,2,2,2,2,2,1,0};

static int index1 = 0;
typedef union 
{
   /* data */
   uint32_t d1;
   int16_t d2[2];
} smpl_data;

static smpl_data s1[] = { 
   {.d2[0] = 0, .d2[1] = 0},
   {.d2[0] = 6000, .d2[1] = 1000},
   {.d2[0] = 12000, .d2[1] = 2000},
   {.d2[0] = 18000, .d2[1] = 3000},   
   {.d2[0] = 24000, .d2[1] = 4000},   
   {.d2[0] = 30000, .d2[1] = 5000},   
   {.d2[0] = 24000, .d2[1] = 4000},   
   {.d2[0] = 18000, .d2[1] = 3000},   
   {.d2[0] = 12000, .d2[1] = 2000},   
   {.d2[0] = 6000, .d2[1] = 1000},   
   {.d2[0] = 0000, .d2[1] = 0000},   
   {.d2[0] = -6000, .d2[1] = -1000},
   {.d2[0] = -12000, .d2[1] = -2000},
   {.d2[0] = -18000, .d2[1] = -3000},   
   {.d2[0] = -24000, .d2[1] = -4000},   
   {.d2[0] = -30000, .d2[1] = -5000},   
   {.d2[0] = -24000, .d2[1] = -4000},   
   {.d2[0] = -18000, .d2[1] = -3000},   
   {.d2[0] = -12000, .d2[1] = -2000},   
   {.d2[0] = -6000, .d2[1] = -1000}   
};

static uint32_t generate_sample(void) {
   int32_t samples[N_VOICES];
   int32_t output[2];

   // Start of new tick?
   if(sampleOfTick == 0) {
      for(int i = 0; i < N_VOICES; i++) {
         if(patterns[bars[bar]]->pattern[i][tickOfBar] != ' ') {
            voices[i].pos  = 1;
            voices[i].emph = (patterns[bars[bar]]->pattern[i][tickOfBar]-'1') * 12;
         }
      }
   }

   for(int i = 0; i < N_VOICES; i++) {
      samples[i] = 0;
      if(voices[i].sample >= 0) {
         samples[i] = sounds[voices[i].sample].samples[voices[i].pos ];
         if(voices[i].pos != 0) {
            voices[i].pos++;
            if(voices[i].pos  >= sounds[voices[i].sample].len) {
               voices[i].pos  = 0;
            }
         }
      }
   }

   output[0] = 0;
   output[1] = 0;
   for(int i = 0; i < N_VOICES; i++) {
      int vol = voices[i].emph + voices[i].volume;
      output[0] += vol * samples[i]*(voices[i].pan);
      output[1] += vol * samples[i]*(32-voices[i].pan);
   }

   sampleOfTick++;
   if(sampleOfTick == samplesPerTick) {
      sampleOfTick = 0;
      tickOfBar++;
      if(tickOfBar == BAR_LEN) {
         tickOfBar = 0;
         bar++;
         if(bar == LOOP_BARS)
            bar = 0;
      }
   }

   // Convert samples back to 16 bit

   //return (((output[0] >> 15)&0xFFFF) << 16) + ((output[1] >> 15) & 0xFFFF);
   if (index1 >= 400)
      index1 = 0;
   smpl_data smpl;

   int16_t ret;
   double arg = (index1++ * 2.0 * 3.142)/400.0;
   ret = (15000.0 * sin(arg)) + 15000.0;
   smpl.d2[0] = ret;
   smpl.d2[1] = ret;
   printf("%d %d %d %d\n\r", index1, smpl.d1, smpl.d2[0], smpl.d2[1]);
   return smpl.d1;
   // if (index1 > 30)
   // {
   //       return 0xffffffff;
   // }
   // else
   //    return 0x7fff7fff;
   // return s1[index1++].d1;
   //return (((ret)&0xFFFF) << 16) + ((ret) & 0xFFFF);
   //return s1[index1++];
   //return 0x2020;
}


static void drum_fill_buffer(void) {
    if(buffer_playing == buffer_to_fill)
       return;

    for(int i = 0; i < BUFFER_SIZE; i++) {
       buffer[buffer_to_fill][i] = generate_sample();
    }
    buffer_to_fill = (buffer_to_fill+1)%N_BUFFERS;
}

void WriteRegister (int reg, int value)
{
   unsigned char cmd[2];

   cmd[0] = reg; // Register 40
   cmd[1] = value; // Ignore SYS CLK Halt
   i2c_write_blocking(i2c1, 0x4D, cmd, 2, false);

}

unsigned char ReadRegister (unsigned char reg)
{
   unsigned char val;
   i2c_write_blocking(i2c1, 0x4D, &reg, 1, true); // true to keep master control of bus
   if (i2c_read_blocking(i2c1, 0x4D, &val, 1, false) != 1)
   {
      printf("Couldn't read register %d \n\r", val);
   }
   printf ("Register %d is %x\n\r", reg, val);
   return val;

}

void CheckRegister(unsigned char reg, unsigned char check)
{
   unsigned char val;
   i2c_write_blocking(i2c1, 0x4D, &reg, 1, true); // true to keep master control of bus
   if (i2c_read_blocking(i2c1, 0x4D, &val, 1, false) != 1)
   {
      printf("Couldn't read register %d \n\r", val);
   }
   if (val != check)
   {
      printf ("Register %d is %x\n\r", reg, val);
   }
}

void SelectPage (unsigned char page)
{
   WriteRegister(0, page);
}

#define I2CCONTROL
int main(void) {
   stdio_init_all();

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(10, GPIO_FUNC_I2C);
    gpio_set_function(11, GPIO_FUNC_I2C);
    gpio_pull_up(10);
    gpio_pull_up(11);    
#ifdef I2CCONTROL
   SelectPage(0);

// Extracted from the reference register sequences from TI at
//   https://e2e.ti.com/support/audio-group/audio/f/audio-forum/428281/pcm5242-slac622-code-example-for-using-non-audio-clock-sources-to-generate-audio-clocks
//
//  The PLL and divider parameters are modified to the specific values for this app.
//  i.e. Sample Frequency is 44.1kHz, BCK frequency is 1.4204MHz. From table this means using the line where K*R/P is 64.
// # Auto divider setting : disable auto config and ignore SCK losses
// w 98 25 1A
   WriteRegister(0x25, 0x1A);
// # PLL P divider to 3
// w 98 14 01     
   WriteRegister(0x14, 0x00); // P=1
// # PLL J divider to 16.D1D2
// w 98 15 10    
   WriteRegister(0x15, 0x32); // J=32
// # PLL D1 divider to J.00
// w 98 16 00     
   WriteRegister(0x16, 0x00); // D1 and D2 are 0
// # PLL D2 divider to J.00
// w 98 17 00
   WriteRegister(0x17, 0x00);
// # PLL R divider to 2
// w 98 18 01
   WriteRegister(0x18, 0x01); // R=2
// # miniDSP CLK divider (NMAC) to 2
// w 98 1B 01
   WriteRegister(0x1B, 0x01); // NMAC=2
// # DAC CLK divider to 16
// w 98 1C 0F
   WriteRegister(0x1C, 0x0F); // NDAC=16
// # NCP CLK divider to 4
// w 98 1D 03
   WriteRegister(0x1D, 0x04); // NCP=4
// # OSC CLK divider is set to one (as its based on the output from the DAC CLK, which is already PLL/16)
// w 98 1E 00
   WriteRegister(0x1E, 0x07); // OSR=8
// # FS setting should be set to single rate speed (48kHZ).
// w 98 22 00
   WriteRegister(0x22, 0x00);
// # IDAC1  sets the number of miniDSP instructions per clock. (set to 1024)
// w 98 23 04
   WriteRegister(0x23, 0x04);
// # IDAC2  
// w 98 24 00
   WriteRegister(0x24, 0x00);

// #############################################


// # Set PLL Clock Source to be BCK instead of SCK
// w 98 0D 10
   WriteRegister(0x0D, 0x10);

   // I2S with 16 bit word length
   WriteRegister(40, 00);

   // Set digital volume to zero
   WriteRegister(61, 0x00);

   // Set digital volume to zero
   WriteRegister(62,0x00);

   // Disable auto mute
   WriteRegister(65, 0x00);

// #### Stand-by request and release ############
// # Stand-by request
// w 98 02 10
   WriteRegister(0x02, 0x10);
// # Stand-by release
// w 98 02 00
   WriteRegister(0x02, 0x00);
// ##############################################

   unsigned char reg;
   for (reg = 0; reg < 255; reg++)
   {
      ReadRegister(reg);
   }
#endif


    ////////////////////////////////////////////////////////////
    // Calculate the timing parameters and fill all the buffers
    ////////////////////////////////////////////////////////////
    samplesPerBeat = SAMPLE_RATE*60/BPM;
    samplesPerTick = samplesPerBeat * BEATS_PER_BAR / BAR_LEN;
    buffer_playing = -1; // To stop the filling routine from stalling
    for(int i = 0; i < N_BUFFERS; i++) {
       drum_fill_buffer();
    }
    buffer_playing = N_BUFFERS-1;

    ////////////////////////////////////////////////////////////
    // Set up the DMA transfers, then call the 
    // handler to trigger the first transfer
    ////////////////////////////////////////////////////////////
    setup_dma();
    dma_handler();

    ////////////////////////////////////////////////////////////
    // Fill buffers with new samples as they are consumed
    // by the DMA transfers
    ////////////////////////////////////////////////////////////
    while (true) {
      drum_fill_buffer();

      // CheckRegister(90, 0);
      // CheckRegister(91, 0x30);
      // CheckRegister(93, 0x20);
      // CheckRegister(94, 0x40);
      // CheckRegister(95, 0x10);
      // CheckRegister(108, 0x33);
      // CheckRegister(109, 0);   
      // //CheckRegister(114, 0);
      // CheckRegister(115, 0);   
      // CheckRegister(118, 0x85);
      // //CheckRegister(119, 0);
      // CheckRegister(120, 0);
      // CheckRegister(4, 0x01);      
      // SelectPage(44);
      // CheckRegister(1, 0xff);
      // sleep_ms(100);
    }
}
