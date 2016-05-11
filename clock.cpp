#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>

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

uint32_t get_time_ms()
{
    return time_ms;
}

