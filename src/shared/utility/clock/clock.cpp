#include "../../../nuclear/src/clock.hpp"
// #include "nuclear"
#include "extension/hal/SFR.hpp"
#include "stm32f7xx.h"

namespace utility {
namespace clock {

    template <typename Timer, bool down>
    void init_timer() {

        // Enable the device clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

        // Set the registers
        Timer::reg->CR1 = Timer::reg->CR1 & ~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD | TIM_CR1_ARPE | TIM_CR1_UDIS)
                          | (down ? TIM_CR1_DIR : 0);
        Timer::reg->ARR = 0xFFFFFFFF;  // Auto Reload
        Timer::reg->PSC = 32;          // Prescaler
        // Timer::reg->CNT = 1000000;     // Set the timer to 0 initially

        /* Generate an update event to reload the Prescaler */
        Timer::reg->EGR = TIM_EGR_UG;

        /// Reset the SMS, TS, ECE, ETPS and ETRF bits after which we will be using the internal clock
        Timer::reg->SMCR =
            Timer::reg->SMCR
            & ~(TIM_SMCR_SMS | TIM_SMCR_MSM | TIM_SMCR_TS | TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP);

        Timer::reg->CR2 &= ~(TIM_CR2_MMS);

        Timer::reg->CR1 |= TIM_CR1_CEN;
    }


    void initialise() {

        using Timer2 = extension::hal::SFR<TIM_TypeDef, TIM2_BASE>;
        using Timer5 = extension::hal::SFR<TIM_TypeDef, TIM5_BASE>;

        // Enable the device clocks
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

        // NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
        // NVIC_EnableIRQ(TIM2_IRQn);
        NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
        NVIC_EnableIRQ(TIM5_IRQn);

        // Do some basic common config
        init_timer<Timer2, false>();
        init_timer<Timer5, true>();

        // Enable the interrupt
        Timer5::reg->DIER |= TIM_DIER_UIE;
    }

}  // namespace clock
}  // namespace utility

namespace NUClear {


clock::time_point clock::now() { return time_point(duration(uint32_t(TIM2->CNT))); }

void timer::start(const NUClear::clock::duration& duration) {
    using Timer = ::extension::hal::SFR<TIM_TypeDef, TIM5_BASE>;

    // Set the count and make sure the timer is enabled
    Timer::reg->CR1 |= TIM_CR1_UDIS;
    Timer::reg->CNT = std::chrono::duration_cast<NUClear::timer::duration>(duration).count();
    Timer::reg->CR1 &= ~TIM_CR1_UDIS;
}
}  // namespace NUClear

extern "C" {
void TIM2_IRQHandler() {}
void TIM5_IRQHandler() {
    using Timer = extension::hal::SFR<TIM_TypeDef, TIM5_BASE>;

    Timer::reg->SR = ~TIM_IT_UPDATE;
    // NUClear::extension::timer_fired_interrupt();
}
}