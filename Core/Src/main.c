#include "stm32f3xx.h"
#include <math.h>
#include <stdint.h>

#define PI_F 3.14159265358979323846f

#define SINE_FREQ 50.0f
#define PWM_FREQ_HZ 500u
#define ISR_RATE_HZ ((float)PWM_FREQ_HZ)
#define SINE_SAMPLES 256
#define MODULATION_INDEX 0.95f
#define BOOTSTRAP_PRECHARGE_MS 6u

#define TIMER_CLK_HZ 64000000u
#define PWM_PERIOD ((TIMER_CLK_HZ / (PWM_FREQ_HZ * 2u)) - 1u)
#define PWM_PRESCALER 0
#define DEADTIME_US 2u
/* TIM1 DTG=0x80 at 64MHz gives 128*tDTS = 2.0us dead-time. */
#define TIM1_DTG_2US_AT_64MHZ 0x80u
#define PRECHARGE_TICKS ((uint32_t)((((uint64_t)BOOTSTRAP_PRECHARGE_MS) * PWM_FREQ_HZ + 999u) / 1000u))

#define FIVE_LEVEL_ENABLE 1u
#define FIVE_LEVEL_T1 0.2f
#define FIVE_LEVEL_T2 0.6f
#define DUTY_LOW_CLAMP 0.01f
#define DUTY_HIGH_CLAMP 0.99f

static float sine_lut[SINE_SAMPLES];
static float phase_accumulator = 0.0f;
static float phase_increment = (SINE_FREQ / ISR_RATE_HZ) * SINE_SAMPLES;

static void SystemClock_Config(void);
static void Generate_Sine_LUT(void);
static void GPIO_Config(void);
static void TIM1_Config(void);
static void TIM8_Config(void);
static void NVIC_Config(void);
static void System_Init(void);
static float clamp_f(float value, float min_val, float max_val);
static int8_t quantize_5level(float ref);
static void bridge_level_to_duty(int8_t level, float *duty_leg_a, float *duty_leg_b);

static float clamp_f(float value, float min_val, float max_val)
{
    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

static int8_t quantize_5level(float ref)
{
    if (ref >= FIVE_LEVEL_T2) {
        return 2;
    }
    if (ref >= FIVE_LEVEL_T1) {
        return 1;
    }
    if (ref <= -FIVE_LEVEL_T2) {
        return -2;
    }
    if (ref <= -FIVE_LEVEL_T1) {
        return -1;
    }
    return 0;
}

static void bridge_level_to_duty(int8_t level, float *duty_leg_a, float *duty_leg_b)
{
    if (level > 0) {
        *duty_leg_a = DUTY_HIGH_CLAMP;
        *duty_leg_b = DUTY_LOW_CLAMP;
        return;
    }
    if (level < 0) {
        *duty_leg_a = DUTY_LOW_CLAMP;
        *duty_leg_b = DUTY_HIGH_CLAMP;
        return;
    }
    *duty_leg_a = DUTY_LOW_CLAMP;
    *duty_leg_b = DUTY_LOW_CLAMP;
}

static void SystemClock_Config(void)
{
    /* HSI -> PLL 64 MHz (no external crystal required, avoids unsupported PLL macros). */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
    }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    RCC->CFGR = 0;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    RCC->CFGR2 = RCC_CFGR2_PREDIV_DIV2;
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_PLLMUL16;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
    }

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }
}

static void Generate_Sine_LUT(void)
{
    for (uint32_t i = 0; i < SINE_SAMPLES; i++) {
        sine_lut[i] = sinf(2.0f * PI_F * (float)i / (float)SINE_SAMPLES);
    }
}


static void GPIO_Config(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    /* TIM1 on GPIOA: PA8/PA7/PA9/PA12 (CH1/CH1N/CH2/CH2N). */
    GPIOA->MODER &= ~(GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9 |
                      GPIO_MODER_MODER12);
    GPIOA->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 |
                     GPIO_MODER_MODER12_1;
    GPIOA->AFR[0] &= ~(0xFu << (7 * 4));
    GPIOA->AFR[1] &= ~((0xFu << ((8 - 8) * 4)) | (0xFu << ((9 - 8) * 4)) |
                      (0xFu << ((12 - 8) * 4)));
    GPIOA->AFR[0] |= (0x6u << (7 * 4));
    GPIOA->AFR[1] |= (0x6u << ((8 - 8) * 4)) | (0x6u << ((9 - 8) * 4)) |
                     (0x6u << ((12 - 8) * 4));

    /* TIM8 on GPIOB: PB6/PB3/PB8/PB0 (CH1/CH1N/CH2/CH2N). */
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER3 | GPIO_MODER_MODER6 |
                      GPIO_MODER_MODER8);
    GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER6_1 |
                     GPIO_MODER_MODER8_1;
    GPIOB->AFR[0] &= ~((0xFu << (0 * 4)) | (0xFu << (3 * 4)) | (0xFu << (6 * 4)));
    GPIOB->AFR[1] &= ~(0xFu << ((8 - 8) * 4));
    GPIOB->AFR[0] |= (0x4u << (0 * 4)) | (0x4u << (3 * 4)) | (0x5u << (6 * 4));
    GPIOB->AFR[1] |= (0xAu << ((8 - 8) * 4));

    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9 |
                       GPIO_OTYPER_OT_12);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_6 |
                       GPIO_OTYPER_OT_8);

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9 |
                      GPIO_PUPDR_PUPDR12);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR6 |
                      GPIO_PUPDR_PUPDR8);

    /* Medium speed reduces edge ringing on gate-drive traces while keeping timing margin. */
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9 |
                        GPIO_OSPEEDER_OSPEEDR12);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR8_0 |
                      GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR12_0;
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR6 |
                        GPIO_OSPEEDER_OSPEEDR8);
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_0 | GPIO_OSPEEDER_OSPEEDR3_0 |
                      GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR8_0;
}

static void TIM1_Config(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->CR1 = 0u;
    TIM1->CR2 = 0u;
    TIM1->SMCR = 0u;
    TIM1->DIER = 0u;
    TIM1->CCMR1 = 0u;
    TIM1->CCER = 0u;
    TIM1->BDTR = 0u;

    TIM1->PSC = PWM_PRESCALER;
    TIM1->ARR = PWM_PERIOD;
    TIM1->RCR = 1u;
    TIM1->CCR1 = PWM_PERIOD / 2u;
    TIM1->CCR2 = PWM_PERIOD / 2u;

    TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_URS;

    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE;

    /* 2us dead-time for complementary outputs (configured for 64MHz timer clock). */
    TIM1->BDTR = (TIM1_DTG_2US_AT_64MHZ << TIM_BDTR_DTG_Pos) | TIM_BDTR_OSSR | TIM_BDTR_OSSI;

    TIM1->EGR |= TIM_EGR_UG;
    TIM1->SR = 0u;
    TIM1->DIER = TIM_DIER_UIE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void TIM8_Config(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    TIM8->CR1 = 0u;
    TIM8->CR2 = 0u;
    TIM8->SMCR = 0u;
    TIM8->DIER = 0u;
    TIM8->CCMR1 = 0u;
    TIM8->CCER = 0u;
    TIM8->BDTR = 0u;

    TIM8->PSC = PWM_PRESCALER;
    TIM8->ARR = PWM_PERIOD;
    TIM8->RCR = 1u;
    TIM8->CCR1 = PWM_PERIOD / 2u;
    TIM8->CCR2 = PWM_PERIOD / 2u;

    TIM8->CR1 |= TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_URS;

    TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM8->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM8->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE;

    /* 2us dead-time for complementary outputs (configured for 64MHz timer clock). */
    TIM8->BDTR = (TIM1_DTG_2US_AT_64MHZ << TIM_BDTR_DTG_Pos) | TIM_BDTR_OSSR | TIM_BDTR_OSSI;

    TIM8->EGR |= TIM_EGR_UG;
    TIM8->SR = 0u;
    TIM8->BDTR |= TIM_BDTR_MOE;
    TIM8->CR1 |= TIM_CR1_CEN;
}

static void NVIC_Config(void)
{
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

static void System_Init(void)
{
    SystemClock_Config();
    Generate_Sine_LUT();
    GPIO_Config();
    TIM1_Config();
    TIM8_Config();
    NVIC_Config();
}

int main(void)
{
    System_Init();

    while (1) {
        __NOP();
    }
}

void PWM_Update_IRQHandler(void)
{
    static uint32_t precharge_ticks = 0;
    static uint8_t precharge_done = 0u;

    if ((TIM1->SR & TIM_SR_UIF) == 0) {
        return;
    }

    TIM1->SR &= ~TIM_SR_UIF;

    phase_accumulator += phase_increment;
    if (phase_accumulator >= (float)SINE_SAMPLES) {
        phase_accumulator -= (float)SINE_SAMPLES;
    }

    uint32_t sine_index = (uint32_t)phase_accumulator;
    float sine_value = MODULATION_INDEX * sine_lut[sine_index];

    /* Bootstrap precharge: force complementary outputs (low-sides) ON briefly. */
    if (precharge_done == 0u) {
        TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
        TIM1->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC2NE);

        TIM8->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
        TIM8->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC2NE);

        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;

        precharge_ticks++;
        if (precharge_ticks < PRECHARGE_TICKS) {
            return;
        }

        TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
        TIM8->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
        precharge_done = 1u;
    }

    float ref = clamp_f(sine_value, -1.0f, 1.0f);
    int8_t level = quantize_5level(ref);

    int8_t bridge1 = 0;
    int8_t bridge2 = 0;

    if (level >= 2) {
        bridge1 = 1;
        bridge2 = 1;
    } else if (level == 1) {
        bridge1 = 1;
        bridge2 = 0;
    } else if (level == 0) {
        bridge1 = 0;
        bridge2 = 0;
    } else if (level == -1) {
        bridge1 = -1;
        bridge2 = 0;
    } else {
        bridge1 = -1;
        bridge2 = -1;
    }

    float duty1_leg_a = DUTY_LOW_CLAMP;
    float duty1_leg_b = DUTY_LOW_CLAMP;
    float duty2_leg_a = DUTY_LOW_CLAMP;
    float duty2_leg_b = DUTY_LOW_CLAMP;

    bridge_level_to_duty(bridge1, &duty1_leg_a, &duty1_leg_b);
    bridge_level_to_duty(bridge2, &duty2_leg_a, &duty2_leg_b);

    TIM1->CCR1 = (uint32_t)(duty1_leg_a * (float)PWM_PERIOD);
    TIM1->CCR2 = (uint32_t)(duty1_leg_b * (float)PWM_PERIOD);
    TIM8->CCR1 = (uint32_t)(duty2_leg_a * (float)PWM_PERIOD);
    TIM8->CCR2 = (uint32_t)(duty2_leg_b * (float)PWM_PERIOD);
}

