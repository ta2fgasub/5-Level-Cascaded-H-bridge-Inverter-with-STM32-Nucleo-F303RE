# 5-Level Cascaded H-Bridge Inverter with STM32 Nucleo-F303RE

This project drives a 5-level cascaded H-bridge inverter using an STM32F303RE (Nucleo-F303RE). It generates a quantized sine reference and maps it to two H-bridge legs. TIM1 and TIM8 provide complementary PWM with dead-time.

## Highlights
- 5-level staircase output from two cascaded H-bridges
- Center-aligned complementary PWM with dead-time
- Sine LUT with modulation index and quantization thresholds
- Bootstrap precharge at startup to charge high-side drivers
- Register-level control (CMSIS), no HAL init in main loop

## Control Flow
1. System init sets clock, LUT, GPIO, timers, and NVIC.
2. TIM1 update interrupt calls `PWM_Update_IRQHandler`.
3. ISR advances the sine phase, quantizes to 5 levels, and updates PWM duty for both bridges.

## Key Parameters
You can tune these in [Core/Src/main.c](Core/Src/main.c):
- `SINE_FREQ`: Output sine frequency (default 50 Hz)
- `PWM_FREQ_HZ`: PWM carrier update rate (default 500 Hz)
- `SINE_SAMPLES`: LUT size (default 256)
- `MODULATION_INDEX`: Output amplitude (default 0.95)
- `FIVE_LEVEL_T1`, `FIVE_LEVEL_T2`: Quantizer thresholds
- `DEADTIME_US`: Complementary dead-time (default 2 us)
- `BOOTSTRAP_PRECHARGE_MS`: Precharge duration (default 6 ms)

## MCU and Clock
- MCU: STM32F303RE (LQFP64)
- Clock: HSI -> PLL, 64 MHz system clock (see `SystemClock_Config`)

## PWM and Bridge Mapping
- TIM1 controls Bridge 1
- TIM8 controls Bridge 2
- Both timers run center-aligned PWM with complementary outputs and dead-time

Bridge level mapping (per update):
- +2: Bridge1=+1, Bridge2=+1
- +1: Bridge1=+1, Bridge2=0
-  0: Bridge1=0,  Bridge2=0
- -1: Bridge1=-1, Bridge2=0
- -2: Bridge1=-1, Bridge2=-1

## Pinout (per code configuration)
| Timer | Channel | Pin |
|------|---------|-----|
| TIM1 | CH1     | PA8 |
| TIM1 | CH1N    | PA7 |
| TIM1 | CH2     | PA9 |
| TIM1 | CH2N    | PA12 |
| TIM8 | CH1     | PB6 |
| TIM8 | CH1N    | PB3 |
| TIM8 | CH2     | PB8 |
| TIM8 | CH2N    | PB0 |

## Build and Flash
1. Open the project in STM32CubeIDE (tested with 1.18+).
2. Build the project.
3. Connect the Nucleo-F303RE via USB.
4. Run or debug to flash the target.

## Notes and Safety
- This project drives power stages. Use proper gate drivers, isolation, and protection.
- Verify dead-time and bootstrap timing for your hardware.
- Ensure no shoot-through on your power stage before applying high voltage.

## Repo Layout
- [Core/](Core/) Application sources and startup code
- [Drivers/](Drivers/) CMSIS and HAL drivers
- [5levelchb.ioc](5levelchb.ioc) CubeMX configuration
