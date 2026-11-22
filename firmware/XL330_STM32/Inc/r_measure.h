/*
 * r_measure.h — XL330 equivalent resistance R, manual measurement mode
 *
 * Compile switch:
 *   MOTOR_R_TEST 0 (default) -> normal firmware v4
 *   MOTOR_R_TEST 1           -> boot into the R measurement console instead
 * Change the value below, or add MOTOR_R_TEST=1 to the CubeIDE preprocessor symbols
 * (Project Properties > C/C++ Build > Settings > MCU GCC Compiler > Preprocessor).
 * See docs/r_measurement_mode.md.
 */
#ifndef R_MEASURE_H
#define R_MEASURE_H

#ifndef MOTOR_R_TEST
#define MOTOR_R_TEST 0
#endif

#if MOTOR_R_TEST
/* Runs the command console forever (does not return).
 * Needs dxl_init() and USART2 (printf) to be set up first. */
void r_measure_run(void);
#endif

#endif /* R_MEASURE_H */
