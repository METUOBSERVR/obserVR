#include <gpiod.h>
#include <stdio.h>
#include "i2c_tools.h"
#include "pwm_gen.h"

#define PWMCHIP 0

int main(void) {
    PWM pin18pwm = {PWMCHIP, 0};
    pwm_init(&pin18pwm);
    pwm_duty_cycle(&pin18pwm, 125000);
    pwm_period(&pin18pwm, 250000);
    pwm_enable(&pin18pwm);
    printf("Press any key to continue\n");
    getchar();
    pwm_disable(&pin18pwm);
    return 0;
}
