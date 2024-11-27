#ifndef FIRMWARE_PWM_GEN_H
#define FIRMWARE_PWM_GEN_H

#include <stdio.h>
#include <unistd.h>

/* Descriptor for PWM port and channel, must be initialised using pwm_init() */
typedef struct{
    int chip;
    int channel;
}PWM;

typedef enum{
    normal,
    inversed
}PWM_POLARITY;

/* Initialise pwm port and channel */
int pwm_init(PWM* pwm);

/* Set duty cycle in nanoseconds */
int pwm_duty_cycle(PWM* pwm, int on_period_ns);

/* Set period in nanoseconds */
int pwm_period(PWM* pwm, int period_ns);

/* Set pwm polarity */
int pwm_polarity(PWM* pwm, PWM_POLARITY polarity);

/* Enable pwm */
int pwm_enable(PWM* pwm);

/* Disable pwm */
int pwm_disable(PWM* pwm);

#endif //FIRMWARE_PWM_GEN_H
