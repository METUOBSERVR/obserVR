#include <stdio.h>
#include "pwm_gen.h"
#include <unistd.h>

int pwm_init(PWM* pwm){
    FILE *fptr;
    char path[48];
    // Unexport first to refresh channel
    sprintf(path,"/sys/class/pwm/pwmchip%d/unexport", pwm->chip);
    fptr = fopen(path, "a");
    fprintf(fptr, "%d", pwm->channel);
    fclose(fptr);
    // Wait a bit
    sleep(1);
    // Export
    sprintf(path,"/sys/class/pwm/pwmchip%d/export", pwm->chip);
    fptr = fopen(path, "a");
    fprintf(fptr, "%d", pwm->channel);
    fclose(fptr);
    return 0;
}

int pwm_duty_cycle(PWM* pwm, int on_period_ns){
    FILE *fptr;
    char path[48];
    sprintf(path,"/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", pwm->chip, pwm->channel);
    fptr = fopen(path, "a");
    fprintf(fptr, "%d", on_period_ns);
    fclose(fptr);
    return 0;
}

int pwm_period(PWM* pwm, int period_ns){
    FILE *fptr;
    char path[48];
    sprintf(path,"/sys/class/pwm/pwmchip%d/pwm%d/period", pwm->chip, pwm->channel);
    fptr = fopen(path, "a");
    fprintf(fptr, "%d", period_ns);
    fclose(fptr);
    return 0;
}

int pwm_polarity(PWM* pwm, PWM_POLARITY polarity){
    FILE *fptr;
    char path[48];
    sprintf(path,"/sys/class/pwm/pwmchip%d/pwm%d/polarity", pwm->chip, pwm->channel);
    fptr = fopen(path, "a");
    if (polarity == inversed) {
        fprintf(fptr, "inversed");
    }
    if (polarity == normal)  {
        fprintf(fptr, "normal");
    }
    fclose(fptr);
    return 0;
}

int pwm_enable(PWM* pwm){
    FILE *fptr;
    char path[48];
    sprintf(path,"/sys/class/pwm/pwmchip%d/pwm%d/enable", pwm->chip, pwm->channel);
    fptr = fopen(path, "a");
    fprintf(fptr, "1");
    fclose(fptr);
    return 0;
}

int pwm_disable(PWM* pwm){
    FILE *fptr;
    char path[48];
    sprintf(path,"/sys/class/pwm/pwmchip%d/pwm%d/enable", pwm->chip, pwm->channel);
    fptr = fopen(path, "a");
    fprintf(fptr, "0");
    fclose(fptr);
    return 0;
}