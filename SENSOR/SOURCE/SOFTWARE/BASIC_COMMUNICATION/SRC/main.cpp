#include <iostream>     // CPP STANDARD LIBRARY INSERTION 
#include <stdio.h>      // C STANDARD LIBRARY INSERTION
#include "i2c.hpp"





int main () 
{
    uint8_t data_return;
    I2Cdev  ic2_imu;
    ic2_imu.initialize();
    ic2.enable()
    ic2.readByte(0x68, 0x75, &data_return);
    std::cout << "DEVICE ID:" << (int)data_return << std::endl;
    return 0;
}