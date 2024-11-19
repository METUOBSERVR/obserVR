#include <iostream>
#include "counter.hpp"
#include <thread>





int main () {
    counter timer;


    timer.update_time_now_for_system_clock();
    timer.print_time_now_for_system_clock();
    std::this_thread::sleep_for(std::chrono::microseconds(200));
    timer.update_time_now_for_system_clock();
    timer.print_time_now_for_system_clock();
; 


    timer.update_time_now_for_high_resolution_clock();
    timer.print_time_now_for_high_resolution_clock();
    std::this_thread::sleep_for(std::chrono::microseconds(200));
    timer.update_time_now_for_high_resolution_clock();
    timer.print_time_now_for_high_resolution_clock();




    return 0;
}