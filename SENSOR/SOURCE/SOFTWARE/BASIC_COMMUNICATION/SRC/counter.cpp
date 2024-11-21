#include "counter.hpp"
#include <iostream>

// Function to print the current time (in ticks)
void counter::print_time_now_for_system_clock() 
{
    std::cout << "Now time: " << this->now_for_system_clock.time_since_epoch().count() << " Size of: "<< sizeof(this->now_for_system_clock) << std::endl;
}

// Function to update the current time
void counter::update_time_now_for_system_clock() 
{
    this->now_for_system_clock = std::chrono::system_clock::now();
}

long int counter::get_time_now_for_system_clock()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(this->now_for_system_clock.time_since_epoch()).count();
};

// Function to print the current time (in ticks)
void counter::print_time_now_for_high_resolution_clock() 
{
    std::cout << "Now time: " << this->now_for_high_resolution_clock.time_since_epoch().count() << " Size of: "<< sizeof(this->now_for_high_resolution_clock) << std::endl;
}

// Function to update the current time
void counter::update_time_now_for_high_resolution_clock() 
{
    this->now_for_high_resolution_clock = std::chrono::high_resolution_clock::now();
}

long int counter::get_time_now_for_high_resolution_clock()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(this->now_for_high_resolution_clock.time_since_epoch()).count();
};