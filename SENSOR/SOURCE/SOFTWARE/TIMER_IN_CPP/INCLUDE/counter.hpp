#ifndef COUNTER_HPP
#define COUNTER_HPP

// LIBRARIES USED IN COUNTER
// ---
#include <chrono>
// ---

class counter {
    private:
    // TIME NOW DECLARTION 
    std::chrono::system_clock::time_point now_for_system_clock;   
    std::chrono::high_resolution_clock::time_point now_for_high_resolution_clock;   
    
    
    
    public:
    // CONTRUCTOR DECLRATION AND IMPLEMENTATION 
    counter(){
        now_for_system_clock            = std::chrono::system_clock::now();
        now_for_high_resolution_clock  = std::chrono::high_resolution_clock::now();
    };

    void print_time_now_for_system_clock();
    void update_time_now_for_system_clock();
    long int get_time_now_for_system_clock();
    void print_time_now_for_high_resolution_clock();
    void update_time_now_for_high_resolution_clock();
    long int get_time_now_for_high_resolution_clock();
    
};


#endif