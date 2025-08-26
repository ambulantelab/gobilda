#include "gobilda_robot/motor.hpp"

//using namespace gobilda_robot;

// non_blocking_motor.cpp
#include <fstream>
#include <chrono>

Motor::Motor(int pwm_chip, int pwm_channel) 
    : pwm_chip_number_(pwm_chip), pwm_channel_(pwm_channel) {
        
    pwm_chip_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_number_) + "/";
    pwm_channel_path_ = pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/";
    pwm_duty_cycle_path_ = pwm_channel_path_ + "duty_cycle";
    
    // Try to export the PWM channel (it might already be exported)
    std::ifstream check_file(pwm_channel_path_ + "period");
    if (!check_file) {
        // Channel doesn't exist, need to export it
        std::ofstream export_file(pwm_chip_path_ + "export");
        if (!export_file) {
            throw std::runtime_error("Failed to open export file");
        }
        export_file << pwm_channel_;
        export_file.close();
        
        // Wait for kernel to create the directory
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Export the PWM channel
    std::ofstream export_file(pwm_chip_path_ + "export");
    if (!export_file) {
        throw std::runtime_error("Failed to open export file for writing");
    }
    export_file << pwm_channel_;
    export_file.close();
    
    // Wait for kernel to create the PWM directory (important!)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Check if PWM channel directory was created
    std::ifstream check_file_2(pwm_channel_path_ + "period");
    if (!check_file_2) {
        throw std::runtime_error("PWM channel directory not created after export");
    }
    
    // Set period (20ms for 50Hz = 20,000,000 nanoseconds)
    std::ofstream period_file(pwm_channel_path_ + "period");
    if (!period_file) {
        throw std::runtime_error("Failed to open period file");
    }
    period_file << 20000000;
    period_file.close();
    
    // Set initial duty cycle to neutral (1.5ms)
    std::ofstream duty_file(pwm_channel_path_ + "duty_cycle");
    if (!duty_file) {
        throw std::runtime_error("Failed to open duty_cycle file");
    }
    duty_file << 1500000; // 1.5ms in nanoseconds
    duty_file.close();
    
    // Enable PWM
    std::ofstream enable_file(pwm_channel_path_ + "enable");
    if (!enable_file) {
        throw std::runtime_error("Failed to open enable file");
    }
    enable_file << 1;
    enable_file.close();
    
        // Initialize hardware here if needed
    running_ = true;
    pwm_thread_ = std::thread(&Motor::hardwareThread, this);
}

Motor::~Motor() {
    stop();
}

bool Motor::trySetVelocity(int pulse_width_us) {
    // NON-BLOCKING: Just update the target value
    target_pulse_.store(pulse_width_us);
    return true;
}

void Motor::stop() {
    running_ = false;
    if (pwm_thread_.joinable()) {
        pwm_thread_.join();
    }
    // Do final hardware cleanup here
}

void Motor::hardwareThread() {
    while (running_) {
        int current_pulse = target_pulse_.load();
        setVelocityHardware(current_pulse); // BLOCKING call in separate thread
        
        // Sleep at reasonable rate (e.g., 50Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool Motor::setVelocityHardware(int pulse_width_us) {
    // This is the original BLOCKING implementation
    // but now it runs in a separate thread
    std::ofstream file(pwm_duty_cycle_path_);
    if (!file) return false;
    file << (pulse_width_us * 1000);
    return !file.fail();
}