#include <fstream>
#include <string>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>

// You can compile using g++

class HardwarePWMMotor {
private:
    std::string pwm_chip_path_;
    std::string pwm_channel_path_;
    int pwm_channel_;

public:
    HardwarePWMMotor(int pwm_chip_number, int pwm_channel) : pwm_channel_(pwm_channel) {
        pwm_chip_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_number) + "/";
        pwm_channel_path_ = pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/";
        
        std::cout << "Initializing hardware PWM on chip " << pwm_chip_number 
                  << ", channel " << pwm_channel << "..." << std::endl;
        
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
        
        std::cout << "Hardware PWM initialized successfully!" << std::endl;
    }
    
    ~HardwarePWMMotor() {
        try {
            // Return to neutral before cleanup
            trySetVelocity(1500);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Disable PWM
            std::ofstream enable_file(pwm_channel_path_ + "enable");
            enable_file << 0;
            enable_file.close();
            
            // Unexport the PWM channel
            std::ofstream unexport_file(pwm_chip_path_ + "unexport");
            unexport_file << pwm_channel_;
            unexport_file.close();
            
            std::cout << "Hardware PWM cleaned up" << std::endl;
        } catch (...) {
            // Silent cleanup in destructor
        }
    }
    
    bool trySetVelocity(int pulse_width_us) {
        // Convert microseconds to nanoseconds
        int duty_cycle_ns = pulse_width_us * 1000;
        
        std::ofstream duty_file(pwm_channel_path_ + "duty_cycle");
        if (!duty_file) {
            std::cerr << "Failed to open duty_cycle file for writing" << std::endl;
            return false;
        }
        duty_file << duty_cycle_ns;
        return !duty_file.fail();
    }
};

// Global flag for clean shutdown
std::atomic<bool> running{true};

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    running = false;
}

bool ensurePWMReady(int pwm_chip_number, int pwm_channel) {
    std::string channel_path = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_number) + 
                            "/pwm" + std::to_string(pwm_channel) + "/";
    
    // Check if PWM channel directory exists
    std::ifstream check_file(channel_path + "period");
    if (!check_file) {
        // Doesn't exist, try to export it
        std::ofstream export_file("/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_number) + "/export");
        if (!export_file) {
            return false;
        }
        export_file << pwm_channel;
        export_file.close();
        
        // Wait for creation
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Check again
        std::ifstream check_again(channel_path + "period");
        return !!check_again;
    }

    return true;
}

int main(int argc, char* argv[]) {
    // Register signal handler
    signal(SIGINT, signalHandler);
    
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <pwm_chip_number> <pwm_channel>" << std::endl;
        std::cerr << "Example: " << argv[0] << " 0 0" << std::endl;
        std::cerr << "Run 'ls /sys/class/pwm/' to see available PWM chips" << std::endl;
        return 1;
    }
    
    int pwm_chip_number = std::stoi(argv[1]);
    int pwm_channel = std::stoi(argv[2]);

    // In your main function, before creating HardwarePWMMotor:
    if (!ensurePWMReady(pwm_chip_number, pwm_channel)) {
        std::cerr << "Failed to setup PWM channel. Try running with sudo?" << std::endl;
        return 1;
    }
    
    try {
        std::cout << "=== Hardware PWM Motor Test ===" << std::endl;
        std::cout << "Chip: " << pwm_chip_number << ", Channel: " << pwm_channel << std::endl;
        
        HardwarePWMMotor motor(pwm_chip_number, pwm_channel);
        
        std::cout << "\nTesting PWM levels. Press Ctrl+C to exit at any time." << std::endl;
        
        // Test sequence
        const int test_values[] = {1500, 1600, 1700, 1600, 1500, 1400, 1300, 1400, 1500};
        const char* descriptions[] = {
            "NEUTRAL", "SLOW FORWARD", "FAST FORWARD", "SLOW FORWARD", 
            "NEUTRAL", "SLOW REVERSE", "FAST REVERSE", "SLOW REVERSE", "NEUTRAL"
        };
        
        for (int i = 0; i < 9 && running; i++) {
            std::cout << "\n" << (i+1) << "/9: " << descriptions[i] 
                      << " (" << test_values[i] << "μs)" << std::endl;
            
            if (motor.trySetVelocity(test_values[i])) {
                std::cout << "PWM set successfully" << std::endl;
            } else {
                std::cout << "Failed to set PWM" << std::endl;
            }
            
            // Wait for 3 seconds or until interrupted
            auto start = std::chrono::steady_clock::now();
            while (running && 
                   std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        if (running) {
            std::cout << "\nTest completed successfully!" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
