#include <fstream>
#include <string>
#include <algorithm>

#include <stdexcept>
#include <iostream>
#include <chrono>

#include <thread>
#include <signal.h>
#include <atomic>
#include <cmath>

// You can compile using g++
class Motor {
private:
    std::string pwm_chip_path_;
    std::string pwm_channel_path_;
    int pwm_channel_;

public:
    Motor(int pwm_chip_number, int pwm_channel) : pwm_channel_(pwm_channel) {
        pwm_chip_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_number) + "/";
        pwm_channel_path_ = pwm_chip_path_ + "pwm" + std::to_string(pwm_channel_) + "/";
        
        std::cout << "Initializing hardware PWM on chip " << pwm_chip_number 
                  << ", channel " << pwm_channel << "..." << std::endl;
         
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
    
    ~Motor() {
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

class DiffDriveRobot{
private:
    std::unique_ptr<Motor> left_motor_;
    std::unique_ptr<Motor> right_motor_;

    const int neutral_pw_ = 1480;
    const double max_radians_ = 32.2;
    const int top_fwd = 1950, top_rev = 1050;
    const int min_us = 1000, max_us = 2000;

    const int deadband_fwd = 60;
    const int deadband_rev = 60;
    const double cmd_deadband = 0.20;

    bool invert_right_ = false, invert_left_ = true;

public:
     DiffDriveRobot(std::unique_ptr<Motor> left, std::unique_ptr<Motor> right)
        : left_motor_(std::move(left)), right_motor_(std::move(right)) {
        if (!left_motor_ || !right_motor_) throw std::invalid_argument("null unique_ptr");
    }

    bool InverseKinematics(const double v, const double w){
        // Use Kinematic Equations
        // From Chapter 4 in the book:
        // translational_speed = 0.5 * radius * (w_r + w_l)
        // and angular_speed = (raduis / length) * (w_r - w_l)

        // First solve for the w_r speed
        // i.e. system of equations
        // w_r = (1/R) * (v - (0.5 * L * theta))
        // w_l = (L*theta/R) - w_r
        const double R = 0.08;
        // True value
        const double L = 0.4318;

        double wr = (v + 0.5 * L * w) / R;
        double wl = (v - 0.5 * L * w) / R;

        std::cout << "wheel rad/s  L=" << wl << "  R=" << wr << "\n";

        // Apply hardware inversion ONLY here
        if (invert_left_)  wl = -wl;
        if (invert_right_) wr = -wr;

        int pulse_left  = ConvertRadPulseWidth(wl);
        int pulse_right = ConvertRadPulseWidth(wr);

        // Clamp (safety)
        pulse_left  = std::clamp(pulse_left,  min_us, max_us);
        pulse_right = std::clamp(pulse_right, min_us, max_us);

        std::cout << "pulses (us) L=" << pulse_left << "  R=" << pulse_right << "\n";

        left_motor_->trySetVelocity(pulse_left);
        right_motor_->trySetVelocity(pulse_right);
        return true;
    }

    int ConvertRadPulseWidth(double rad_s) {
        // treat small commands as neutral
        if (std::fabs(rad_s) < cmd_deadband) return neutral_pw_;

        const bool fwd = rad_s > 0.0;
        const double mag = std::min(std::fabs(rad_s), max_radians_); // keep your limit

        // separate spans (your endpoints are asymmetric)
        const double span_fwd = (top_fwd - neutral_pw_);  // e.g., 1950-1480 = 470
        const double span_rev = (neutral_pw_ - top_rev);  // e.g., 1480-1050 = 430

        // bigger static "punch" to beat friction (tune 90–150 µs)
        const double kS_fwd = 120;
        const double kS_rev = 120;

        // realistic effective wheel max under load (tune 8–12)
        const double omega_wheel_max_eff = 32.0;

        const double usable_fwd = span_fwd - kS_fwd;
        const double usable_rev = span_rev - kS_rev;
        const double denom      = (omega_wheel_max_eff - cmd_deadband);
        const double kV_fwd     = usable_fwd / denom;  // µs per rad/s
        const double kV_rev     = usable_rev / denom;

        double pulse = neutral_pw_;
        if (fwd) pulse += kS_fwd + kV_fwd * (mag - cmd_deadband);
        else     pulse -= kS_rev + kV_rev * (mag - cmd_deadband);

        return static_cast<int>(std::llround(std::clamp(pulse, double(min_us), double(max_us))));
    }
};

// Global flag for clean shutdown
std::atomic<bool> running{true};

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    running = false;
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
    
    try {
        std::cout << "=== Hardware PWM Motor Test ===" << std::endl;
        std::cout << "Chip: " << pwm_chip_number << ", Channel: " << pwm_channel << std::endl;
        
        // Create Motor Objects
        // The channel and chip number were taken from the Jetson Orin /sys/class files 
        auto left_motor= std::make_unique<Motor>(pwm_chip_number, pwm_channel);
        auto right_motor  = std::make_unique<Motor>(3, 0);

        // Create Robot Object
        DiffDriveRobot gobilda_robot(std::move(left_motor), std::move(right_motor));
        
        // Signal for moving robot forward
	    const std::pair<double, double> move_forward = {0.5, 0.0}; // m/s | rad/s
	    const std::pair<double, double> turn_90_degrees = {0.0, 0.785}; // m/s | rad/s (45 degrees)
        
	    std::cout << "\nDrive robot forward one meter!\n";
        std::cout << "Sending linear speed: " << move_forward.first<< '\n';
        std::cout << "Sending angular speed: " << move_forward.second << '\n';

        // Compute the desired rad/s for each wheel given the target speeds
        // Inverse Kinematics
        running = gobilda_robot.InverseKinematics(move_forward.first, move_forward.second);
        std::this_thread::sleep_for(std::chrono::seconds(2));  // wait 2 seconds
	
    	// Make the robot turn 90 degrees!
	    std::cout << "\nTurning robot 90 degrees!\n";
        std::cout << "Sending linear speed: " << turn_90_degrees.first<< '\n';
        std::cout << "Sending angular speed: " << turn_90_degrees.second << '\n';

        // Compute the desired rad/s for each wheel given the target speeds
        // Inverse Kinematics
        running = gobilda_robot.InverseKinematics(turn_90_degrees.first, turn_90_degrees.second);
        std::this_thread::sleep_for(std::chrono::seconds(2));  // wait 2 seconds

        if (running) {
            std::cout << "\nTest completed successfully!" << std::endl;
        }
        
        // Send neutral singals to the robot after finishing
        gobilda_robot.InverseKinematics(0.0, 0.0);
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
