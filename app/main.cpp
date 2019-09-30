/** @file main.cpp
* @brief Main file to run Velocity PID Controller
* @author  (Part 1) Kamakshi Jain (driver), Rohan Singh (navigator).
* 			(Part 2) Suyash Yeotikar (driver), Prasheel Renkuntla (navigator)
* @copyright  : This code is developed for the course ENPM808X.  
*/
#include <iostream>
#include <memory>
#include <sstream>
#include "pidcontroller.hpp"

int main() {
    bool newGains = true;
    std::shared_ptr<PIDController> pid = std::make_shared<PIDController>();
    std::shared_ptr<std::string> input = std::make_shared<std::string>();
    pid->DisplayPIDInfo();
    while (newGains) {
        std::cout << "Input target velocity: " << std::endl;
        double targetVel;
        std::getline(std::cin, *input);
        std::stringstream(*input) >> targetVel;
        double actualVel;
        std::cout << "Input actual velocity: " << std::endl;
        std::getline(std::cin, *input);
        std::stringstream(*input) >> actualVel;
        std::cout << "Reset errors: enter 1 or 0:" << std::endl;
        bool resetErrors;
        std::getline(std::cin, *input);
        std::stringstream(*input) >> resetErrors;
        if (resetErrors) {
            pid->ResetPIDErrors();
        }
        std::cout << "Output velocity is: " <<
                pid->ComputeVelocity(targetVel, actualVel) << std::endl;
        pid->DisplayPIDInfo();
        std::cout << "Do you want to update gains: "
        "Enter 1 to update and 0 to exit " << std::endl;
        std::getline(std::cin, *input);
        std::stringstream(*input) >> newGains;
        if ( newGains ) {
            double newKp;
            double newKi;
            double newKd;
            std::cout << "Enter Kp: " << std::endl;
            std::getline(std::cin, *input);
            std::stringstream(*input) >> newKp;
            std::cout << "Enter Ki: " << std::endl;
            std::getline(std::cin, *input);
            std::stringstream(*input) >> newKi;
            std::cout << "Enter Ki: " << std::endl;
            std::getline(std::cin, *input);
            std::stringstream(*input) >> newKd;
            pid->SetPIDGains(newKp, newKi, newKd);
        }
    }
}

