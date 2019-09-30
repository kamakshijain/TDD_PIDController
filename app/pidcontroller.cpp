/** @file pidcontroller.cpp
* @brief Function Definitions for class PIDController
* @author (Part 1) Kamakshi Jain (driver), Rohan Singh (navigator)
*		   (Part 2) Suyash Yeotikar (driver), Prasheel Renkuntla (navigator)
* @copyright : This code is developed for the course ENPM808X.  
*/
#include <iostream>

#include "pidcontroller.hpp"

PIDController::PIDController() {
    this->kp = 0;
    this->ki = 0;
    this->kd = 0;
    this->errorCurr = 0;
    this->errorSum = 0;
    this->errorDiff = 0;
    this->commandedVelocity = 0;
    this->errorPrev = 0;
}

PIDController::PIDController(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->errorCurr = 0;
    this->errorSum = 0;
    this->errorDiff = 0;
    this->commandedVelocity = 0;
    this->errorPrev = 0;
}

PIDController::~PIDController() {
}

double PIDController::ComputeVelocity(double targetSetpoint, double actual) {
    double velocityRate;
    this->errorCurr = targetSetpoint - actual;
    this->errorSum += this->errorCurr;
    this->errorDiff = this->errorCurr - this->errorPrev;
    velocityRate = (this->kp * this->errorCurr) +
                              (this->ki * this->errorSum) +
                              (this->kd * this->errorDiff);
    velocityRate = this->RestrictVelocityRate(velocityRate);
    this->commandedVelocity = actual + velocityRate;
    this->errorPrev = this->errorCurr;
    return this->commandedVelocity;
}

double PIDController::RestrictVelocityRate(double velocityRate) {
    if ((velocityRate < this->maxVelocityRate)
              && (velocityRate > this->minVelocityRate)) {
        return velocityRate;
    } else if (velocityRate >= this->maxVelocityRate) {
        return this->maxVelocityRate;
    }
    return this->minVelocityRate;
}

std::vector<double> PIDController::GetPIDGains() {
    std::vector<double> t{0, 0, 0};
    t[0] = this->kp;
    t[1] = this->ki;
    t[2] = this->kd;
    return t;
}

void PIDController::SetPIDGains(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

std::vector<double> PIDController::GetPIDErrors() {
    std::vector<double> t{0, 0, 0};
    t[0] = this->errorCurr;
    t[1] = this->errorSum;
    t[2] = this->errorDiff;
    return t;
}

void PIDController::ResetPIDErrors() {
    this->errorCurr = 0;
    this->errorSum = 0;
    this->errorDiff = 0;
}

bool PIDController::DisplayPIDInfo() {
    std::cout << "The gains of the PID controller are: " << std::endl;
    std::cout << "kp: " << this->kp << std::endl;
    std::cout << "ki: " << this->ki << std::endl;
    std::cout << "kd: " << this->kd << std::endl;
    std::cout << "The errors of the PID controller are: " << std::endl;
    std::cout << "current error: " << this->errorCurr << std::endl;
    std::cout << "error sum: " << this->errorSum << std::endl;
    std::cout << "Error difference from last: " << this->errorDiff << std::endl;
    return true;
}

