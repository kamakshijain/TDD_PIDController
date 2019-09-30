/** @file pidcontroller.cpp
* @brief Function Definitions for class PIDController
* @author Kamakshi Jain, Rohan Singh
* @copyright : This code is developed for the course ENPM808X.  
*/
#include <iostream>
#include "pidcontroller.hpp"

PIDController::PIDController() {
}

PIDController::PIDController(double kp, double ki, double kd) {
}

PIDController::~PIDController() {
}

double PIDController::ComputeVelocity(double targetSetpoint, double actual) {
  return 0.0;
}

double PIDController::RestrictVelocityRate(double velocityRate) {
  return 0.0;
}

std::vector<double> PIDController::GetPIDGains() {
  std::vector<double> t{0, 0, 0};
  return t;
}

void PIDController::SetPIDGains(double kp, double ki, double kd) {
}

std::vector<double> PIDController::GetPIDErrors() {
  std::vector<double> t{0, 0, 0};
  return t;
}

void PIDController::ResetPIDErrors() {
}

void PIDController::DisplayPIDInfo() {
}

