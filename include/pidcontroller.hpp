/** @file pidcontroller.hpp
* @brief Header file with funtion declarations
* @author Kamakshi Jain, Rohan Singh
* @copyright  : This code is developed for the course ENPM808X.  
*/

#pragma once

#include <iostream>
#include <vector>

/**
 * @brief Class for 1-D velocity PID Controller       
 */
class PIDController {
 public:
    /**
     * @brief Constructs pidController object to default values (0.0) 
     */
    PIDController();

    /**
     * @brief Contructs PIDController object with given gain values
     * @param kp is the proportional gain
     * @param ki is the integral gain
     * @param kd is the differential gain
     */
    PIDController(double kp, double ki, double kd);

    /**
     * @brief Destroys the PIDController object
    */
    ~PIDController();

    /**
     * @brief Computes the new velocity with given setpoint and actual velocity.
     *
     * FORMULA : 
     *   output velocity = actual + velocity rate
     *   where,
     *          velocity rate = kp*errorCurr + kd*errorDiff + ki*errorSum
     *
     * NOTE : PID assumed to run at constant frequency and hence "delta t" is 
     *        constant and not explicitly included in equation
     *
     * @param targetsetpoint is the desired set velocity
     * @param actual is the current velocity
     * @return
     */
    double ComputeVelocity(double targetSetpoint, double actual);

    /**
     * @brief Limits the system's velocity rate
     *
     * FORMULA : 
     *   minVelocityRate <= velocity rate <= maxVelocityRate
     *  
     * @param velocityrate Unrestricted velocity rate
     * @return Restrictd velocity rate
     */
    double RestrictVelocityRate(double velocityRate);

    /**
     * @brief Returns vector of PID Gains
     * @return Vector with the proportional, integral and
     *  differential gains
     */
    std::vector<double> GetPIDGains();

    /**
     * @brief Sets PID Gains for the controller
     * @param kp is the proportional gain
     * @param ki is the integral gain
     * @param kd is the differential gain
     * @return Void
     */
    void SetPIDGains(double kp, double ki, double kd);

    /**
     * @brief Returns vector of PID Errors
     * @return Vector with the Current Error, Previous Error,
     *  Error Sum, Error Differential
     */
    std::vector<double> GetPIDErrors();

    /**
     * @brief Resets Error Values to default (0.0)
     * @return Void
     */
    void ResetPIDErrors();

    /**
    * @brief Displays PID Information
    *        Displays the Current, Target, and Output Velocity, the gains and the errors
    * @return Void
    */
    void DisplayPIDInfo();

 private:
    double kp;                   // Proportional Gain
    double ki;                   // Integral Gain
    double kd;                   // Differential Gain
    double errorCurr;            // Velocity Setpoint - Actual Velocity
    double errorPrev;            // From the previous PID loop iteration
    double errorSum;             // Previous Error Sum + Current Error
    double errorDiff;            // Current Error - Previous Error
    double commandedVelocity;    // Output Velocity from the Controller
    double minVelocityRate = -100;      // Minimum Threshold for acceleration
    double maxVelocityRate = 100;       // Maximum Threshold for acceleration
};

