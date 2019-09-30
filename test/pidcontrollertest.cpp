/** @file pidcontrollertest.cpp
* @brief Tests for PID Controller
* @author Kamakshi Jain, Rohan Singh
* @copyright  : This code is developed for the course ENPM808X.
*/
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "pidcontroller.hpp"

/**
 * @brief Test to check functionality of get and set functions for gains.
 * Also checks for correct default parameter initialization.
 */
TEST(PIDController, TestGainChange) {
  std::shared_ptr <PIDController> testPID;
  testPID = std::make_shared <PIDController> ();
  std::vector<double> testGains = testPID->GetPIDGains();
  ASSERT_EQ(0, testGains[0]);
  ASSERT_EQ(0, testGains[1]);
  ASSERT_EQ(0, testGains[2]);
  testPID->SetPIDGains(2.2, 4.4, 6.6);
  testGains = testPID->GetPIDGains();
  ASSERT_EQ(2.2, testGains[0]);
  ASSERT_EQ(4.4, testGains[1]);
  ASSERT_EQ(6.6, testGains[2]);
}

/**
 * @brief Test to check functionality of get and reset functions for error.
 */
TEST(PIDController, TestPIDError) {
  std::shared_ptr<PIDController> testPID;
  testPID = std::make_shared<PIDController>(1, 1, 1);
  ASSERT_GE(testPID->ComputeVelocity(7, 5), 0);
  testPID->ResetPIDErrors();
  std::vector<double> testErrors = testPID->GetPIDErrors();
  ASSERT_EQ(0, testErrors[0]);
  ASSERT_EQ(0, testErrors[1]);
  ASSERT_EQ(0, testErrors[2]);
}

/**
 * @brief This is to test whether the compute velocity gives
 *        correct output after running for 3 iterations and
 *        Info is displayed
 */
TEST(PIDControllerTest, TestCorrectVelocity) {
  std::shared_ptr<PIDController> testPID;
  testPID = std::make_shared<PIDController>(1.1, 2.2, 3.3);
  ASSERT_NEAR(testPID->ComputeVelocity(7, 5), 18.2, 0.0001);
  ASSERT_NEAR(testPID->ComputeVelocity(7, 6), 10.4, 0.0001);
  ASSERT_NEAR(testPID->ComputeVelocity(7, 7), 10.3, 0.0001);
  ASSERT_NEAR(testPID->ComputeVelocity(7, 8), 8.0, 0.0001);
  ASSERT_TRUE(testPID->DisplayPIDInfo());
}

/**
 * @brief Test for ensuring Control Output is within min-max
 *        velocity rate range
 */
TEST(PIDControllerTest, TestVelocityRange) {
  std::shared_ptr<PIDController> testPID;
  testPID = std::make_shared<PIDController>(10, 10, 10);
  ASSERT_LE(testPID->ComputeVelocity(100, 0), 100);
  testPID = std::make_shared<PIDController>(10, 10, 10);
  ASSERT_GE(testPID->ComputeVelocity(-100, 0), -100);
}
