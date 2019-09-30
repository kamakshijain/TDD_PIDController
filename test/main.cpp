/** @file main.cpp
* @brief Main file to run test
* @author Kamakshi Jain, Rohan Singh
* @copyright  : This code is developed for the course ENPM808X.
*/
#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
