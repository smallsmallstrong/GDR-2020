// Bring in gtest
#include <gtest/gtest.h>

#include <turtlebot3_arm_controller_exercise/arm_controller_node.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "turtlebot3_arm_controller");
  return RUN_ALL_TESTS();
}
