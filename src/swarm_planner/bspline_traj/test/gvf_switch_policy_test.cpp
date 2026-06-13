#include <gtest/gtest.h>

#include <bspline_race/gvf_manager.h>

TEST(GvfSwitchPolicy, HeaderCompilesWithoutGovernorPathShortPolicy)
{
  SUCCEED();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
