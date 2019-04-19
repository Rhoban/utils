#include <gtest/gtest.h>
#include <rhoban_utils/history/history.h>

using namespace rhoban_utils;

// Verify that the constructor actually works and that the object has
// sane properties.
TEST(history, instantiate)
{
  HistoryDouble h;
  EXPECT_EQ(0, h.size());
}

// Check a range of timestamps/values to push in the history.
TEST(history, addIncrementingTimestamps)
{
  HistoryDouble h;
  h.pushValue(3., 1.);
  EXPECT_EQ(1, h.size());
  h.pushValue(3.2, 3.);
  EXPECT_EQ(2, h.size());
  h.pushValue(3.5, 3.);
  EXPECT_EQ(3, h.size());
  h.pushValue(4.2, 3.);
  EXPECT_EQ(4, h.size());
}

// Violate the condition of incrementing the timestamp
TEST(history, addDecrementingTimeStamps)
{
  HistoryDouble h;
  h.pushValue(3., 1.);
  ASSERT_THROW(h.pushValue(2., 33.), std::logic_error);
}

// Check that putting the same timestamp is harmless.
TEST(history, addSameTimestamp)
{
  HistoryDouble h;
  h.pushValue(4.2, 3.);
  h.pushValue(4.2, 4.);
  EXPECT_EQ(1, h.size());
  EXPECT_DOUBLE_EQ(3., h.back().second);
}

// Check that behavior of front and back methods are consistent.
TEST(history, checkFrontBack)
{
  HistoryDouble h;
  h.pushValue(1., 0.);
  EXPECT_DOUBLE_EQ(1., h.front().first);
  EXPECT_DOUBLE_EQ(1., h.back().first);
  EXPECT_DOUBLE_EQ(0., h.front().second);
  EXPECT_DOUBLE_EQ(0., h.back().second);

  h.pushValue(2., 1.);
  EXPECT_DOUBLE_EQ(1., h.front().first);
  EXPECT_DOUBLE_EQ(0., h.front().second);
  EXPECT_DOUBLE_EQ(2., h.back().first);
  EXPECT_DOUBLE_EQ(1., h.back().second);

  h.pushValue(3., 2.);
  EXPECT_DOUBLE_EQ(1., h.front().first);
  EXPECT_DOUBLE_EQ(0., h.front().second);
  EXPECT_DOUBLE_EQ(3., h.back().first);
  EXPECT_DOUBLE_EQ(2., h.back().second);
}

// Check that casual history values are properly logged.
// TEST(history, loggingBasic)
// {
//   HistoryDouble h;
//   h.startLogging();
//   h.pushValue(1., 2.);
//   h.pushValue(3., 4.);
//   std::ostringstream log;
//   h.stopLogging(log);
//   EXPECT_EQ(std::string{ "1 2\n3 4\n" }, log.str());
// }

// Check that history values starting at 0 are properly logged.
// TODO: Re-enable this test when the issue is fixed
// TEST(history, loggingFromZero)
// {
//   HistoryDouble h;
//   h.startLogging();
//   h.pushValue(0., 1.);
//   h.pushValue(2., 3.);
//   std::ostringstream log;
//   h.stopLogging(log);
//   // FIXME: log.str() returns "";
//   EXPECT_EQ(std::string{ "0 1\n2 3\n" }, log.str());
// }

// Check numbers interpolation
TEST(history, interpolationNumber)
{
  HistoryDouble h(8.);
  h.pushValue(1., 2.);
  h.pushValue(3., 4.);
  h.pushValue(5., 12.);
  EXPECT_DOUBLE_EQ(3., h.interpolate(2.));
  EXPECT_DOUBLE_EQ(3.5, h.interpolate(2.5));
  EXPECT_DOUBLE_EQ(6., h.interpolate(3.5));
  EXPECT_DOUBLE_EQ(8., h.interpolate(4.));
  EXPECT_DOUBLE_EQ(10., h.interpolate(4.5));
}

// Check angles interpolation
TEST(history, interpolationAngleRad)
{
  HistoryAngle h(8.);
  h.pushValue(1., 2.);
  h.pushValue(3., 4.);
  h.pushValue(5., 12.);
  EXPECT_DOUBLE_EQ(3., h.interpolate(2.));
  EXPECT_DOUBLE_EQ(-2.6215653753294101, h.interpolate(2.5));
  EXPECT_DOUBLE_EQ(-1.9495462246455295, h.interpolate(3.5));
  EXPECT_DOUBLE_EQ(-1.4247779607693793, h.interpolate(4.));
  EXPECT_DOUBLE_EQ(-0.90000969689322929, h.interpolate(4.5));
}

// Check that interpolation handles timestamp outside of the range of
// stored ones.
TEST(history, interpolationLimits)
{
  HistoryDouble h;
  EXPECT_NO_THROW(h.interpolate(0.5));
  h.pushValue(1., 2.);
  h.pushValue(3., 4.);
  EXPECT_NO_THROW(h.interpolate(3.5));
}

// Check loadReplay with basic scenario.
// TEST(history, loadReplay)
// {
//   HistoryDouble h;
//   std::istringstream is{ "1 2\n3 4\n" };
//   h.loadReplay(is);
//   EXPECT_DOUBLE_EQ(h.front().first, 1.);
//   EXPECT_DOUBLE_EQ(h.front().second, 2.);
//   EXPECT_DOUBLE_EQ(h.back().first, 3.);
//   EXPECT_DOUBLE_EQ(h.back().second, 4.);
// }

// Check loadReplay with timeshift argument.
// TEST(history, loadReplayTimeShift)
// {
//   HistoryDouble h;
//   std::istringstream is{ "1 2\n3 4\n" };
//   h.loadReplay(is, false, 1.0);
//   EXPECT_DOUBLE_EQ(2., h.front().first);
//   EXPECT_DOUBLE_EQ(2., h.front().second);
//   EXPECT_DOUBLE_EQ(4., h.back().first);
//   EXPECT_DOUBLE_EQ(4., h.back().second);
// }

// Check loadReplay when the input contains a comment.
// TEST(history, loadReplayComment)
// {
//   HistoryDouble h;
//   std::istringstream is{ "1 2\n#comment\n3 4\n" };
//   // Comment detection must stop loading process.
//   h.loadReplay(is);
//   EXPECT_DOUBLE_EQ(1., h.front().first);
//   EXPECT_DOUBLE_EQ(2., h.front().second);
//   EXPECT_DOUBLE_EQ(1., h.back().first);
//   EXPECT_DOUBLE_EQ(2., h.back().second);
// }

// loadReplay with decreasing timestamps
// TEST(history, loadReplayDecreasing)
// {
//   HistoryDouble h;
//   std::istringstream is{ "3 2\n1 4\n" };
//   ASSERT_THROW(h.loadReplay(is), std::runtime_error);
// }

// For binary formatting ensure that what is logged can then be
// replayed.
TEST(history, binary)
{
  HistoryDouble h1;
  h1.startNamedLog("test");
  h1.pushValue(1., 2.);
  h1.pushValue(3., 4.);
  std::ostringstream log;
  h1.freezeNamedLog("test");
  h1.closeFrozenLog("test", log);

  HistoryDouble h2;
  std::istringstream is{ log.str() };
  h2.loadReplay(is);
  EXPECT_DOUBLE_EQ(h2.front().first, 1.);
  EXPECT_DOUBLE_EQ(h2.front().second, 2.);
  EXPECT_DOUBLE_EQ(h2.back().first, 3.);
  EXPECT_DOUBLE_EQ(h2.back().second, 4.);
}

TEST(history, pose_interpolate)
{
  HistoryPose h;

  Eigen::Quaterniond rotation =
    Eigen::AngleAxisd(2, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  h.pushValue(0, pose);

  pose.fromPositionOrientationScale(
    Eigen::Vector3d(1, 4, 5.2),
    Eigen::AngleAxisd(1, Eigen::Vector3d::UnitY()),
    Eigen::Vector3d(1, 1, 1)
  );
  h.pushValue(2, pose);

  Eigen::Affine3d interpolated = h.interpolate(1);
  EXPECT_DOUBLE_EQ(interpolated.translation().x(), 0.5);
  EXPECT_DOUBLE_EQ(interpolated.translation().y(), 2);
  EXPECT_DOUBLE_EQ(interpolated.translation().z(), 2.6);

  Eigen::Vector3d rpy = interpolated.rotation().eulerAngles(0, 1, 2);
  EXPECT_FLOAT_EQ(rpy[0], 0);
  EXPECT_FLOAT_EQ(rpy[1], 0.5);
  EXPECT_FLOAT_EQ(rpy[2], 0);

  interpolated = h.interpolate(0.5);
  rpy = interpolated.rotation().eulerAngles(0, 1, 2);
  EXPECT_FLOAT_EQ(rpy[0], 0);
  EXPECT_FLOAT_EQ(rpy[1], 0.25);
  EXPECT_FLOAT_EQ(rpy[2], 0);
}

TEST(history, pose_binary)
{
  HistoryPose h1;

  Eigen::Affine3d pose;
  pose.fromPositionOrientationScale(
    Eigen::Vector3d(1, 2, 3),
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()),
    Eigen::Vector3d(1, 1, 1)
  );

  h1.startNamedLog("test");
  h1.pushValue(0., pose);
  h1.pushValue(1., pose);
  std::ostringstream log;
  h1.freezeNamedLog("test");
  h1.closeFrozenLog("test", log);

  HistoryPose h2;
  std::istringstream is{ log.str() };
  h2.loadReplay(is);

  Eigen::Affine3d result = h2.interpolate(0.5);
  EXPECT_FLOAT_EQ(result.translation().x(), 1);
  EXPECT_FLOAT_EQ(result.translation().y(), 2);
  EXPECT_FLOAT_EQ(result.translation().z(), 3);
}

TEST(history, collection)
{
  HistoryCollection collection;
  collection.number("a")->pushValue(0, 0);
  collection.number("a")->pushValue(1, 1);
  collection.number("a")->pushValue(2, 2);

  EXPECT_DOUBLE_EQ(collection.number("a")->interpolate(0.5), 0.5);
  ASSERT_THROW(collection.angle("a"), std::logic_error);
}

TEST(history, collection_log)
{
  HistoryCollection collection;
  collection.number("a");
  collection.angle("b");

  collection.startNamedLog("/tmp/test");
  collection.number("a")->pushValue(0, 0);
  collection.number("a")->pushValue(1, 1);
  collection.number("a")->pushValue(2, 2);

  collection.angle("b")->pushValue(1, 2);
  collection.angle("b")->pushValue(3, 4);
  collection.angle("b")->pushValue(5, 12);
  collection.stopNamedLog("/tmp/test");

  HistoryCollection collection2;
  collection2.number("a");
  collection2.angle("b");
  collection2.loadReplays("/tmp/test");

  EXPECT_DOUBLE_EQ(collection2.number("a")->interpolate(0.5), 0.5);
  EXPECT_DOUBLE_EQ(-2.6215653753294101, collection2.angle("b")->interpolate(2.5));
  EXPECT_DOUBLE_EQ(-1.9495462246455295, collection2.angle("b")->interpolate(3.5));
  EXPECT_DOUBLE_EQ(-1.4247779607693793, collection2.angle("b")->interpolate(4.));
  EXPECT_DOUBLE_EQ(-0.90000969689322929, collection2.angle("b")->interpolate(4.5));

  HistoryCollection collection3;
  ASSERT_THROW(collection3.loadReplays("/tmp/test"), std::runtime_error);
}

TEST(history, collection_smaller_ts)
{
  HistoryCollection collection;

  collection.number("a");
  collection.number("a")->pushValue(1.27, 2);
  collection.number("a")->pushValue(2.5, 3);

  collection.number("b");
  collection.number("b")->pushValue(2.27, 2);
  collection.number("b")->pushValue(3.5, 3);

  collection.number("c");

  EXPECT_FLOAT_EQ(collection.smallerTimestamp(), 1.27);

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}  