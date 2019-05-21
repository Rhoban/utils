#include <gtest/gtest.h>
#include <rhoban_utils/angle.h>
#include <typeinfo>
#include <rhoban_utils/serialization/json_serializable.h>

#define _USE_MATH_DEFINES
#include <cmath>

#define EPSILON std::pow(10, -6)

/*******************************************************
 * Constructor test
 */

TEST(constructor, testSuccess)
{
  // Default
  rhoban_utils::Angle a1(-410.0);
  rhoban_utils::Angle a2(-50.0);
  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle a4(50.0);
  rhoban_utils::Angle a5(410.0);

  EXPECT_EQ(310.0, a1.getValue());
  EXPECT_EQ(310.0, a2.getValue());
  EXPECT_EQ(0.0, a3.getValue());
  EXPECT_EQ(50.0, a4.getValue());
  EXPECT_EQ(50.0, a5.getValue());

  // Degree
  rhoban_utils::Angle b1(-410.0, rhoban_utils::Angle::DEG);
  rhoban_utils::Angle b2(-50.0, rhoban_utils::Angle::DEG);
  rhoban_utils::Angle b3(0.0, rhoban_utils::Angle::DEG);
  rhoban_utils::Angle b4(50.0, rhoban_utils::Angle::DEG);
  rhoban_utils::Angle b5(410.0, rhoban_utils::Angle::DEG);

  EXPECT_EQ(310.0, b1.getValue());
  EXPECT_EQ(310.0, b2.getValue());
  EXPECT_EQ(0.0, b3.getValue());
  EXPECT_EQ(50.0, b4.getValue());
  EXPECT_EQ(50.0, b5.getValue());

  // Radian
  rhoban_utils::Angle c1(M_PI, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle c2(2 * M_PI, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle c3(M_PI / 4, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle c4(9 * M_PI / 4, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle c5(-7 * M_PI / 4, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle c6(-M_PI / 4, rhoban_utils::Angle::RAD);

  EXPECT_NEAR(180.0, c1.getValue(), EPSILON);
  EXPECT_NEAR(0.0, c2.getValue(), EPSILON);
  EXPECT_NEAR(45.0, c3.getValue(), EPSILON);
  EXPECT_NEAR(45.0, c4.getValue(), EPSILON);
  EXPECT_NEAR(45.0, c5.getValue(), EPSILON);
  EXPECT_NEAR(315.0, c6.getValue(), EPSILON);
}

/*******************************************************
 * GetSignedValue test
 */

TEST(getSignedValue, testSuccess)
{
  rhoban_utils::Angle a1(0.0);
  rhoban_utils::Angle a2(50.0);
  rhoban_utils::Angle a3(250.0);

  EXPECT_EQ(0.0, a1.getSignedValue());
  EXPECT_EQ(50.0, a2.getSignedValue());
  EXPECT_EQ(-110.0, a3.getSignedValue());
}

/*******************************************************
 * GetSignedValue test
 */

TEST(getSignedValueRad, testSuccess)
{
  rhoban_utils::Angle a1(0, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle a2(M_PI / 3, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle a3(-M_PI / 3, rhoban_utils::Angle::RAD);
  rhoban_utils::Angle a4(5 * M_PI / 3, rhoban_utils::Angle::RAD);

  EXPECT_NEAR(0.0, a1.getSignedValueRad(), EPSILON);
  EXPECT_NEAR(M_PI / 3, a2.getSignedValueRad(), EPSILON);
  EXPECT_NEAR(-M_PI / 3, a3.getSignedValueRad(), EPSILON);
  EXPECT_NEAR(-M_PI / 3, a4.getSignedValueRad(), EPSILON);
}

/*******************************************************
 * FromXY test
 */

TEST(fromXY, testSuccess)
{
  rhoban_utils::Angle a1 = rhoban_utils::Angle::fromXY(1, 1);
  rhoban_utils::Angle a2 = rhoban_utils::Angle::fromXY(-1, -1);

  EXPECT_EQ(45.0, a1.getValue());
  EXPECT_EQ(225.0, a2.getValue());
}

/*******************************************************
 * Mean test
 */

TEST(mean_stdDev, testSuccessOne)
{
  rhoban_utils::Angle a1(90.0);

  std::vector<rhoban_utils::Angle> angle_vector;
  angle_vector.push_back(a1);

  double stdDev;
  rhoban_utils::Angle madeAngle = rhoban_utils::Angle::mean(angle_vector, &stdDev);

  EXPECT_EQ(a1.getValue(), madeAngle.getValue());
  EXPECT_EQ(0, stdDev);
}

// Tests for random angle
TEST(mean_stdDev, testSuccessRandomAngle)
{
  rhoban_utils::Angle a1(30.0);
  rhoban_utils::Angle a2(292.0);

  rhoban_utils::Angle meanAngle = rhoban_utils::Angle::fromXY((rhoban_utils::cos(a1) + rhoban_utils::cos(a2)) / 2,
                                                              (rhoban_utils::sin(a1) + rhoban_utils::sin(a2)) / 2);

  std::vector<rhoban_utils::Angle> angle_vector;
  angle_vector.push_back(a1);
  angle_vector.push_back(a2);

  double stdDev = 0;

  rhoban_utils::Angle madeAngle = rhoban_utils::Angle::mean(angle_vector, &stdDev);

  EXPECT_EQ(meanAngle.getValue(), madeAngle.getValue());

  // stdDev is at least upper diff between mean and angles
  EXPECT_LT(madeAngle.getValue() - a2.getValue(), stdDev);
  EXPECT_GT(madeAngle.getValue() - a2.getValue() + 5, stdDev);
}

// Tests for epsilon = 30 angle
TEST(mean_stdDev, testSuccessMirror1)
{
  rhoban_utils::Angle a1(30.0);
  rhoban_utils::Angle a2(-30.0);

  std::vector<rhoban_utils::Angle> angle_vector;
  angle_vector.push_back(a1);
  angle_vector.push_back(a2);

  double stdDev = 0;

  rhoban_utils::Angle madeAngle = rhoban_utils::Angle::mean(angle_vector, &stdDev);

  EXPECT_EQ(madeAngle.getSignedValue(), 0);

  // stdDev is at least upper diff between mean and angles
  EXPECT_LT(30.0, stdDev);
  EXPECT_GT(35.0, stdDev);
}

// Tests for epsilon = 45 angle
TEST(mean_stdDev, testSuccessMirror2)
{
  rhoban_utils::Angle a1(45.0);
  rhoban_utils::Angle a2(-45.0);

  std::vector<rhoban_utils::Angle> angle_vector;
  angle_vector.push_back(a1);
  angle_vector.push_back(a2);

  double stdDev = 0;

  rhoban_utils::Angle madeAngle = rhoban_utils::Angle::mean(angle_vector, &stdDev);

  EXPECT_EQ(madeAngle.getSignedValue(), 0);

  // stdDev is at least upper diff between mean and angles
  EXPECT_LT(45.0, stdDev);
  EXPECT_GT(50.0, stdDev);
}

// Tests for epsilon = 90 angle
// Disabled due to the result of stdDev
TEST(mean_stdDev, DISABLED_testSuccessNullMean)
{
  rhoban_utils::Angle a1(90.0);
  rhoban_utils::Angle a2(-90.0);

  std::vector<rhoban_utils::Angle> angle_vector;
  angle_vector.push_back(a1);
  angle_vector.push_back(a2);

  double stdDev = 0;

  rhoban_utils::Angle madeAngle = rhoban_utils::Angle::mean(angle_vector, &stdDev);

  EXPECT_TRUE(madeAngle.getValue() == 0.0 || madeAngle.getValue() == 180.0);

  // stdDev is at least upper diff between mean and angles
  // In this case = 495
  EXPECT_LT(90.0, stdDev);
  EXPECT_GT(95.0, stdDev);
}

/*******************************************************
 * Arcsin test
 */

TEST(arcsin, testSuccess)
{
  double sinus1 = std::sin(0);
  double sinus2 = std::sin(1 * M_PI / 4);
  double sinus3 = std::sin(-1 * M_PI / 2);

  rhoban_utils::Angle angle1 = rhoban_utils::Angle::arcsin(sinus1);
  rhoban_utils::Angle angle2 = rhoban_utils::Angle::arcsin(sinus2);
  rhoban_utils::Angle angle3 = rhoban_utils::Angle::arcsin(sinus3);
  EXPECT_NEAR(angle1.getValue(), 0, 0.00001);
  EXPECT_NEAR(angle2.getValue(), 45.0, 0.00001);
  EXPECT_NEAR(angle3.getValue(), 270.0, 0.00001);
}

/*******************************************************
 * Arccos test
 */

TEST(arccos, testSuccess)
{
  double cosinus1 = std::cos(0);
  double cosinus2 = std::cos(1 * M_PI / 4);
  double cosinus3 = std::cos(M_PI / 2);

  rhoban_utils::Angle angle1 = rhoban_utils::Angle::arccos(cosinus1);
  rhoban_utils::Angle angle2 = rhoban_utils::Angle::arccos(cosinus2);
  rhoban_utils::Angle angle3 = rhoban_utils::Angle::arccos(cosinus3);
  EXPECT_NEAR(angle1.getValue(), 0, 0.00001);
  EXPECT_NEAR(angle2.getValue(), 45.0, 0.00001);
  EXPECT_NEAR(angle3.getValue(), 90.0, 0.00001);
}

/*******************************************************
 * weightedAverage test
 */

TEST(weightedAverage, testSuccessWeight0)
{
  rhoban_utils::Angle a1(46.0);
  rhoban_utils::Angle a2(268.0);
  rhoban_utils::Angle res = rhoban_utils::Angle::weightedAverage(a1, 0.0, a2, 0.0);

  EXPECT_EQ(0.0, res.getValue());
}

TEST(weightedAverage, testSuccessWeight1)
{
  rhoban_utils::Angle a1(45.0);
  rhoban_utils::Angle a2(15.0);
  rhoban_utils::Angle res = rhoban_utils::Angle::weightedAverage(a1, 1.0, a2, 1.0);

  EXPECT_NEAR(30.0, res.getValue(), 0.00001);
}

TEST(weightedAverage, testSuccess)
{
  // cos = sqrt(2) / 2
  // sin = sqrt(2) / 2
  rhoban_utils::Angle a1(45.0);

  // cos = sqrt(3) / 2
  // sin = -1 / 2
  rhoban_utils::Angle a2(-30.0);

  rhoban_utils::Angle res = rhoban_utils::Angle::weightedAverage(a1, 2.0, a2, 4.0);

  // Add of vector with weight =>
  // x = sqrt(2) * 2 / 2 + sqrt(3) * 4 / 2
  // x = sqrt(2) * 2 / 2 + 1 * 4 / 2
  rhoban_utils::Angle expectedRes =
      rhoban_utils::Angle::fromXY(std::sqrt(2.0) + 2.0 * std::sqrt(3.0), std::sqrt(2) + -2.0);

  EXPECT_EQ(expectedRes.getValue(), res.getValue());
}

/*******************************************************
 * equals test
 */

TEST(equals, testSuccessTrue)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(180.0);
  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle a4(0.0);

  EXPECT_TRUE(a1.equals(a2));
  EXPECT_TRUE(a3.equals(a4));
}

TEST(equals, testSuccessFalse)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(310.0);
  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle a4(1.0);

  EXPECT_FALSE(a1.equals(a2));
  EXPECT_FALSE(a3.equals(a4));
}

/*******************************************************
 * operator- test
 */

TEST(minus, testSuccess)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(45.0);
  rhoban_utils::Angle res = a1 - a2;
  EXPECT_EQ(res.getValue(), 135.0);
}

TEST(minus, testSuccess_null)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(0.0);
  rhoban_utils::Angle res12 = a1 - a2;

  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle a4(0.0);
  rhoban_utils::Angle res34 = a3 - a4;

  EXPECT_EQ(res12.getValue(), 180.0);
  EXPECT_EQ(res34.getValue(), 0.0);
}

/*******************************************************
 * operator+ test
 */

TEST(plus, testSuccess)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(45.0);
  rhoban_utils::Angle res = a1 + a2;
  EXPECT_EQ(res.getValue(), 225.0);
}

TEST(plus, testSuccess_null)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(0.0);
  rhoban_utils::Angle res12 = a1 + a2;

  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle a4(0.0);
  rhoban_utils::Angle res34 = a3 + a4;

  EXPECT_EQ(res12.getValue(), 180.0);
  EXPECT_EQ(res34.getValue(), 0.0);
}

/*******************************************************
 * operator== test
 */

TEST(operator_equal_equal, testSuccessTrue)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(180.0);
  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle a4(0.0);

  EXPECT_TRUE(a1 == a2);
  EXPECT_TRUE(a3 == a4);
}

TEST(operator_equal_equal, testSuccessFalse)
{
  rhoban_utils::Angle a1(180.0);
  rhoban_utils::Angle a2(310.0);
  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle a4(1.0);

  EXPECT_FALSE(a1 == a2);
  EXPECT_FALSE(a3 == a4);
}

/*******************************************************
 * operator- global test
 */

TEST(operator_minus_global, testSuccess)
{
  rhoban_utils::Angle a1(90.0);
  rhoban_utils::Angle res1 = -a1;
  rhoban_utils::Angle a2(180.0);
  rhoban_utils::Angle res2 = -a2;
  rhoban_utils::Angle a3(0.0);
  rhoban_utils::Angle res3 = -a3;

  EXPECT_EQ(res1.getValue(), 270.0);
  EXPECT_EQ(res2.getValue(), 180.0);
  EXPECT_EQ(res3.getValue(), 0.0);
}

/*******************************************************
 * operator* global test
 */

TEST(operator_scale_multiplication_global_1, testSuccess)
{
  rhoban_utils::Angle a1(45.0);
  rhoban_utils::Angle res1 = a1 * 3;
  rhoban_utils::Angle res2 = a1 * -1;
  rhoban_utils::Angle res3 = a1 * 0;

  EXPECT_EQ(res1.getValue(), 135.0);
  EXPECT_EQ(res2.getValue(), 315.0);
  EXPECT_EQ(res3.getValue(), 0.0);
}

/*******************************************************
 * operator* global test
 */

TEST(operator_scale_multiplication_global_2, testSuccess)
{
  rhoban_utils::Angle a1(45.0);
  rhoban_utils::Angle res1 = 3 * a1;
  rhoban_utils::Angle res2 = -1 * a1;
  rhoban_utils::Angle res3 = 0 * a1;

  EXPECT_EQ(res1.getValue(), 135.0);
  EXPECT_EQ(res2.getValue(), 315.0);
  EXPECT_EQ(res3.getValue(), 0.0);
}

/*******************************************************
 * operator/ global test
 */

TEST(operator_division_global, testSuccess)
{
  rhoban_utils::Angle a1(90.0);
  rhoban_utils::Angle res1 = a1 / 2;
  rhoban_utils::Angle res2 = a1 / 3;
  rhoban_utils::Angle res3 = a1 / -3;

  EXPECT_EQ(res1.getValue(), 45.0);
  EXPECT_EQ(res2.getValue(), 30.0);
  EXPECT_EQ(res3.getValue(), 330.0);
}

/*******************************************************
 * operator<< global test
 */

TEST(operator_ostream, testSuccess)
{
  std::ostringstream stream1;
  std::ostringstream stream2;
  rhoban_utils::Angle a1(45);
  rhoban_utils::Angle a2(45.99);
  stream1 << a1;
  stream2 << a2;
  EXPECT_EQ("45", stream1.str());
  EXPECT_EQ("45.99", stream2.str());
}

/*******************************************************
 * Sin test
 */

TEST(sin, testSuccess)
{
  rhoban_utils::Angle a1(0.0);
  rhoban_utils::Angle a2(45.0);
  rhoban_utils::Angle a3(270.0);

  double a1Rad = a1.getValue() * M_PI / 180;
  double a2Rad = a2.getValue() * M_PI / 180;
  double a3Rad = a3.getValue() * M_PI / 180;

  EXPECT_EQ(rhoban_utils::sin(a1), std::sin(a1Rad));
  EXPECT_EQ(rhoban_utils::sin(a2), std::sin(a2Rad));
  EXPECT_EQ(rhoban_utils::sin(a3), std::sin(a3Rad));
}

/*******************************************************
 * cos test
 */

TEST(cos, testSuccess)
{
  rhoban_utils::Angle a1(0.0);
  rhoban_utils::Angle a2(45.0);
  rhoban_utils::Angle a3(270.0);

  double a1Rad = a1.getValue() * M_PI / 180;
  double a2Rad = a2.getValue() * M_PI / 180;
  double a3Rad = a3.getValue() * M_PI / 180;

  EXPECT_EQ(rhoban_utils::cos(a1), std::cos(a1Rad));
  EXPECT_EQ(rhoban_utils::cos(a2), std::cos(a2Rad));
  EXPECT_EQ(rhoban_utils::cos(a3), std::cos(a3Rad));
}

/*******************************************************
 * tan test
 */

TEST(tan, testSuccess)
{
  rhoban_utils::Angle a1(0.0);
  rhoban_utils::Angle a2(45.0);
  rhoban_utils::Angle a3(270.0);

  double a1Rad = a1.getValue() * M_PI / 180;
  double a2Rad = a2.getValue() * M_PI / 180;
  double a3Rad = a3.getValue() * M_PI / 180;

  EXPECT_EQ(rhoban_utils::tan(a1), std::tan(a1Rad));
  EXPECT_EQ(rhoban_utils::tan(a2), std::tan(a2Rad));
  EXPECT_EQ(rhoban_utils::tan(a3), std::tan(a3Rad));
}

/*******************************************************
 * deg2rad test
 */

TEST(deg2rad, testSuccess)
{
  EXPECT_EQ(rhoban_utils::deg2rad(0.0), 0.0);
  EXPECT_EQ(rhoban_utils::deg2rad(45.0), M_PI / 4);
  EXPECT_EQ(rhoban_utils::deg2rad(90.0), M_PI / 2);
  EXPECT_EQ(rhoban_utils::deg2rad(180.0), M_PI);
  EXPECT_EQ(rhoban_utils::deg2rad(270.0), 3 * M_PI / 2);
}

/*******************************************************
 * rad2deg test
 */

TEST(rad2deg, testSuccess)
{
  EXPECT_EQ(rhoban_utils::rad2deg(0.0), 0.0);
  EXPECT_EQ(rhoban_utils::rad2deg(M_PI / 4), 45.0);
  EXPECT_EQ(rhoban_utils::rad2deg(M_PI / 2), 90.0);
  EXPECT_EQ(rhoban_utils::rad2deg(M_PI), 180.0);
  EXPECT_EQ(rhoban_utils::rad2deg(3 * M_PI / 2), 270.0);
}

/*******************************************************
 * normalizeRad test
 */

TEST(normalizeRad, testSuccess)
{
  EXPECT_EQ(rhoban_utils::normalizeRad(0.0), 0.0);
  EXPECT_EQ(rhoban_utils::normalizeRad(M_PI / 4), M_PI / 4);
  EXPECT_EQ(rhoban_utils::normalizeRad(M_PI), M_PI);
  EXPECT_EQ(rhoban_utils::normalizeRad(3 * M_PI / 2), -1 * M_PI / 2);
  EXPECT_EQ(rhoban_utils::normalizeRad(-1 * M_PI / 2), -1 * M_PI / 2);
}

/*******************************************************
 * getJsonVal test
 */

TEST(getJsonVal, testSuccess)
{
  Json::Value json(45.0);
  rhoban_utils::Angle res = rhoban_utils::getJsonVal<rhoban_utils::Angle>(json);
  EXPECT_EQ(res.getValue(), 45.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
