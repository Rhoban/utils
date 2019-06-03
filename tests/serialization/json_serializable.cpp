#include <gtest/gtest.h>
#include <rhoban_utils/serialization/json_serializable.h>
#include <json/json.h>
#include "rhoban_utils/io_tools.h"
#include "rhoban_utils/util.h"
using namespace std;
using namespace rhoban_utils;

const string testFilePath = "ressources/file.json";
const string fileContentShould = "{\"key1\" : \"valeur1\",\"key2\" : \"valeur2\"}";

string getAbsoluteTestFilePath()
{
  string filePath = __FILE__;
  string currentDirPath = filePath.substr(0, filePath.rfind("/"));
  return currentDirPath + "/" + testFilePath;
}

// TODO: solve issue with docker file system
TEST(file2Json, DISABLED_TestSuccess)
{
  string absoluteTestFilePath = getAbsoluteTestFilePath();
  Json::Value json_content;
  Json::Value result;
  json_content["key1"] = "valeur1";
  json_content["key2"] = "valeur2";
  ASSERT_EQ(getAbsoluteTestFilePath(), "HGDECHD");
  try
  {
    result = rhoban_utils::file2Json(absoluteTestFilePath);
  }
  catch (string const& exception)
  {
    // should not pass here
    ASSERT_TRUE(false);
  }

  ASSERT_EQ(result["key1"], json_content["key1"]);
  ASSERT_EQ(result["key2"], json_content["key2"]);
}

class testFactoryJson : public JsonSerializable
{
public:
  int int_value = 0;
  double double_value = 0.0;
  testFactoryJson(){};
  Json::Value toJson() const
  {
    Json::Value v("toto");
    return v;
  }
  std::string getClassName() const
  {
    return "testFactoryJson";
  }
  void fromJson(const Json::Value& json_value, const std::string& dir_name)
  {
    (void)dir_name;
    (void)json_value;
    int_value = 12;
    double_value = 12.5;
  }
};

TEST(testFactoryJson, testSuccessFactory)
{
  JsonSerializable* object = new testFactoryJson();
  Json::Value resultat = object->toFactoryJson();
  Json::StreamWriterBuilder builder;
  builder.settings_["indentation"] = "";
  std::string res = Json::writeString(builder, resultat["class name"]);
  std::cout << res;
  // XXX: how to avoid the inner quotes?
  EXPECT_EQ(std::string("\"testFactoryJson\""), res);

  Json::StreamWriterBuilder builder2;
  builder2.settings_["indentation"] = "";
  std::string res2 = Json::writeString(builder2, resultat["content"]);
  // XXX: how to avoid the inner quotes?
  EXPECT_EQ(std::string("\"toto\""), res2);
}

TEST(read, TestSuccess)
{
  testFactoryJson* object = new testFactoryJson();

  Json::Value v;
  v["key1"] = "valeur1";
  v["key2"] = "valeur2";
  std::string dir_name;
  object->read(v, "key1", dir_name);
  EXPECT_EQ(object->int_value, 12);
  EXPECT_EQ(object->double_value, 12.5);
}

TEST(checkMember, testException)
{
  Json::Value v;
  v["key2"] = "valeur2";
  v["key3"] = "valeur3";
  std::string key = "key1";

  try
  {
    checkMember(v, key);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Could not find member '" + key + "'"));
  }
}

TEST(checkMember, testSuccess)
{
  EXPECT_NO_THROW({
    Json::Value v;
    v["key2"] = "valeur2";
    v["key3"] = "valeur3";
    std::string key = "key2";
    checkMember(v, key);
  });
}

TEST(getJsonVal, TestBoolSuccess)
{
  Json::Value v;
  v["key1"] = true;
  EXPECT_TRUE(getJsonVal<bool>(v["key1"]));
}

TEST(getJsonVal, TestBoolExeption)
{
  Json::Value v;
  v["key1"] = "vleur1";

  try
  {
    getJsonVal<bool>(v["key1"]);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Expecting a bool"));
  }
}

TEST(getJsonVal, TestIntSuccess)
{
  Json::Value v;
  v["key1"] = 2;
  EXPECT_EQ(2, getJsonVal<int>(v["key1"]));
}

TEST(getJsonVal, TestIntExeption)
{
  Json::Value v;
  v["key1"] = "vleur1";

  try
  {
    getJsonVal<int>(v["key1"]);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Expecting an int"));
  }
}

TEST(getJsonVal, TestFloatSuccess)
{
  Json::Value v;
  v["key1"] = 1.2;
  EXPECT_FLOAT_EQ(1.2, getJsonVal<float>(v["key1"]));
}

TEST(getJsonVal, TestFloatExeption)
{
  Json::Value v;
  v["key1"] = "vleur1";

  try
  {
    getJsonVal<float>(v["key1"]);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Expecting a float"));
  }
}

TEST(getJsonVal, TestDoubleSuccess)
{
  Json::Value v;
  v["key1"] = 1.2656563;
  EXPECT_DOUBLE_EQ(1.2656563, getJsonVal<double>(v["key1"]));
}

TEST(getJsonVal, TestDoubleExeption)
{
  Json::Value v;
  v["key1"] = "vleur1";

  try
  {
    getJsonVal<double>(v["key1"]);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Expecting a double"));
  }
}

TEST(getJsonVal, TestStringSuccess)
{
  Json::Value v;
  std::string valeur = "valeur1";
  v["key1"] = valeur;
  EXPECT_EQ(valeur, getJsonVal<std::string>(v["key1"]));
}

TEST(getJsonVal, TestStringExeption)
{
  Json::Value v;
  v["key1"] = 156;

  try
  {
    getJsonVal<std::string>(v["key1"]);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Expecting a string"));
  }
}

TEST(getJsonVal, TestQuaterniondSuccess)
{
  Json::Value v;
  v[0] = 1;
  v[1] = 0;
  v[2] = 0;
  v[3] = 0;
  Eigen::Quaterniond result;
  result = getJsonVal<Eigen::Quaterniond>(v);

  Eigen::Quaterniond expected(1, 0, 0, 0);

  EXPECT_EQ(expected.w(), result.w());
  EXPECT_EQ(expected.x(), result.x());
  EXPECT_EQ(expected.y(), result.y());
  EXPECT_EQ(expected.z(), result.z());
}

TEST(getJsonVal, TestQuaterniondArrayException)
{
  Json::Value v;
  v["key1"] = "value1";

  try
  {
    getJsonVal<Eigen::Quaterniond>(v["key1"]);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Expecting an array."));
  }
}

TEST(getJsonVal, TestQuaterniondArraySizeException)
{
  Json::Value v;
  v[0] = 1;
  v[1] = 0;
  v[2] = 0;

  try
  {
    getJsonVal<Eigen::Quaterniond>(v);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("Expecting an array of size 4."));
  }
}

TEST(getJsonVal, TestVector2dSuccess)
{
  Json::Value v;
  v[0] = 15;
  v[1] = 3;
  Eigen::Vector2d expected(2);
  expected(0) = 15;
  expected(1) = 3;
  Eigen::Vector2d result;

  result = json2eigen<2, 1>(v);

  EXPECT_EQ(expected(0), result(0));
  EXPECT_EQ(expected(1), result(1));
}

TEST(getJsonVal, TestVector2dExceptionArray)
{
  Json::Value v;
  v["key1"] = 1;
  v["key2"] = 2;
  Eigen::Vector2d expected(2);
  expected(0) = 1;
  expected(1) = 2;
  Eigen::Vector2d result;
  try
  {
    result = json2eigen<2, 1>(v);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("json2eigen<Eigen::Matrix<2,1>>: Expecting an array"));
  }
}

TEST(getJsonVal, TestVector2dException2Values)
{
  Json::Value v;
  v[0] = 1;
  v[1] = 2;
  v[3] = 3;
  Eigen::Vector2d result;
  try
  {
    result = json2eigen<2, 1>(v);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("json2eigen<Eigen::Matrix<2,1>>: 4 rows received"));
  }
}

TEST(getJsonVal, TestVectorXdSuccess)
{
  Json::Value v;
  v["rows"] = 2;
  v["values"][0] = 1;
  v["values"][1] = 2;
  Eigen::VectorXd result;
  result = json2eigen<-1, 1>(v);

  Eigen::VectorXd expected(2);
  expected(0) = 1;
  expected(1) = 2;

  EXPECT_EQ(expected(0), result(0));
  EXPECT_EQ(expected(1), result(1));
}

// Test with an element empty in the Json::Value
TEST(getJsonVal, TestVectorXdExceptionObject)
{
  Json::Value v;
  v[0] = 1;
  v[2] = 2;
  Eigen::VectorXd result;
  try
  {
    result = json2eigen<-1, 1>(v);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
  }
}

TEST(getJsonVal, TestVectorXdExceptionRows)
{
  Json::Value v;
  v["rows"] = 3;
  v["values"][0] = 1;
  v["values"][1] = 2;
  Eigen::VectorXd result;
  try
  {
    result = json2eigen<-1, 1>(v);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("getJsonVal<Eigen::VectorXd>: Inconsistency in rows numbers"));
  }
}

// TODO: solve issue of inconsistency between the way of writing and reading a json::value
TEST(GetJsonVal_vector2Json, DISABLED_TestSuccess)
{
  Eigen::Vector2d Input(2);
  Input(0) = 1.0;
  Input(1) = 2.0;
  Json::Value v1;
  Json::Value v2;
  v1[0] = 1;
  v1[1] = 2;
  Eigen::Vector2d Output(2);

  Output = json2eigen<2, 1>(v1);
  v2 = vector2Json(Output);

  EXPECT_EQ(v1[0], v2[0]);
  EXPECT_EQ(v1[1], v2[1]);
}

TEST(vector2Json, TestSuccess)
{
  Eigen::VectorXd Input(2);
  Input(0) = 1.0;
  Input(1) = 2.0;
  Json::Value v1;
  v1["values"][0] = 1.0;
  v1["values"][1] = 2.0;
  v1["rows"] = 2;
  Json::Value v2;

  v2 = vector2Json(Input);
  EXPECT_EQ(v1["rows"], v2["rows"]);
  EXPECT_EQ(v1["values"][0], v2["values"][0]);
  EXPECT_EQ(v1["values"][1], v2["values"][1]);
}

TEST(vector2Json, TestFailed)
{
  Eigen::VectorXd Input(2);
  Input(0) = 5.0;
  Input(1) = 2.0;
  Json::Value v1;
  v1["values"][0] = 5.0;
  v1["values"][1] = 3.0;
  v1["rows"] = 2;
  Json::Value v2;

  v2 = vector2Json(Input);
  EXPECT_FALSE(v2["values"][0] == v1["values"][0] && v2["values"][1] == v1["values"][1] && v2["rows"] == v1["rows"]);
}
// TODO: solve issue with function getJsonVal<Eigen::Vector>().
TEST(vector2Json_GetJsonVal, DISABLED_TestSuccess)
{
  Eigen::Vector2d Input(2);
  Input(0) = 1.0;
  Input(1) = 2.0;
  Json::Value v2;
  Eigen::Vector2d Output(2);

  v2 = vector2Json(Input);
  Output = json2eigen<2, 1>(v2);

  EXPECT_EQ(Input.size(), Output.size());
  EXPECT_EQ(Input(0), Output(0));
  EXPECT_EQ(Input(1), Output(1));
}

TEST(matrix2json, TestSuccess)
{
  Eigen::MatrixXd Input(2, 2);
  Input(0, 0) = 3.0;
  Input(1, 0) = 2.5;
  Input(0, 1) = -1.0;
  Input(1, 1) = 5.0;

  Json::Value m1;
  m1["values"][0][0] = 3.0;
  m1["values"][1][0] = 2.5;
  m1["values"][0][1] = -1.0;
  m1["values"][1][1] = 5.0;
  Json::Value m2;

  m2 = matrix2Json(Input);
  EXPECT_EQ(m2["rows"], 2);
  EXPECT_EQ(m2["cols"], 2);
  EXPECT_EQ(m2["values"][0][0], m1["values"][0][0]);
  EXPECT_EQ(m2["values"][1][0], m1["values"][1][0]);
  EXPECT_EQ(m2["values"][0][1], m1["values"][0][1]);
  EXPECT_EQ(m2["values"][1][1], m1["values"][1][1]);
}

TEST(matrix2json, TestFailed)
{
  Eigen::MatrixXd Input(2, 2);
  Input(0, 0) = 3.0;
  Input(1, 0) = 2.5;
  Input(0, 1) = -1.0;
  Input(1, 1) = 5.0;

  Json::Value m1;
  m1["values"][0][0] = 3.0;
  m1["values"][1][0] = 2.5;
  m1["values"][0][1] = -13.0;
  m1["values"][1][1] = 5.0;
  Json::Value m2;

  m2 = matrix2Json(Input);
  bool b = true;
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      if (m2["values"][i][j] != m1["values"][i][j])
      {
        b = false;
        break;
      }
    }
  }
  EXPECT_FALSE(b && m2["rows"] == 2 && m2["cols"] == 2);
}

TEST(matrix2json_GetJsonVal, TestSuccess)
{
  Eigen::MatrixXd Input(2, 2);
  Input(0, 0) = 3.0;
  Input(1, 0) = 2.5;
  Input(0, 1) = -1.0;
  Input(1, 1) = 5.0;

  Json::Value m2;
  Eigen::MatrixXd Output(2, 2);

  m2 = matrix2Json(Input);
  Output = json2eigen<-1, -1>(m2);
  EXPECT_EQ(Input(0, 0), Output(0, 0));
  EXPECT_EQ(Input(1, 0), Output(1, 0));
  EXPECT_EQ(Input(0, 1), Output(0, 1));
  EXPECT_EQ(Input(1, 1), Output(1, 1));
  EXPECT_EQ(Input.size(), Output.size());
}

TEST(getJsonVal, TestMatrixXdSuccess)
{
  Json::Value v1;
  v1["rows"] = 3;
  v1["cols"] = 3;
  v1["values"][0][0] = 1;
  v1["values"][0][1] = 2;
  v1["values"][0][2] = 3;
  v1["values"][1][0] = 4;
  v1["values"][1][1] = 5;
  v1["values"][1][2] = 6;
  v1["values"][2][0] = 7;
  v1["values"][2][1] = 8;
  v1["values"][2][2] = 3;

  Eigen::MatrixXd m(3, 3);
  m(0, 0) = 1;
  m(0, 1) = 2;
  m(0, 2) = 3;
  m(1, 0) = 4;
  m(1, 1) = 5;
  m(1, 2) = 6;
  m(2, 0) = 7;
  m(2, 1) = 8;
  m(2, 2) = 3;

  Eigen::MatrixXd result(3, 3);
  result = json2eigen<-1, -1>(v1);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      EXPECT_EQ(m(i, j), result(i, j));
    }
  }
}

TEST(readVector, TestSuccess)
{
  Json::Value v;
  v["values"][0] = 5;
  v["values"][1] = 3;
  const Json::Value va = v;
  const std::string key = "values";
  std::vector<int> expected;
  expected.push_back(5);
  expected.push_back(3);
  std::vector<int> result;
  result = readVector<int>(va, key);

  EXPECT_EQ(expected[0], result[0]);
  EXPECT_EQ(expected[1], result[1]);
  EXPECT_EQ(expected.size(), result.size());
}

TEST(getJsonVal, MatrixXdExceptionObject)
{
  Json::Value v1;
  v1[0][0] = 1;
  v1[0][1] = 2;
  v1[0][2] = 3;
  v1[1][0] = 4;
  v1[1][1] = 5;
  v1[1][2] = 6;
  v1[2][0] = 7;
  v1[2][1] = 8;
  v1[2][2] = 3;

  Eigen::MatrixXd result(3, 3);
  try
  {
    result = json2eigen<-1, -1>(v1);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("getJsonVal<Eigen::MatrixXd>: Expecting an object"));
  }
}

TEST(getJsonVal, MatrixXdExceptionArrayValues)
{
  Json::Value v1;
  v1["rows"] = 3;
  v1["cols"] = 3;
  v1["values"] = 1;

  Eigen::MatrixXd result(3, 3);
  try
  {
    result = json2eigen<-1, -1>(v1);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("getJsonVal<Eigen::MatrixXd>: Expecting an array for 'values'"));
  }
}

TEST(getJsonVal, TestMatrixXdExceptionRows)
{
  Json::Value v1;
  v1["rows"] = 4;
  v1["cols"] = 3;
  v1["values"][0][0] = 1;
  v1["values"][0][1] = 2;
  v1["values"][0][2] = 3;
  v1["values"][1][0] = 4;
  v1["values"][1][1] = 5;
  v1["values"][1][2] = 6;
  v1["values"][2][0] = 7;
  v1["values"][2][1] = 8;
  v1["values"][2][2] = 3;

  Eigen::MatrixXd result(3, 3);
  try
  {
    result = json2eigen<-1, -1>(v1);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("getJsonVal<Eigen::MatrixXd>: Inconsistency in rows numbers"));
  }
}

TEST(getJsonVal, TestMatrixXdExceptionArrayRows)
{
  Json::Value v1;
  v1["rows"] = 3;
  v1["cols"] = 3;
  v1["values"][0] = 1;
  v1["values"][1] = 2;
  v1["values"][2] = 3;

  Eigen::MatrixXd result(3, 3);
  try
  {
    result = json2eigen<-1, -1>(v1);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("getJsonVal<Eigen::MatrixXd>: Expecting an array for row"));
  }
}

TEST(getJsonVal, TestMatrixXdExceptionCols)
{
  Json::Value v1;
  v1["rows"] = 3;
  v1["cols"] = 4;
  v1["values"][0][0] = 1;
  v1["values"][0][1] = 2;
  v1["values"][0][2] = 3;
  v1["values"][1][0] = 4;
  v1["values"][1][1] = 5;
  v1["values"][1][2] = 6;
  v1["values"][2][0] = 7;
  v1["values"][2][1] = 8;
  v1["values"][2][2] = 3;

  Eigen::MatrixXd result(3, 3);
  try
  {
    result = json2eigen<-1, -1>(v1);
    FAIL();
  }
  catch (JsonParsingError& err)
  {
    EXPECT_EQ(err.what(), std::string("getJsonVal<Eigen::MatrixXd>: Inconsistency for cols"));
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
