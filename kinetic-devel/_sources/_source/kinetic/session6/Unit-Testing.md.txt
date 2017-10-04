# Unit Testing
>In this exercise we will write a unit tests in the myworkcell_core package.

## Motivation

The ROS Scan-N-Plan application is complete and documented.  Now we want to test the program to make sure it behaves as expected. 

## Information and Resources
[Google Test](https://github.com/google/googletest): C++ XUnit test framework

[rostest](http://wiki.ros.org/rostest): ROS wrapper for XUnit test framework

[catkin testing](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html?highlight=run_tests#building-and-running-tests): Building and running tests with catkin

## Scan-N-Plan Application: Problem Statement
We have completed and and documented our Scan-N-Plan program.  We need to create a test framework so we can be sure our program runs as intended after it is built. In addition to ensuring the code runs as intended, unit tests allow you to easily check if new code changes functionality in unexpected ways.  Your goal is to create the unit test frame work and write a few tests. 

## Scan-N-Plan Application: Guidance

### Create the unit test frame work

1. Create a `test` folder in the myworkcell_core/src folder. In the workspace directory:

```
catkin build
source devel/setup.bash
roscd myworkcell_core
mkdir src/test
```

2. Create utest.cpp file in the `myworkcell_core/src/test` folder:

```
touch src/test/utest.cpp
```

3. Open utest.cpp in QT and include ros & gtest:

``` c++
#include <ros/ros.h>
#include <gtest/gtest.h>
```

4. Write a dummy test that will return true if executed. This will test our framework and we will replace it later with more useful tests:

``` c++
TEST(TestSuite, myworkcell_core_framework){
  ASSERT(true);
}
```

5. Next include the general main function, which will execute the unit tests we write later:

``` c++
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

6. Edit myworkcell_core CMakeLists.txt to build the u_test.cpp file.  Append CMakeLists.txt:

``` cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(utest_node test/utest_launch.test src/test/utest.cpp)
  target_link_libraries(utest_node ${catkin_LIBRARIES})
endif()
```

7. Create a test folder under myworkcell_core

```
mkdir test
```

8. Create a test launch file:

```
touch test/utest_launch.test
```

9. Open the utest_launch.test file in QT and populate the file:

``` xml
<?xml version="1.0"?>
<launch>
  <test test-name="unit_test_node" pkg="myworkcell_core" type="utest_node"/>
</launch>
```

10. Test the framework

```
catkin build
catkin run_tests
```
The console output should show:
```
[ROSTEST]-----------------------------------------------------------------------

[myworkcell_core.rosunit-unit_test_node/myworkcell_core_framework][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0
```
This means our framework is functional and now we can add usefull unit tests.

### Write the unit tests
 1. Since we will be testing the messages we get from the fake_ar_publisher package, include the relevant header file:
``` c++
#include <fake_ar_publisher/ARMarker.h>
```

 2. Declare a global variable:
``` c++
fake_ar_publisher::ARMarkerConstPtr test_msg_;
```

 3. Write a unit test to check the reference frame of the ar_pose_marker:
``` c++

```
 4. Run the test:
```
catkin build
rostest myworkcell_core utest_launch.test
```
 5. view the results of the test:
```
catkin_test_results build/myworkcell_core
```
You should see:
Summary: 2 tests, 0 errors, 0 failures
