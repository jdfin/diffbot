// Copyright (c) 2021 PickNik, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Denis Stogl

#include <gmock/gmock.h>

#include <string>
using std::string;

#include "hardware_interface/resource_manager.hpp"
using hardware_interface::ResourceManager;

#include "rclcpp/node.hpp"
using rclcpp::Node;


class TestRosBotSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rosbot_system = R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="MinimalRobot">

  <link name="world"/>

  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>

  <link name="base_link"/>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>

  <link name="link1"/>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="0.1" lower="-3.14159265359" upper="3.14159265359" velocity="0.2"/>
  </joint>

  <link name="link2"/>

  <ros2_control name="MockRosBotSystem" type="system">

    <hardware>
      <plugin>rosbot/RosBotSystem</plugin>
    </hardware>

    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
    </joint>

    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>

  </ros2_control>

</robot>
)";
  }
  string rosbot_system;
  Node node_ = Node("TestRosBotSystem");
};


class TestableResourceManager : public ResourceManager
{
public:
  explicit TestableResourceManager(Node& node, const string& urdf,
                                   bool activate_all=false,
                                   unsigned int cm_update_rate=100) :
      ResourceManager(urdf, node.get_node_clock_interface(),
                      node.get_node_logging_interface(), activate_all,
                      cm_update_rate)
  {
  }
};


TEST_F(TestRosBotSystem, load_rosbot_system)
{
  ASSERT_NO_THROW(TestableResourceManager rm(node_, rosbot_system));
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
