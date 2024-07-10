// Copyright 2024 Gentlebots
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

#ifndef NAVIGATION__UTILS_HPP_
#define NAVIGATION__UTILS_HPP_

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace navigation
{

std::string nav_to_pose_truncated_xml =
  R"(<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <Sequence>
            <RecoveryNode number_of_retries="1" name="ComputePathToPose">
              <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </RecoveryNode>
            <TruncatePath distance="0.000000" input_path="{path}" output_path="{path}"/>
          </Sequence>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>)";

std::string dynamic_following_xml =
  R"(<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <Sequence>
          <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
              <Sequence>
                <ComputePathToPose goal="{updated_goal}" path="{path}" planner_id="GridBased"/>
              </Sequence>    
          </GoalUpdater>
         <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
        </Sequence>
      </RateController>
      <KeepRunningUntilFailure>
        <Sequence>
          <FollowPath path="{truncated_path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
        </Sequence>
      </KeepRunningUntilFailure>
    </PipelineSequence>
  </BehaviorTree>
</root>)";

std::string modify_truncate_distance(std::string xml, double distance)
{
  std::string distance_str = std::to_string(distance);
  std::string to_find = "distance=\"";
  std::size_t found = xml.find(to_find);
  if (found != std::string::npos) {
    std::size_t start = found + to_find.size();
    std::size_t end = xml.find("\"", start);
    xml.replace(start, end - start, distance_str);
  }
  return xml;
}

// generate a temp file with the given content, as input the distance to truncate the path
inline std::string generate_xml_file(std::string content, double distance)
{
  std::string temp_file = "/tmp/bt_xml_XXXXXX";
  char * temp_file_c = new char[temp_file.length() + 1];
  strcpy(temp_file_c, temp_file.c_str());
  int fd = mkstemp(temp_file_c);
  if (fd == -1) {
    std::cerr << "Error creating temp file" << std::endl;
    return "";
  }
  std::ofstream file(temp_file_c);
  file << modify_truncate_distance(content, distance);
  file.close();
  return std::string(temp_file_c);
}

}  // namespace navigation

#endif  // NAVIGATION__UTILS_HPP_
