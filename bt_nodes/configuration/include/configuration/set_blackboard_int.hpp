/*  Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef CONFIGURATION__SET_BLACKBOARD_INT_H
#define CONFIGURATION__SET_BLACKBOARD_INT_H

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace configuration
{
class SetBlackboardInt : public BT::SyncActionNode
{
public:
  SetBlackboardInt(const std::string& name, const BT::NodeConfiguration& config) :
    SyncActionNode(name, config)
  {
    setRegistrationID("SetBlackboard");
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort("value", "Value to be written int othe output_key"),
            BT::InputPort("output_key", "Key where the value will be written")};
  }

private:
  virtual BT::NodeStatus tick() override
  {
    std::string output_key;
    getInput("output_key", output_key);

    int value;
    getInput("value", value);
    
    config().blackboard->set(output_key, value);

    return BT::NodeStatus::SUCCESS;
  }
};
}   // namespace BT

#endif