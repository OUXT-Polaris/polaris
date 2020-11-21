// Copyright (c) 2020, OUXT-Polaris
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

#include <polaris/types/task/task.hpp>

#include <vector>

namespace polaris
{
namespace types
{
Task::Task() {}

Task::Task(std::vector<Task> depends, double time, double reward)
{
  depends_ = depends;
  time_ = time;
  reward_ = reward;
  std::string description = task_state_description;
  state_machine_ptr_ = std::make_shared<StateMachine>(description);
}
}  // namespace types
}  // namespace polaris
