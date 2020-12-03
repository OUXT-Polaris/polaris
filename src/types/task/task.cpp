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

#include <polaris/exception.hpp>
#include <polaris/types/task/task.hpp>

#include <vector>
#include <memory>
#include <string>

namespace polaris
{
namespace types
{
Task::Task() {}

Task::Task(
  std::vector<Task> depends,
  std::vector<Entity> entities,
  double time,
  double reward)
{
  depends_ = depends;
  time_ = time;
  reward_ = reward;
  std::string description = task_state_description;
  state_machine_ptr_ = std::make_shared<StateMachine>(description);
  entities_ = entities;
}

void Task::addDepends(Task task)
{
  depends_.emplace_back(task);
}

double Task::getSpendTime() const
{
  double spend_time = time_;
  for (const auto depend : depends_) {
    spend_time = spend_time + depend.getSpendTime();
  }
  return spend_time;
}

TaskState Task::getState() const
{
  const auto current_state = state_machine_ptr_->getCurrentState();
  if (current_state == "initialized") {
    return TaskState::INITIALIZED;
  }
  if (current_state == "running") {
    return TaskState::RUNNING;
  }
  if (current_state == "succeed") {
    return TaskState::SUCCEED;
  }
  if (current_state == "failed") {
    return TaskState::FAILED;
  }
  if (current_state == "yeild") {
    return TaskState::YEILD;
  }
  std::string message = "task state " + current_state + " is invalid.";
  throw StateMachineRuntimeError(message);
}
}  // namespace types
}  // namespace polaris
