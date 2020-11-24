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

#ifndef POLARIS__TYPES__TASK__TASK_HPP_
#define POLARIS__TYPES__TASK__TASK_HPP_

#include <polaris/types/task/state_machine.hpp>
#include <polaris/types/entity.hpp>

#include <vector>
#include <string>
#include <memory>

namespace polaris
{
namespace types
{
const char task_state_description[] =
  R"(
<state_machine>
  <init_state name="initialize"/>
  <state_machine_name name="task_state_machine"/>

  <transition from = "initialize" to="running" name="start"/>
  <transition from = "running" to="succeed" name="success"/>
  <transition from = "running" to="failed" name="failure"/>
  <transition from = "running" to="yield" name="yield"/>
  <transition from = "initialize" to="yield" name="yield"/>
  <transition from = "yield" to="initialize" name="initialized"/>
</state_machine>
)";

class Task
{
public:
  Task();
  explicit Task(
    std::vector<Task> depends,
    std::vector<Entity> entities,
    double time,
    double reward);

private:
  std::vector<Task> depends_;
  double time_;
  double reward_;
  std::shared_ptr<StateMachine> state_machine_ptr_;
  std::vector<Entity> entities_;
};
}  // namespace types
}  // namespace polaris

#endif  // POLARIS__TYPES__TASK__TASK_HPP_
