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
  <init_state name="initialized"/>
  <state_machine_name name="task_state_machine"/>

  <transition from = "initialized" to="running" name="start"/>
  <transition from = "running" to="succeed" name="success"/>
  <transition from = "running" to="failed" name="failure"/>
  <transition from = "running" to="yield" name="yield"/>
  <transition from = "initialized" to="yield" name="yield"/>
  <transition from = "yield" to="initialized" name="initialize"/>
</state_machine>
)";

enum class TaskState
{
  INITIALIZED,
  RUNNING,
  SUCCEED,
  FAILED,
  YEILD
};

/**
 * @brief Task variable class
 */
class Task
{
public:
  Task();
  /**
   * @brief Construct a new Task object
   * @param depends the list of depended tasks
   * @param entities target entities of the task
   * @param time estimated elapsed time to process this task
   * @param reward reward of the task
   */
  explicit Task(
    std::vector<Task> depends,
    std::vector<Entity> entities,
    double time,
    double reward);
  /**
   * @brief add dependency
   * @param task the task depends on this task
   */
  void addDepends(Task task);
  /**
   * @brief Get current state of the task
   * @return TaskState
   */
  TaskState getState() const;
  double getSpendTime() const;
  double getReward() const;

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
