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

#include <polaris/types/task/state_machine.hpp>

#include <string>
#include <vector>

namespace polaris
{
namespace types
{
/**
 * @brief Construct a new State Machine:: State Machine object
 *
 * @param xml_string XML string for the RostateMachine Definition
 */
StateMachine::StateMachine(std::string xml_string)
{
  boost::property_tree::ptree pt;
  std::stringstream ss;
  ss << xml_string;
  boost::property_tree::read_xml(ss, pt);
  std::string init_state_name;
  for (const boost::property_tree::ptree::value_type & state_itr : pt.get_child("state_machine")) {
    if (state_itr.first == "init_state") {
      init_state_name = state_itr.second.get<std::string>("<xmlattr>.name");
    }
    if (state_itr.first == "state_machine_name") {
      name_ = state_itr.second.get<std::string>("<xmlattr>.name");
    }
  }
  for (const boost::property_tree::ptree::value_type & state_itr : pt.get_child("state_machine")) {
    if (state_itr.first == "transition") {
      std::string from_state_name = state_itr.second.get<std::string>("<xmlattr>.from");
      std::string to_state_name = state_itr.second.get<std::string>("<xmlattr>.to");
      std::string trigger_event_name = state_itr.second.get<std::string>("<xmlattr>.name");
      addTransition(from_state_name, to_state_name, trigger_event_name);
    }
  }
  setCurrentState(init_state_name);
}

/**
 * @brief Destroy the State Machine:: State Machine object
 *
 */
StateMachine::~StateMachine()
{
}

/**
 * @brief get name of the state machine
 *
 * @return std::string name of the state machine
 */
std::string StateMachine::getName()
{
  return name_;
}

/**
 * @brief Function for setting Current State Infomation
 *
 * @param current_state target state
 * @return true Succeed to set state
 * @return false Failed to set state
 */
bool StateMachine::setCurrentState(std::string current_state)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto vertex_range = boost::vertices(state_graph_);
  for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first) {
    vertex_t v = *first;
    if (state_graph_[v].name == current_state) {
      current_state_ = v;
      return true;
    }
  }
  return false;
}

/**
 * @brief add Transition function for the State Machine
 *
 * @param from_state_name state transition from
 * @param to_state_name state transition to
 * @param trigger_event_name trigger event
 */
void StateMachine::addTransition(
  std::string from_state_name, std::string to_state_name,
  std::string trigger_event_name)
{
  std::lock_guard<std::mutex> lock(mtx_);
  vertex_t from_state;
  vertex_t to_state;
  edge_t transition;
  auto vertex_range = boost::vertices(state_graph_);
  if (from_state_name != to_state_name) {
    bool from_state_found = false;
    bool to_state_found = false;
    for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first) {
      vertex_t v = *first;
      if (state_graph_[v].name == from_state_name) {
        from_state_found = true;
        from_state = v;
      }
      if (state_graph_[v].name == to_state_name) {
        to_state_found = true;
        to_state = v;
      }
    }
    if (!from_state_found) {
      vertex_t v = boost::add_vertex(state_graph_);
      state_graph_[v].name = from_state_name;
      from_state = v;
    }
    if (!to_state_found) {
      vertex_t v = boost::add_vertex(state_graph_);
      state_graph_[v].name = to_state_name;
      to_state = v;
    }
    bool inserted = false;
    boost::tie(transition, inserted) = boost::add_edge(from_state, to_state, state_graph_);
    state_graph_[transition].trigger_event = trigger_event_name;
    state_graph_[transition].from_state = from_state_name;
    state_graph_[transition].to_state = to_state_name;
  } else {
    bool state_found = false;
    for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first) {
      vertex_t v = *first;
      if (state_graph_[v].name == from_state_name) {
        state_found = true;
        from_state = v;
        to_state = v;
      }
    }
    if (!state_found) {
      vertex_t v = boost::add_vertex(state_graph_);
      state_graph_[v].name = from_state_name;
      from_state = v;
      to_state = v;
    }
    bool inserted = false;
    boost::tie(transition, inserted) = boost::add_edge(from_state, to_state, state_graph_);
    state_graph_[transition].trigger_event = trigger_event_name;
    state_graph_[transition].from_state = from_state_name;
    state_graph_[transition].to_state = to_state_name;
  }
}

/**
 * @brief Function for getting possible transition states
 *
 * @return std::vector<std::string> possible transition states
 */
std::vector<std::string> StateMachine::getPossibeTransitionStates()
{
  std::lock_guard<std::mutex> lock(mtx_);
  std::vector<std::string> ret;
  adjacency_iterator_t vi;
  adjacency_iterator_t vi_end;
  for (boost::tie(vi, vi_end) = adjacent_vertices(current_state_, state_graph_); vi != vi_end;
    ++vi)
  {
    ret.push_back(state_graph_[*vi].name);
  }
  return ret;
}

/**
 * @brief Function for getting possible transition trigger event
 *
 * @return std::vector<std::string> get possible trigger event
 */
std::vector<std::string> StateMachine::getPossibeTransitions()
{
  std::lock_guard<std::mutex> lock(mtx_);
  std::vector<std::string> ret;
  out_edge_iterator_t ei;
  out_edge_iterator_t ei_end;
  for (boost::tie(ei, ei_end) = out_edges(current_state_, state_graph_); ei != ei_end; ++ei) {
    ret.push_back(state_graph_[*ei].trigger_event);
  }
  return ret;
}

/**
 * @brief Try transition from trigger event
 *
 * @param trigger_event_name trigger event name
 * @return true Succeed to transition
 * @return false Failed to transition
 */
bool StateMachine::tryTransition(std::string trigger_event_name)
{
  std::lock_guard<std::mutex> lock(mtx_);
  out_edge_iterator_t ei;
  out_edge_iterator_t ei_end;
  for (boost::tie(ei, ei_end) = out_edges(current_state_, state_graph_); ei != ei_end; ++ei) {
    if (trigger_event_name == state_graph_[*ei].trigger_event) {
      auto vertex_range = boost::vertices(state_graph_);
      for (auto first = vertex_range.first, last = vertex_range.second; first != last; ++first) {
        vertex_t v = *first;
        if (state_graph_[v].name == state_graph_[*ei].to_state) {
          current_state_ = v;
          return true;
        }
      }
      return false;
    }
  }
  return false;
}

/**
 * @brief Function getting current state info
 *
 * @return StateInfo current state info
 */
StateInfo StateMachine::getStateInfo()
{
  std::lock_guard<std::mutex> lock(mtx_);
  std::string current_state = state_graph_[current_state_].name;
  std::vector<std::string> possible_transitions;
  out_edge_iterator_t ei;
  out_edge_iterator_t ei_end;
  for (boost::tie(ei, ei_end) = out_edges(current_state_, state_graph_); ei != ei_end; ++ei) {
    possible_transitions.push_back(state_graph_[*ei].trigger_event);
  }
  std::vector<std::string> possible_transition_states;
  adjacency_iterator_t vi;
  adjacency_iterator_t vi_end;
  for (boost::tie(vi, vi_end) = adjacent_vertices(current_state_, state_graph_); vi != vi_end;
    ++vi)
  {
    possible_transition_states.push_back(state_graph_[*vi].name);
  }
  StateInfo ret(current_state, possible_transition_states, possible_transitions);
  return ret;
}

/**
 * @brief Function getting current state name
 *
 * @return std::string current state name
 */
std::string StateMachine::getCurrentState()
{
  return state_graph_[current_state_].name;
}
}  // namespace types
}  // namespace polaris
