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

#ifndef POLARIS__TYPES__TASK__STATE_MACHINE_HPP_
#define POLARIS__TYPES__TASK__STATE_MACHINE_HPP_

// headers in boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/graph/graphviz.hpp>

// headers in STL
#include <mutex>
#include <string>
#include <vector>

/**
 * @brief State Transition Property for State Machine Class
 *
 */
struct TransitionProperty
{
  std::string trigger_event;
  std::string from_state;
  std::string to_state;
};

/**
 * @brief State Property for State Machine Class
 *
 */
struct StateProperty
{
  std::string name;
};

/**
 * @brief Boost::Graph type for State Machine Class
 *
 */
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, StateProperty,
    TransitionProperty> graph_t;
/**
 * @brief vertex type for State Machine Class (State)
 *
 */
typedef graph_t::vertex_descriptor vertex_t;
/**
 * @brief edge type for State Machine Class (State Transition)
 *
 */
typedef graph_t::edge_descriptor edge_t;
typedef boost::graph_traits<graph_t>::adjacency_iterator adjacency_iterator_t;
typedef boost::graph_traits<graph_t>::out_edge_iterator out_edge_iterator_t;

/**
 * @brief Struct for State Infomation
 *
 */
struct StateInfo
{
  /**
   * @brief Possibe Transition States from the Current State
   *
   */
  const std::vector<std::string> possibe_transition_states;
  /**
   * @brief Possibe Transitions from the Current State
   *
   */
  const std::vector<std::string> possibe_transitions;
  /**
   * @brief Current State
   *
   */
  const std::string current_state;
  StateInfo(
    std::string current_state_,
    std::vector<std::string> possibe_transition_states_,
    std::vector<std::string> possibe_transitions_)
  : current_state(current_state_),
    possibe_transition_states(possibe_transition_states_),
    possibe_transitions(possibe_transitions_) {}
};

/**
 * @brief State Machine Class using Boost::Graph
 */
class StateMachine
{
public:
  explicit StateMachine(std::string xml_string);
  ~StateMachine();
  bool tryTransition(std::string trigger_event_name);
  bool setCurrentState(std::string current_state);
  std::vector<std::string> getPossibeTransitionStates();
  std::vector<std::string> getPossibeTransitions();
  std::string getCurrentState();
  StateInfo getStateInfo();
  void drawStateMachine(std::string dot_filename);
  std::string getDotString();
  std::string getName();
private:
  void addTransition(
    std::string from_state_name, std::string to_state_name,
    std::string trigger_event_name);
  std::mutex mtx_;
  graph_t state_graph_;
  vertex_t current_state_;
  std::string name_;
  template<typename Map>
  NodeWriter<Map> node_writer_(Map & map, std::string current_state)
  {
    return NodeWriter<Map>(map, current_state);
  }
  template<typename Map>
  EdgeWriter<Map> edge_writer_(Map & map) {return EdgeWriter<Map>(map);}
};
#endif  // POLARIS__TYPES__TASK__STATE_MACHINE_HPP_
