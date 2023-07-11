#pragma once

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <memory>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

namespace goap {
template <typename T, typename Goal>
concept world_state = requires(T &s, Goal &g) {
  { s.heuristic(g) } -> std::convertible_to<std::int32_t>;
  { std::hash<T>{}(s) } -> std::convertible_to<std::size_t>;
};

template <typename T, typename WorldState>
concept goal = requires(T &g, WorldState &s) {
  { g.satisfied(s) } -> std::convertible_to<bool>;
};

template <typename WorldState> class action {
public:
  virtual bool
  preconditions(const WorldState &state) const noexcept = 0;

  virtual WorldState
  apply_effects(const WorldState &state) const noexcept = 0;

  virtual bool execute(WorldState &state) const noexcept = 0;
  
  virtual std::int32_t cost() const noexcept = 0;
};

template <typename WorldState, typename Goal>
requires world_state<WorldState, Goal> &&
goal<Goal, WorldState>
class planner {
public:
  using world_state_type = WorldState;
  using goal_type = Goal;
  using action_type = action<WorldState>;

  struct node {
    world_state_type state;
    std::shared_ptr<action_type> action_taken;
    int32_t cost;
    int32_t heuristic;
    int32_t total_cost;
    std::shared_ptr<node> parent;

    node(const world_state_type &state, std::shared_ptr<action_type> action_taken, int32_t cost, int32_t heuristic, std::shared_ptr<node> parent)
    : state(state), action_taken(action_taken), cost(cost), heuristic(heuristic), total_cost(cost + heuristic), parent(parent) {}
  };

  struct compare {
    bool operator()(const std::shared_ptr<node> &a, const std::shared_ptr<node> &b) {
      return a->total_cost > b->total_cost;
    }
  };

  constexpr planner() noexcept = default;

  std::vector<std::shared_ptr<action_type>>
  plan(const WorldState &start,
       const Goal &goal) const noexcept {
    std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node>>, compare> frontier;
    std::unordered_map<WorldState, std::shared_ptr<node>> came_from;
    std::shared_ptr<node> start_node = std::make_shared<node>(start, nullptr, 0, start.heuristic(goal), nullptr);
    frontier.push(start_node);
    came_from[start] = start_node;

    while (!frontier.empty()) {
      auto current = frontier.top();
      frontier.pop();

      if (goal.satisfied(current->state)) {
        std::vector<std::shared_ptr<action_type>> result;
        while (current->parent != nullptr) {
          result.push_back(current->action_taken);
          current = current->parent;
        }
        std::reverse(result.begin(), result.end());
        return result;
      }

      for (auto &action : _actions) {
        if (action->preconditions(current->state)) {
          WorldState next_state = action->apply_effects(current->state);
          int32_t new_cost = current->cost + action->cost();

          if (!came_from.count(next_state) || new_cost < came_from[next_state]->cost) {
            std::shared_ptr<node> next_node = std::make_shared<node>(next_state, action, new_cost, next_state.heuristic(goal), current);
            frontier.push(next_node);
            came_from[next_state] = next_node;
          }
        }
      }
    }

    return {}; // If there is no valid path to the goal
  }

  void add_action(std::shared_ptr<action_type> action) {
    _actions.push_back(std::move(action));
  }

private:
  std::vector<std::shared_ptr<action_type>> _actions;
};

} // namespace goap