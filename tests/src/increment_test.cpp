#include <chrono>
#include <goap/goap.hpp>
#include <iostream>
#include <string>

class world_state {
public:
  int value;

  world_state(int value) : value(value) {}

  bool satisfied(const world_state &other) const noexcept {
    return value == other.value;
  }

  std::int32_t
  heuristic(const world_state &other) const noexcept {
    return std::abs(value - other.value);
  }

  bool operator==(const world_state &other) const noexcept {
    return value == other.value;
  }
};

namespace std {
template <> struct hash<world_state> {
  std::size_t
  operator()(const world_state &s) const noexcept {
    return std::hash<int>{}(s.value);
  }
};
} // namespace std

class increment_action
    : public goap::base_action<world_state> {
public:
  int amount;

  increment_action(int amount = 5) : amount(amount) {}

  bool
  preconditions(const world_state &state) const noexcept {
    return true;
  }

  world_state
  apply_effects(const world_state &state) const noexcept {
    world_state new_state = state;
    _effect(new_state);
    return new_state;
  }

  void execute(world_state &state) const noexcept {
    _effect(state);
  }

  std::int32_t
  cost(const world_state &state) const noexcept {
    return std::abs(amount) * 2;
  }

private:
  void _effect(world_state &state) const noexcept {
    state.value += amount;
  }
};

class decrement_action
    : public goap::base_action<world_state> {
public:
  int amount;

  decrement_action(int amount = 5) : amount(amount) {}

  bool
  preconditions(const world_state &state) const noexcept {
    return true;
  }

  world_state
  apply_effects(const world_state &state) const noexcept {
    return world_state(state.value - amount);
  }

  std::int32_t
  cost(const world_state &state) const noexcept {
    return 1;
  }
};

int main() {
  world_state state(-1);
  world_state goal(100);

  goap::planner<world_state, world_state, increment_action>
      planner;
  planner.add_action(
      std::make_shared<increment_action>(10));
  planner.add_action(
      std::make_shared<increment_action>(-1));

  const auto plan = planner.plan(state, goal);
  for (const auto &action : plan) {
    action->execute(state);
  }

  std::cout << "Final state: " << state.value << std::endl;
  return 0;
}
