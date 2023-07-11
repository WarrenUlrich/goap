#include <goap/goap.hpp> // assuming the above code is saved in this header file
#include <iostream>
#include <string>
#include <chrono>

class goal;

class world_state {
public:
  int value;

  world_state(int value) : value(value) {}

  int heuristic(const goal &g) const noexcept;

  bool operator==(const world_state &other) const noexcept {
    return value == other.value;
  }
};

namespace std {
  template <>
  struct hash<world_state> {
    std::size_t operator()(const world_state &s) const noexcept {
      return std::hash<int>{}(s.value);
    }
  };
}

class goal {
public:
  int value;

  goal(int value) : value(value) {}

  bool satisfied(const world_state &state) const noexcept {
    return state.value == value;
  }
};

int world_state::heuristic(const goal &g) const noexcept {
  return g.value - value;
}

class increment_action : public goap::action<world_state> {
public:
  int amount;

  increment_action(int amount = 5) : amount(amount) {}

  bool preconditions(const world_state &state) const noexcept {
    return true;
  }

  world_state apply_effects(const world_state &state) const noexcept {
    return world_state(state.value + amount);
  }

  bool execute(world_state &state) const noexcept {
    std::cout << "Increment\n";
    state.value += amount;
    return true;
  }

  std::int32_t cost() const noexcept {
    return amount * 2;
  }
};

class decrement_action : public goap::action<world_state> {
public:
  int amount;

  decrement_action(int amount = 5) : amount(amount) {}

  bool preconditions(const world_state &state) const noexcept {
    return true;
  }

  world_state apply_effects(const world_state &state) const noexcept {
    return world_state(state.value - amount);
  }

  bool execute(world_state &state) const noexcept {
    std::cout << "Decrement\n";
    state.value -= amount;
    return true;
  }

  std::int32_t cost() const noexcept {
    return amount * 2;
  }
};

int main() {
  world_state state(-1);
  goal g(100);

  goap::planner<world_state, goal> planner;
  planner.add_action(std::make_shared<increment_action>(10));
  planner.add_action(std::make_shared<decrement_action>(1));

  const auto plan = planner.plan(state, g);
  for (const auto &action : plan) {
    action->execute(state);
  }

  std::cout << "Final state: " << state.value << std::endl;
  return 0;
}
