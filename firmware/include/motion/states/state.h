#pragma once

#include <functional>

class MotionSystem;

enum class MOTION_STATE { DEACTIVATED, IDLE, CALIBRATION, REST, STAND, CRAWL, WALK };

template <typename StateEnum>
class StateMachine;

template <typename StateEnum>
class State {
  public:
    virtual ~State() = default;
    virtual void enter() = 0;
    virtual void exit() = 0;
    virtual void update() = 0;
    virtual StateEnum getStateType() const = 0;
};

using TransitionFunction = std::function<bool(const MotionSystem&)>;