#include "AuroX/Space/StateSpace.hpp"

#include <string>
#include <vector>

namespace AuroX {
namespace Space {

// Write the current state estimate (called by Dynamics each tick).
void StateSpace::setState(const std::vector<double>& x, double ts) {
    state_ = x;
    timestamp_ = ts;
    hasState_ = true;
}

const std::vector<double>& StateSpace::getState() const { return state_; }
std::vector<double> StateSpace::getStateCopy() const { return state_; }

void StateSpace::setNames(std::vector<std::string> names) { names_ = std::move(names); }
const std::vector<std::string>& StateSpace::names() const { return names_; }

size_t StateSpace::dim() const { return state_.size(); }
bool StateSpace::hasState() const { return hasState_; }
double StateSpace::timestamp() const { return timestamp_; }

void StateSpace::reset() { state_.clear(); hasState_ = false; timestamp_ = 0.0; }

// Description for the Operator Console / persistence.
json StateSpace::describe() const {
    json j = json::object();
    j["dim"] = state_.size();
    j["hasState"] = hasState_;
    j["timestamp"] = timestamp_;
    json arr = json::array();
    for (double v : state_) arr.push_back(v);
    j["state"] = arr;
    j["names"] = names_;
    return j;
}

}  // namespace Space
}  // namespace AuroX
