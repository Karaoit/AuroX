#pragma once

// ============================================================================
// AuroX - StateSpace  (part of the Space layer)
// ----------------------------------------------------------------------------
// MINIMAL standardized state holder. The Dynamics layer writes the estimated
// state x_hat here every tick; downstream consumers (Policy / Optimizer,
// Controller) read it. It intentionally holds NO estimation / prediction logic
// -- that lives in the Dynamics layer. That keeps StateSpace thin and lets the
// Dynamics brain be swapped without touching this recipient.
//
// Declarations only; method definitions live in StateSpace.cpp.
// ============================================================================

#include "json.hpp"

#include <string>
#include <vector>

namespace AuroX {
namespace Space {

using json = nlohmann::json;

class StateSpace {
public:
    // Write the current state estimate (called by Dynamics each tick).
    void setState(const std::vector<double>& x, double ts = 0.0);

    const std::vector<double>& getState() const;
    std::vector<double> getStateCopy() const;

    void setNames(std::vector<std::string> names);
    const std::vector<std::string>& names() const;

    size_t dim() const;
    bool hasState() const;
    double timestamp() const;

    void reset();

    // Description for the Operator Console / persistence.
    json describe() const;

private:
    std::vector<double> state_;
    std::vector<std::string> names_;
    double timestamp_ = 0.0;
    bool hasState_ = false;
};

}  // namespace Space
}  // namespace AuroX
