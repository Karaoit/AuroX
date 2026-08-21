#include "AuroX/Optimizer/Optimizer.hpp"

#include <algorithm>
#include <vector>

namespace AuroX {
namespace Optimizer {

// ---------------------------------------------------------------------------
// Optimizer (base)
// ---------------------------------------------------------------------------
void Optimizer::update(ParameterSpace& space, const std::vector<double>& gradient) {
    std::vector<double> delta = computeUpdate(space, gradient);
    std::vector<double> theta = space.vectorizeDouble();
    const size_t n = std::min(theta.size(), delta.size());
    for (size_t i = 0; i < n; ++i) theta[i] += delta[i];
    space.unvectorizeDouble(theta);
}

bool Optimizer::isStochastic() const { return false; }

std::string Optimizer::type() const { return name(); }

json Optimizer::config() const { return {}; }

void Optimizer::onEpochBegin() {}

// ---------------------------------------------------------------------------
// GradientDescent
// ---------------------------------------------------------------------------
GradientDescent::GradientDescent(double learningRate) : lr_(learningRate) {}

std::vector<double> GradientDescent::computeUpdate(ParameterSpace& space,
                                                   const std::vector<double>& gradient) {
    std::vector<double> theta = space.vectorizeDouble();
    std::vector<double> delta(theta.size(), 0.0);
    const size_t n = std::min(theta.size(), gradient.size());
    for (size_t i = 0; i < n; ++i) delta[i] = -lr_ * gradient[i];
    return delta;
}

bool GradientDescent::isStochastic() const { return false; }
const char* GradientDescent::name() const { return "GradientDescent"; }
json GradientDescent::config() const { return {{"lr", lr_}}; }

// ---------------------------------------------------------------------------
// SGD
// ---------------------------------------------------------------------------
SGD::SGD(double learningRate, double momentum, double decay)
    : lr_(learningRate), momentum_(momentum), decay_(decay) {}

std::vector<double> SGD::computeUpdate(ParameterSpace& space,
                                       const std::vector<double>& gradient) {
    if (velocity_.size() != gradient.size())
        velocity_.assign(gradient.size(), 0.0);
    std::vector<double> delta(gradient.size(), 0.0);
    const size_t n = gradient.size();
    for (size_t i = 0; i < n; ++i) {
        velocity_[i] = momentum_ * velocity_[i] - lr_ * gradient[i];
        delta[i] = velocity_[i];
    }
    (void)space;
    return delta;
}

bool SGD::isStochastic() const { return true; }
const char* SGD::name() const { return "SGD"; }
json SGD::config() const {
    return {{"lr", lr_}, {"momentum", momentum_}, {"decay", decay_}};
}

void SGD::onEpochBegin() {
    if (decay_ > 0.0) lr_ *= (1.0 - decay_);
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
std::unique_ptr<Optimizer> createOptimizer(const std::string& type,
                                           const json& config) {
    if (type == "GradientDescent")
        return std::make_unique<GradientDescent>(config.value("lr", 0.01));
    if (type == "SGD")
        return std::make_unique<SGD>(config.value("lr", 0.01),
                                     config.value("momentum", 0.9),
                                     config.value("decay", 0.0));
    return nullptr;
}

}  // namespace Optimizer
}  // namespace AuroX
