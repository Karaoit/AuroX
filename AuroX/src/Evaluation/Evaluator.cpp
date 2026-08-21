#include "AuroX/Evaluation/Evaluator.hpp"

#include <algorithm>
#include <numeric>
#include <random>

namespace AuroX {
namespace Evaluation {

namespace {
    std::vector<size_t> buildAllIndices(size_t n) {
        std::vector<size_t> v(n);
        for (size_t i = 0; i < n; ++i) v[i] = i;
        return v;
    }
}  // namespace

RegressionEvaluator::RegressionEvaluator(std::vector<std::vector<double>> X,
                                         std::vector<double> y,
                                         bool stochastic,
                                         size_t miniBatchSize,
                                         unsigned int seed)
    : X_(std::move(X)),
      y_(std::move(y)),
      stochastic_(stochastic),
      miniBatchSize_(miniBatchSize),
      seed_(seed) {
    N_ = y_.size();
    D_ = (!X_.empty() && !X_[0].empty()) ? X_[0].size() : 0;
    order_ = buildAllIndices(N_);
}

EvaluatorResult RegressionEvaluator::compute(const std::vector<double>& params,
                                             const std::vector<size_t>& indices) const {
    EvaluatorResult r;
    r.gradient.assign(D_, 0.0);
    double loss = 0.0;
    for (size_t i : indices) {
        const auto& xi = X_[i];
        double pred = 0.0;
        for (size_t d = 0; d < D_; ++d) pred += xi[d] * params[d];
        const double err = pred - y_[i];
        loss += err * err;
        for (size_t d = 0; d < D_; ++d) r.gradient[d] += 2.0 * xi[d] * err;
    }
    // Divide by batch size to get the empirical mean (MSE and gradient).
    // Degrades to 0 for an empty batch.
    const double inv = indices.empty() ? 0.0 : 1.0 / static_cast<double>(indices.size());
    r.value = loss * inv;
    for (double& g : r.gradient) g *= inv;
    return r;
}

EvaluatorResult RegressionEvaluator::evaluate(const std::vector<double>& params) const {
    return compute(params, buildAllIndices(N_));
}

EvaluatorResult RegressionEvaluator::evaluateBatch(const std::vector<double>& params,
                                                   const std::vector<size_t>& indices) const {
    return compute(params, indices);
}

void RegressionEvaluator::beginEpoch() {
    std::mt19937 rng(seed_ + static_cast<unsigned int>(epochCounter_));
    ++epochCounter_;
    std::shuffle(order_.begin(), order_.end(), rng);
    cursor_ = 0;
}

std::vector<size_t> RegressionEvaluator::nextBatchIndices(size_t batchSize) {
    if (order_.empty()) beginEpoch();
    const size_t take = std::min(batchSize, order_.size() - cursor_);
    std::vector<size_t> out;
    out.reserve(take);
    for (size_t k = 0; k < take; ++k) out.push_back(order_[cursor_++]);
    return out;
}

json RegressionEvaluator::config() const {
    return {
        {"type", "RegressionEvaluator"},
        {"samples", N_},
        {"dimension", D_},
        {"stochastic", stochastic_},
        {"miniBatchSize", miniBatchSize_}
    };
}

json RegressionEvaluator::toJson() const {
    json j = json::object();
    j["type"] = "RegressionEvaluator";
    j["stochastic"] = stochastic_;
    j["miniBatchSize"] = miniBatchSize_;
    j["seed"] = seed_;
    j["samples"] = N_;
    j["dimension"] = D_;
    j["X"] = X_;
    j["y"] = y_;
    return j;
}

void RegressionEvaluator::fromJson(const json& j) {
    stochastic_ = j.value("stochastic", false);
    miniBatchSize_ = j.value("miniBatchSize", 32u);
    seed_ = j.value("seed", 1u);
    if (j.contains("X") && j["X"].is_array()) X_ = j["X"].get<std::vector<std::vector<double>>>();
    if (j.contains("y") && j["y"].is_array()) y_ = j["y"].get<std::vector<double>>();
    N_ = y_.size();
    D_ = (!X_.empty() && !X_[0].empty()) ? X_[0].size() : 0;
    order_ = buildAllIndices(N_);
    cursor_ = 0;
    epochCounter_ = 0;
}

std::unique_ptr<Evaluator> createEvaluator(const json& j) {
    const std::string t = j.value("type", "RegressionEvaluator");
    if (t == "RegressionEvaluator") {
        // Construct then load the full serialized form (dataset + hyperparams).
        auto ej = std::make_unique<RegressionEvaluator>(
            std::vector<std::vector<double>>{}, std::vector<double>{},
            j.value("stochastic", false), j.value("miniBatchSize", 32u),
            j.value("seed", 1u));
        ej->fromJson(j);
        return ej;
    }
    return nullptr;
}

}  // namespace Evaluation
}  // namespace AuroX
