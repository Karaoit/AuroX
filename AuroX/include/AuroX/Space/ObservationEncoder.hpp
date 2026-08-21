#pragma once

// ============================================================================
// AuroX - ObservationEncoder  (part of the Space layer)
// ----------------------------------------------------------------------------
// Purpose:
//   Three DEFAULT encoders that turn a single external quantifiable input into
//   a fixed-length std::vector<double>. They are the "observation resolution"
//   half of an ObservationChannel.
//
// Design contract (per the MVP decision):
//   - PURE vectorization only:  input -> vector<double>.
//   - STATELESS: no member changes on encode(); safe to share / call repeatedly.
//   - NO serialization / deserialization / unvectorization: these objects are
//     config, described (not reconstructed) by their owning channel.
//   - ONLY consume input: they never read parameters, models, or history.
//
// The three kinds match the three variable classes discussed:
//   - ContinuousEncoder  : continuous (real-valued) variables
//   - DiscreteEncoder     : discrete / ordinal numeric variables
//   - CategoricalEncoder  : nominal / enum (unordered labels) variables
//
// Declarations only; method definitions live in ObservationEncoder.cpp.
// ============================================================================

#include <string>
#include <vector>

namespace AuroX {
namespace Space {

// ----------------------------------------------------------------------------
// ContinuousEncoder
//   Normalize a single real-valued sample to a fixed-length vector.
//   mode:
//     MinMax    : map [min,max] -> [0,1] (out-of-range clamped)
//     ZScore    : map (x-mean)/std, clamped to [clampLo,clampHi]
//     None      : pass-through (assumed pre-normalized)
//   resolution R:
//     R == 1 -> a single normalized scalar (default)
//     R  > 1 -> a soft (triangular-kernel) histogram over [min,max] of length R
//   Out-of-range is clamped; no value is ever written to the vector itself.
// ----------------------------------------------------------------------------
class ContinuousEncoder {
public:
    enum class Mode { MinMax, ZScore, None };

    Mode   mode = Mode::MinMax;
    double min = 0.0, max = 1.0;          // MinMax domain
    double mean = 0.0, std = 1.0;        // ZScore statistics
    double clampLo = -1.0, clampHi = 1.0; // ZScore output clamp
    size_t resolution = 1;                // output length (1 = scalar; >1 = histogram)

    std::vector<double> encode(double x) const;
    size_t outputDim() const;

private:
    std::vector<double> encodeHistogram(double x) const;
};

// ----------------------------------------------------------------------------
// DiscreteEncoder
//   Ordered / integer discrete value.
//   mode:
//     Scalar     : map the value's rank to a single scalar in [0,1]
//     Thermometer: ordered one-hot of length = number of levels (preserves the
//                  ordinal relationship between adjacent levels)
//   levels: the ordered set of allowed values; empty => identity (already a
//           number, returned as a single scalar).
// ----------------------------------------------------------------------------
class DiscreteEncoder {
public:
    enum class Mode { Scalar, Thermometer };

    Mode mode = Mode::Scalar;
    std::vector<double> levels;   // ordered allowed values; empty => identity

    std::vector<double> encode(double x) const;
    size_t outputDim() const;
};

// ----------------------------------------------------------------------------
// CategoricalEncoder
//   Nominal / enum variable (unordered labels). Output = one-hot of length K,
//   with an EXTRA "unknown" bit (length K+1) so that an unseen label is not
//   silently treated as a valid category (it would otherwise be all-zero and
//   indistinguishable from a real label). Unknown labels light the unknown bit.
// ----------------------------------------------------------------------------
class CategoricalEncoder {
public:
    std::vector<std::string> labels;   // K known labels; order == one-hot index

    std::vector<double> encode(const std::string& label) const;
    size_t outputDim() const;
};

}  // namespace Space
}  // namespace AuroX
