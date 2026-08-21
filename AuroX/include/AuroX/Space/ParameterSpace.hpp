#pragma once

// ============================================================================
// AuroX - ParameterSpace
// ----------------------------------------------------------------------------
// Purpose:
//   Defines a unified parameter space that supports continuous and discrete
//   parameters, and meets the MVP requirements:
//     1. Parameter values are expressed with standard C++ types
//        (bool / int / float / double / std::string);
//     2. ParameterBase is an abstract base class; users can derive their own
//        parameter types when consuming AuroX;
//     3. Full serialization / deserialization via nlohmann::json, so an
//        external config layer can initialize the whole space;
//     4. Vectorization / unvectorization operators:
//          - vectorize<T>()       collects same-type parameters into a
//                                 contiguous std::vector<T> for optimizers;
//          - vectorizeDouble()    packs all numeric parameters into a single
//                                 std::vector<double>, optimizer-friendly;
//          - unvectorize*()       writes an optimizer-updated vector back into
//                                 the parameter objects;
//          - toJson()/valueToJson() exposes structured parameters to the UI.
//
// Design notes:
//   - ParameterSpace owns all parameter objects (std::unique_ptr) and provides
//     add / get / declare;
//   - TypedParameter<T> covers the common standard types; users normally do
//     not need to write their own;
//   - For fully custom parameters (e.g. struct parameters), inherit from
//     ParameterBase, implement all virtuals, and register via
//     add(std::make_unique<MyParam>(...)). The framework needs no changes.
//
// Note on implementation layout:
//   This header contains only declarations and comments plus the template
//   member functions (which must live in the header by C++ rules). All
//   non-template implementations reside in ParameterSpace.cpp.
// ============================================================================

#include "json.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

// DLL export macro: empty when compiled as a static library; define
// AUROX_BUILD_DLL (export) or AUROX_USE_DLL (import) to enable when building
// AuroX.dll.
#ifndef AUROX_API
#  if defined(_WIN32) && defined(AUROX_BUILD_DLL)
#    define AUROX_API __declspec(dllexport)
#  elif defined(_WIN32) && defined(AUROX_USE_DLL)
#    define AUROX_API __declspec(dllimport)
#  else
#    define AUROX_API
#  endif
#endif

namespace AuroX {
namespace Space {

using json = nlohmann::json;

// Parameter category: continuous (with bounds) or discrete (finite choices).
enum class ParameterKind : int {
    Continuous = 0,
    Discrete = 1
};

// Type identifier: used for serialization, type-grouped vectorization, and UI.
enum class ParameterTypeId : int {
    Bool = 0,
    Int = 1,
    Float = 2,
    Double = 3,
    String = 4,
    Enum = 5,
    Custom = 99
};

// Abstract parameter base class.
// All parameters (built-in TypedParameter<T> or user-defined) derive from it.
// The pure-virtual interface guarantees: naming, kind, type, cloning, bounds
// checking, serialization, and vectorization.
class AUROX_API ParameterBase {
public:
    virtual ~ParameterBase() = default;

    // --- Identity ---
    virtual const std::string& name() const = 0;
    virtual ParameterKind kind() const = 0;
    virtual ParameterTypeId typeId() const = 0;
    virtual const char* typeName() const = 0;

    // --- Deep copy: used for episode snapshots, warm-start backups, safe rollback ---
    virtual std::unique_ptr<ParameterBase> clone() const = 0;

    // --- Validity / safety-layer entry: bounds for continuous, choices for discrete ---
    virtual bool isWithinBounds() const = 0;
    virtual bool isValid() const { return isWithinBounds(); }

    // --- Whether explicit bounds are declared ---
    // true  = the parameter space enforces numeric bounds;
    // false = unbounded (some parameters have no bounds); the safety layer
    //         takes over the constraint.
    virtual bool isBounded() const { return false; }

    // --- Serialization: full description of a single parameter (bounds / choices) ---
    virtual json toJson() const = 0;
    virtual void fromJson(const json& j) = 0;

    // --- Value exposure for the UI (structured, human-readable) ---
    virtual json valueToJson() const = 0;
    virtual void setValueFromJson(const json& v) = 0;

    // --- Unified double vectorization (used by vectorizeDouble) ---
    // Not vectorizable by default (e.g. strings). Numeric parameters override.
    virtual bool packDouble(double& out) const { (void)out; return false; }
    virtual bool unpackDouble(double v) { (void)v; return false; }

    // Number of components this parameter contributes to the double vector
    // (0 means not vectorizable).
    virtual size_t componentCount() const { return 0; }
};

// Type traits: map C++ standard types to ParameterTypeId / name.
template <typename T>
struct ParameterTypeInfo {
    static constexpr ParameterTypeId id = ParameterTypeId::Custom;
    static constexpr const char* name = "custom";
};
template <>
struct ParameterTypeInfo<bool> {
    static constexpr ParameterTypeId id = ParameterTypeId::Bool;
    static constexpr const char* name = "bool";
};
template <>
struct ParameterTypeInfo<int> {
    static constexpr ParameterTypeId id = ParameterTypeId::Int;
    static constexpr const char* name = "int";
};
template <>
struct ParameterTypeInfo<float> {
    static constexpr ParameterTypeId id = ParameterTypeId::Float;
    static constexpr const char* name = "float";
};
template <>
struct ParameterTypeInfo<double> {
    static constexpr ParameterTypeId id = ParameterTypeId::Double;
    static constexpr const char* name = "double";
};
template <>
struct ParameterTypeInfo<std::string> {
    static constexpr ParameterTypeId id = ParameterTypeId::String;
    static constexpr const char* name = "string";
};

// Numeric traits: whether a type can be packed into a double vector.
// Strings and other non-numeric types are not vectorizable.
template <typename T>
struct NumericTraits {
    static constexpr bool vectorizable = false;
};
template <>
struct NumericTraits<bool> {
    static constexpr bool vectorizable = true;
    static double conv(bool v) { return v ? 1.0 : 0.0; }
    static bool back(double d) { return d != 0.0; }
};
template <>
struct NumericTraits<int> {
    static constexpr bool vectorizable = true;
    static double conv(int v) { return static_cast<double>(v); }
    static int back(double d) { return static_cast<int>(d); }
};
template <>
struct NumericTraits<float> {
    static constexpr bool vectorizable = true;
    static double conv(float v) { return static_cast<double>(v); }
    static float back(double d) { return static_cast<float>(d); }
};
template <>
struct NumericTraits<double> {
    static constexpr bool vectorizable = true;
    static double conv(double v) { return v; }
    static double back(double d) { return d; }
};

// Built-in templated parameter: covers bool / int / float / double / std::string.
//   - empty choices => continuous (use lower/upper bounds);
//   - non-empty choices => discrete (value must be in the candidate set).
// Fields are public for easy construction and access; declare() can also be used.
// All member functions are templates and therefore remain in this header.
template <typename T>
class TypedParameter : public ParameterBase {
public:
    std::string name_;
    T value{};
    T lower{};
    T upper{};
    std::vector<T> choices;   // non-empty => discrete
    bool hasBounds = true;    // true=explicit bounds; false=unbounded (safety layer)

    TypedParameter() = default;
    // Continuous with explicit bounds.
    TypedParameter(std::string name, T def, T lo, T hi)
        : name_(std::move(name)), value(def), lower(lo), upper(hi), hasBounds(true) {}
    // Continuous, unbounded (no bounds; constraint delegated to the safety layer).
    TypedParameter(std::string name, T def)
        : name_(std::move(name)), value(def), hasBounds(false) {}
    // Discrete with a finite candidate set (constraint comes from choices).
    TypedParameter(std::string name, T def, std::vector<T> ch)
        : name_(std::move(name)), value(def), hasBounds(false), choices(std::move(ch)) {}

    const std::string& name() const override { return name_; }
    ParameterKind kind() const override {
        return choices.empty() ? ParameterKind::Continuous : ParameterKind::Discrete;
    }
    ParameterTypeId typeId() const override { return ParameterTypeInfo<T>::id; }
    const char* typeName() const override { return ParameterTypeInfo<T>::name; }

    std::unique_ptr<ParameterBase> clone() const override {
        return std::make_unique<TypedParameter<T>>(*this);
    }

    bool isBounded() const override { return hasBounds; }

    bool isWithinBounds() const override {
        if (!choices.empty()) {
            for (const auto& c : choices)
                if (c == value) return true;
            return false;
        }
        if (!hasBounds) return true;   // Unbounded: not constrained here.
        return value >= lower && value <= upper;
    }

    json toJson() const override {
        json j = json::object();
        j["name"] = name_;
        j["type"] = typeName();
        j["kind"] = choices.empty() ? "continuous" : "discrete";
        j["value"] = value;
        if (choices.empty()) {
            j["bounded"] = hasBounds;
            if (hasBounds) {
                j["lower"] = lower;
                j["upper"] = upper;
            }
        } else {
            j["choices"] = choices;
        }
        return j;
    }

    void fromJson(const json& j) override {
        if (j.contains("value")) value = j["value"].get<T>();
        if (j.contains("choices") && !j["choices"].empty()) {
            choices = j["choices"].get<std::vector<T>>();
            hasBounds = false;            // Discrete: constraint from candidate set.
        } else if (j.contains("lower") && j.contains("upper")) {
            lower = j["lower"].get<T>();
            upper = j["upper"].get<T>();
            hasBounds = true;            // Explicit bounds.
        } else {
            hasBounds = false;           // Unbounded: delegated to the safety layer.
        }
    }

    json valueToJson() const override { return value; }
    void setValueFromJson(const json& v) override { value = v.get<T>(); }

    bool packDouble(double& out) const override {
        if constexpr (NumericTraits<T>::vectorizable) {
            out = NumericTraits<T>::conv(value);
            return true;
        } else {
            (void)out;
            return false;
        }
    }
    bool unpackDouble(double v) override {
        if constexpr (NumericTraits<T>::vectorizable) {
            value = NumericTraits<T>::back(v);
            return true;
        } else {
            (void)v;
            return false;
        }
    }
    size_t componentCount() const override {
        return NumericTraits<T>::vectorizable ? 1u : 0u;
    }
};

// ----------------------------------------------------------------------------
// Enum (categorical) parameter: a discrete variable whose choices are NAMED
// labels rather than raw numbers. Internally it stores the selected index into
// the label list. It is Discrete by kind, and vectorizes as its selected index
// (a single double component), so it integrates with vectorizeDouble() /
// unvectorizeDouble() seamlessly.
//
// NOTE (MVP limitation): a gradient optimizer treats the index numerically.
// Categorical-aware optimization (e.g. one-hot encoding + a categorical
// solver) is future work; for now the enum packs/unpacks as a plain index.
// ----------------------------------------------------------------------------
class AUROX_API EnumParameter : public ParameterBase {
public:
    std::string name_;
    std::vector<std::string> labels_;   // K known labels; order == index
    size_t index_ = 0;                  // currently selected label index

    EnumParameter() = default;
    EnumParameter(std::string name, size_t defIndex, std::vector<std::string> labels)
        : name_(std::move(name)),
          labels_(std::move(labels)),
          index_(defIndex < labels_.size() ? defIndex : 0) {}

    const std::string& name() const override { return name_; }
    ParameterKind kind() const override { return ParameterKind::Discrete; }
    ParameterTypeId typeId() const override { return ParameterTypeId::Enum; }
    const char* typeName() const override { return "enum"; }

    std::unique_ptr<ParameterBase> clone() const override {
        return std::make_unique<EnumParameter>(*this);
    }

    // Constraint comes from the label set, not numeric bounds.
    bool isBounded() const override { return false; }
    bool isWithinBounds() const override { return index_ < labels_.size(); }

    // --- Accessors ---
    size_t index() const { return index_; }
    void setIndex(size_t i) {
        if (labels_.empty()) { index_ = 0; return; }
        index_ = (i < labels_.size()) ? i : labels_.size() - 1;
    }
    const std::string& label() const {
        static const std::string empty;
        return (index_ < labels_.size()) ? labels_[index_] : empty;
    }
    void setLabel(const std::string& l) {
        for (size_t i = 0; i < labels_.size(); ++i) {
            if (labels_[i] == l) { index_ = i; return; }
        }
        // Unknown label: extend the label set so it can be represented.
        labels_.push_back(l);
        index_ = labels_.size() - 1;
    }
    const std::vector<std::string>& labels() const { return labels_; }

    json toJson() const override {
        json j = json::object();
        j["name"] = name_;
        j["type"] = "enum";
        j["kind"] = "discrete";
        j["labels"] = labels_;
        j["value"] = index_;
        j["label"] = label();
        return j;
    }
    void fromJson(const json& j) override {
        if (j.contains("labels") && j["labels"].is_array()) {
            labels_ = j["labels"].get<std::vector<std::string>>();
        }
        if (j.contains("value")) {
            if (j["value"].is_number()) {
                setIndex(static_cast<size_t>(j["value"].get<long long>()));
            } else if (j["value"].is_string()) {
                setLabel(j["value"].get<std::string>());
            }
        } else if (j.contains("label")) {
            setLabel(j["label"].get<std::string>());
        }
    }

    json valueToJson() const override { return label(); }
    void setValueFromJson(const json& v) override {
        if (v.is_string())      setLabel(v.get<std::string>());
        else if (v.is_number()) setIndex(static_cast<size_t>(v.get<long long>()));
    }

    bool packDouble(double& out) const override {
        out = static_cast<double>(index_);
        return true;
    }
    bool unpackDouble(double v) override {
        setIndex(static_cast<size_t>(static_cast<long long>(std::llround(v))));
        return true;
    }
    size_t componentCount() const override { return 1; }
};

// Parameter space: manages a set of named parameters, with declaration,
// lookup, serialization, and vectorization.
class AUROX_API ParameterSpace {
public:
    // Metadata for each vectorized component (alignment + UI display).
    struct ComponentMeta {
        std::string name;
        ParameterTypeId type;
        size_t components;
    };

    // --- Registration ---
    // Add an already-constructed parameter (supports user-defined types with no
    // framework changes). The space takes ownership.
    void add(std::unique_ptr<ParameterBase> p);

    // Convenience declarations for built-in continuous parameters.
    // Return a raw pointer; the space still owns the object.
    TypedParameter<float>* declare(const std::string& name, float def, float lo, float hi);
    TypedParameter<double>* declare(const std::string& name, double def, double lo, double hi);
    TypedParameter<int>* declare(const std::string& name, int def, int lo, int hi);
    TypedParameter<bool>* declare(const std::string& name, bool def, bool lo, bool hi);

    // Convenience declarations for built-in continuous parameters (unbounded):
    // no explicit bounds are declared; the safety layer takes over.
    TypedParameter<float>* declare(const std::string& name, float def);
    TypedParameter<double>* declare(const std::string& name, double def);
    TypedParameter<int>* declare(const std::string& name, int def);
    TypedParameter<bool>* declare(const std::string& name, bool def);

    // Convenience declarations for built-in discrete parameters.
    TypedParameter<float>* declareDiscrete(const std::string& name, float def, std::vector<float> choices);
    TypedParameter<int>* declareDiscrete(const std::string& name, int def, std::vector<int> choices);
    TypedParameter<std::string>* declareDiscrete(const std::string& name, std::string def, std::vector<std::string> choices);

    // Convenience declarations for enum (categorical) parameters.
    //   - defIndex : index of the default label within `labels`;
    //   - defLabel : default label (looked up in `labels`).
    EnumParameter* declareEnum(const std::string& name, size_t defIndex, std::vector<std::string> labels);
    EnumParameter* declareEnum(const std::string& name, const std::string& defLabel, std::vector<std::string> labels);

    // --- Lookup ---
    ParameterBase* get(const std::string& name);
    const ParameterBase* get(const std::string& name) const;
    bool contains(const std::string& name) const;

    // Type-safe access (dynamic_cast; returns nullptr on failure / mismatch).
    template <typename T>
    TypedParameter<T>* getAs(const std::string& name) {
        auto* p = get(name);
        return (p && dynamic_cast<TypedParameter<T>*>(p)) ? static_cast<TypedParameter<T>*>(p) : nullptr;
    }
    template <typename T>
    const TypedParameter<T>* getAs(const std::string& name) const {
        const auto* p = get(name);
        return (p && dynamic_cast<const TypedParameter<T>*>(p)) ? static_cast<const TypedParameter<T>*>(p) : nullptr;
    }

    ParameterBase* at(size_t i);
    const ParameterBase* at(size_t i) const;

    // Enum-parameter lookup (returns nullptr on absence / type mismatch).
    EnumParameter* getEnum(const std::string& name);
    const EnumParameter* getEnum(const std::string& name) const;

    // --- Container properties ---
    size_t size() const;
    bool empty() const;
    void clear();

    std::vector<std::string> names() const;

    // --- Serialize the whole space (config persistence / network transport) ---
    json toJson() const;
    // Initialize the whole space from config (missing params are created,
    // duplicates are overwritten).
    void fromJson(const json& j);

    // --- Vectorization / unvectorization ---
    // Collect same-type parameters into contiguous memory (for optimizers).
    template <typename T>
    std::vector<T> vectorize() const {
        std::vector<T> out;
        for (const auto& p : items_) {
            if (auto* tp = dynamic_cast<TypedParameter<T>*>(p.get())) {
                out.push_back(tp->value);
            }
        }
        return out;
    }
    template <typename T>
    void unvectorize(const std::vector<T>& v) {
        size_t i = 0;
        for (auto& p : items_) {
            if (auto* tp = dynamic_cast<TypedParameter<T>*>(p.get())) {
                if (i < v.size()) tp->value = v[i++];
            }
        }
    }

    // Unified double vector (most optimizer-friendly).
    std::vector<double> vectorizeDouble() const;
    void unvectorizeDouble(const std::vector<double>& v);

    // Layout of vectorized components: metadata per vectorizable parameter.
    std::vector<ComponentMeta> componentLayout() const;

private:
    std::vector<std::unique_ptr<ParameterBase>> items_;
    std::unordered_map<std::string, size_t> index_;
};

}  // namespace Space
}  // namespace AuroX
