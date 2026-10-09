#pragma once

// ============================================================================
// AuroX - ParameterSpace
// ----------------------------------------------------------------------------
// Purpose:
//   Unified parameter system supporting:
//
//   1. Dynamic ParameterSpace
//      - runtime declaration by string name;
//      - dynamic lookup;
//      - JSON serialization;
//      - optimizer vectorization.
//
//   2. PStruct<Derived>  (short for Parameter Struct; was StructuredSpace)
//      - strongly typed user-defined parameter structs;
//      - IDE member completion;
//      - automatic synchronization with PSpace;
//      - automatic serialization / deserialization;
//      - automatic vectorization / unvectorization.
//
// Example:
//
//   struct PIDParameters
//       : public AuroX::Space::PStruct<PIDParameters>
//   {
//       AuroX::Space::Param<double> Kp;
//       AuroX::Space::Param<double> Ki;
//       AuroX::Space::Param<double> Kd;
//       AuroX::Space::EnumParam mode;
//
//       // 1) declare the iteration list (used by sync / serialize / vectorize)
//       AUROX_PARAMS(Kp, Ki, Kd, mode)
//
//       PIDParameters()
//       {
//           // 2) register into the space. PARAM_BIND auto-derives the
//           //    registered name ("Kp") from the variable name, so you
//           //    never write the string literal again.
//           PARAM_REG(
//               PARAM_BIND(Kp, 1.0, 0.0, 10.0);
//               PARAM_BIND(Ki, 0.1, 0.0, 5.0);
//               PARAM_BIND(Kd, 0.05);
//               PARAM_BIND(mode, "auto", {"auto", "manual", "adaptive"});
//           );
//       }
//   };
//
// File layout (declaration / implementation split):
//   ParameterSpace.hpp - declarations only (this file)
//   PSpace.ipp - template implementations, included at the end of
//                        this header; do NOT include it directly
//   ParameterSpace.cpp - all non-template implementations
// ============================================================================

#include "json.hpp"
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

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

// ============================================================================
// Parameter metadata
// ============================================================================

enum class ParameterKind : int {
    Continuous = 0,
    Discrete = 1
};

enum class ParameterTypeId : int {
    Bool = 0,
    Int = 1,
    Float = 2,
    Double = 3,
    String = 4,
    Enum = 5,
    Custom = 99
};

// ============================================================================
// ParameterBase
// ============================================================================

class AUROX_API ParameterBase {
public:
    virtual ~ParameterBase() = default;

    virtual const std::string& name() const = 0;
    virtual ParameterKind kind() const = 0;
    virtual ParameterTypeId typeId() const = 0;
    virtual const char* typeName() const = 0;
    virtual std::unique_ptr<ParameterBase> clone() const = 0;
    virtual bool isWithinBounds() const = 0;
    bool inBounds() const { return isWithinBounds(); }
    virtual bool isValid() const;
    virtual bool isBounded() const;
    virtual json toJson() const = 0;
    virtual void fromJson(const json& j) = 0;
    virtual json valueToJson() const = 0;
    virtual void setValueFromJson(const json& v) = 0;
    virtual bool packDouble(double& out) const;
    virtual bool unpackDouble(double v);
    virtual std::size_t componentCount() const;
};

// ============================================================================
// Type metadata (data-only traits, definition must stay in the header)
// ============================================================================

template <typename T>
struct ParameterTypeInfo {
    static constexpr ParameterTypeId id = ParameterTypeId::Custom;
    static constexpr const char* name = "custom";
};

template <> struct ParameterTypeInfo<bool> {
    static constexpr ParameterTypeId id = ParameterTypeId::Bool;
    static constexpr const char* name = "bool";
};
template <> struct ParameterTypeInfo<int> {
    static constexpr ParameterTypeId id = ParameterTypeId::Int;
    static constexpr const char* name = "int";
};
template <> struct ParameterTypeInfo<float> {
    static constexpr ParameterTypeId id = ParameterTypeId::Float;
    static constexpr const char* name = "float";
};
template <> struct ParameterTypeInfo<double> {
    static constexpr ParameterTypeId id = ParameterTypeId::Double;
    static constexpr const char* name = "double";
};
template <> struct ParameterTypeInfo<std::string> {
    static constexpr ParameterTypeId id = ParameterTypeId::String;
    static constexpr const char* name = "string";
};

// ============================================================================
// Numeric traits
// (primary template only; the specializations contain functions and live in
//  PSpace.ipp)
// ============================================================================

template <typename T>
struct NumericTraits {
    static constexpr bool vectorizable = false;
};

// ============================================================================
// TypedParameter<T>
// ============================================================================

template <typename T>
class TypedParameter : public ParameterBase {
public:
    std::string name_;
    T value{};
    T lower{};
    T upper{};
    std::vector<T> choices;
    bool hasBounds = true;

    TypedParameter() = default;
    TypedParameter(std::string name, T def, T lo, T hi);
    TypedParameter(std::string name, T def);
    TypedParameter(std::string name, T def, std::vector<T> ch);

    const std::string& name() const override;
    ParameterKind kind() const override;
    ParameterTypeId typeId() const override;
    const char* typeName() const override;
    std::unique_ptr<ParameterBase> clone() const override;
    bool isBounded() const override;
    bool isWithinBounds() const override;
    json toJson() const override;
    void fromJson(const json& j) override;
    json valueToJson() const override;
    void setValueFromJson(const json& v) override;
    bool packDouble(double& out) const override;
    bool unpackDouble(double v) override;
    std::size_t componentCount() const override;
};

// ============================================================================
// EnumParameter (non-template, implementation in ParameterSpace.cpp)
// ============================================================================

class AUROX_API EnumParameter : public ParameterBase {
public:
    std::string name_;
    std::vector<std::string> labels_;
    std::size_t index_ = 0;

    EnumParameter() = default;
    EnumParameter(std::string name, std::size_t defIndex, std::vector<std::string> labels);

    const std::string& name() const override;
    ParameterKind kind() const override;
    ParameterTypeId typeId() const override;
    const char* typeName() const override;
    std::unique_ptr<ParameterBase> clone() const override;
    bool isBounded() const override;
    bool isWithinBounds() const override;

    std::size_t index() const;
    void setIndex(std::size_t i);
    const std::string& label() const;
    void setLabel(const std::string& l);
    const std::vector<std::string>& labels() const;

    json toJson() const override;
    void fromJson(const json& j) override;
    json valueToJson() const override;
    void setValueFromJson(const json& v) override;
    bool packDouble(double& out) const override;
    bool unpackDouble(double v) override;
    std::size_t componentCount() const override;
};

// ============================================================================
// PSpace
// ============================================================================

class AUROX_API ParameterSpace {
public:
    struct ComponentMeta {
        std::string name;
        ParameterTypeId type;
        std::size_t components;
        bool is_enum = false;
    };

    // Registration
    void add(std::unique_ptr<ParameterBase> p);

    TypedParameter<float>* declare(const std::string& name, float def, float lo, float hi);
    TypedParameter<double>* declare(const std::string& name, double def, double lo, double hi);
    TypedParameter<int>* declare(const std::string& name, int def, int lo, int hi);
    TypedParameter<bool>* declare(const std::string& name, bool def, bool lo, bool hi);

    TypedParameter<float>* declare(const std::string& name, float def);
    TypedParameter<double>* declare(const std::string& name, double def);
    TypedParameter<int>* declare(const std::string& name, int def);
    TypedParameter<bool>* declare(const std::string& name, bool def);

    TypedParameter<int>* declareDiscrete(const std::string& name, int def, std::vector<int> choices);
    TypedParameter<float>* declareDiscrete(const std::string& name, float def, std::vector<float> choices);
    TypedParameter<double>* declareDiscrete(const std::string& name, double def, std::vector<double> choices);
    TypedParameter<bool>* declareDiscrete(const std::string& name, bool def, std::vector<bool> choices);
    TypedParameter<std::string>* declareDiscrete(const std::string& name, std::string def, std::vector<std::string> choices);

    EnumParameter* declareEnum(const std::string& name, std::size_t defIndex, std::vector<std::string> labels);
    EnumParameter* declareEnum(const std::string& name, const std::string& defLabel, std::vector<std::string> labels);

    // Lookup
    ParameterBase* get(const std::string& name);
    const ParameterBase* get(const std::string& name) const;
    bool contains(const std::string& name) const;

    template <typename T>
    TypedParameter<T>* getAs(const std::string& name);
    template <typename T>
    const TypedParameter<T>* getAs(const std::string& name) const;

    ParameterBase* at(std::size_t i);
    const ParameterBase* at(std::size_t i) const;
    EnumParameter* getEnum(const std::string& name);
    const EnumParameter* getEnum(const std::string& name) const;

    // Container
    std::size_t size() const;
    bool empty() const;
    void clear();
    std::vector<std::string> names() const;

    // Serialization
    json toJson() const;
    void fromJson(const json& j);
    bool tryFromJson(const json& j) noexcept;

    // Typed vectorization
    template <typename T>
    std::vector<T> vectorize() const;
    template <typename T>
    void unvectorize(const std::vector<T>& v);

    // Unified optimizer vectorization
    std::vector<double> vectorizeDouble() const;
    void unvectorizeDouble(const std::vector<double>& v);
    std::vector<ComponentMeta> componentLayout() const;

    // Short aliases (zero-break)
    std::vector<double> vec() const { return vectorizeDouble(); }
    void unvec(const std::vector<double>& v) { unvectorizeDouble(v); }
    std::vector<ComponentMeta> layout() const { return componentLayout(); }

private:
    std::vector<std::unique_ptr<ParameterBase>> items_;
    std::unordered_map<std::string, std::size_t> index_;
};

// ============================================================================
// Param<T>
// ============================================================================

template <typename T>
class Param {
public:
    using value_type = T;

    Param() = default;

    void bind(ParameterSpace& space, std::string_view name, T def, T lo, T hi);
    void bind(ParameterSpace& space, std::string_view name, T def);
    void bind(ParameterSpace& space, std::string_view name, T def, std::vector<T> choices);
    bool rebind(ParameterSpace& space);
    void sync_to_raw();
    void sync_from_raw();

    Param& operator=(const T& v);
    Param& operator=(T&& v);
    operator const T&() const;
    operator T&();
    T& get();
    const T& get() const;

    const std::string& name() const;
    bool isBound() const;
    bool isValid() const;
    bool inBounds() const { return isValid(); }
    TypedParameter<T>* raw();
    const TypedParameter<T>* raw() const;

protected:
    T value{};
private:
    std::string name_;
    TypedParameter<T>* raw_ = nullptr;
};

// ============================================================================
// EnumParam (non-template, implementation in ParameterSpace.cpp)
// ============================================================================

class EnumParam {
public:
    EnumParam() = default;

    void bind(ParameterSpace& space, std::string_view name, const std::string& def, std::vector<std::string> labels);
    bool rebind(ParameterSpace& space);
    void sync_to_raw();
    void sync_from_raw();

    EnumParam& operator=(const std::string& v);
    EnumParam& operator=(std::string&& v);
    EnumParam& operator=(const char* v);
    operator const std::string&() const;
    std::string& get();
    const std::string& get() const;

    const std::string& name() const;
    bool isBound() const;
    bool isValid() const;
    bool inBounds() const { return isValid(); }
    EnumParameter* raw();
    const EnumParameter* raw() const;

protected:
    std::string value;
private:
    std::string name_;
    EnumParameter* raw_ = nullptr;
};

// ============================================================================
// PStruct<Derived>
// ============================================================================

template <typename Derived>
class PStruct {
public:
    PStruct() = default;
    virtual ~PStruct() = default;
    PStruct(const PStruct&) = delete;
    PStruct& operator=(const PStruct&) = delete;
    PStruct(PStruct&&) noexcept = default;
    PStruct& operator=(PStruct&&) noexcept = default;

    template <typename Fn>
    void registerParams(Fn&& fn);
    void sync_to_space();
    void sync_from_space();
    bool rebind();

    ParameterSpace& space();
    const ParameterSpace& space() const;

    json toJson() const;
    void fromJson(const json& j);
    bool tryFromJson(const json& j) noexcept;

    std::vector<double> vectorize() const;
    std::vector<double> vectorizeDouble() const;
    void unvectorize(const std::vector<double>& v);
    void unvectorizeDouble(const std::vector<double>& v);

    // Short aliases (zero-break)
    std::vector<double> vec() const { return vectorize(); }
    void unvec(const std::vector<double>& v) { unvectorize(v); }
    // 注：ComponentMeta 是 ParameterSpace 的嵌套类型，PStruct 并不继承它，
    //     因此这里必须写全限定名（原先的裸 ComponentMeta 无法通过编译）。
    std::vector<ParameterSpace::ComponentMeta> layout() const { return componentLayout(); }

    bool validate() const;

    std::size_t size() const;
    bool empty() const;
    std::vector<std::string> names() const;
    std::vector<ParameterSpace::ComponentMeta> componentLayout() const;

    ParameterBase* get(const std::string& name);
    const ParameterBase* get(const std::string& name) const;

protected:
    Derived& derived();
    const Derived& derived() const;
    ParameterSpace space_;
};

// Backward-compatible alias. New code should use PStruct directly.
template <typename Derived>
using StructuredSpace = PStruct<Derived>;

// ---- Convenience aliases (zero-break, fully backward compatible) ----
template <typename T>
using TParam = Param<T>;

using EParamRaw = EnumParameter;

using CompMeta = ParameterSpace::ComponentMeta;

// Non-throwing file loader: read `path`, parse JSON, then call tryFromJson.
// Works for both ParameterSpace and PStruct<Derived> (both expose tryFromJson).
template <typename SpaceT>
bool tryLoadJson(SpaceT& s, const std::string& path) noexcept;

// ============================================================================
// Parameter macros  (must stay in the header)
//   - AUROX_PARAMS(...)       : generates for_each_param inside the derived
//                               struct (used by sync / serialize / vectorize).
//   - PARAM_REG(...)          : replaces the verbose
//                               registerParams([this](auto& space){ ... })
//                               lambda boilerplate.
//   - PARAM_BIND(param, ...)  : calls param.bind(space, "param", ...) where the
//                               registered name is auto-derived from the
//                               variable name, so the string literal is gone.
// ============================================================================

#define AUROX_PARAMS(...)                                   \
    template <typename Fn>                                  \
    void for_each_param(Fn&& fn)                            \
    {                                                       \
        auto aurox_apply_params =                           \
            [&](auto&... params)                            \
            {                                               \
                (fn(params), ...);                          \
            };                                              \
        aurox_apply_params(__VA_ARGS__);                    \
    }                                                       \
    template <typename Fn>                                  \
    void for_each_param(Fn&& fn) const                      \
    {                                                       \
        auto aurox_apply_params =                           \
            [&](const auto&... params)                      \
            {                                               \
                (fn(params), ...);                          \
            };                                              \
        aurox_apply_params(__VA_ARGS__);                    \
    }

// PARAM_REG wraps the registration boilerplate. Inside it, the PSpace
// reference is named `space`, which PARAM_BIND relies on.
#define PARAM_REG(...)                                      \
    registerParams([this](auto& space) { __VA_ARGS__ })

// PARAM_BIND stringizes the variable name as the registered parameter name,
// so you no longer repeat the string
// (e.g. Kp.bind(space, "Kp", ...)  ->  PARAM_BIND(Kp, ...)).
#define PARAM_BIND(param, ...)                              \
    (param).bind(space, #param, __VA_ARGS__)


// Backward-compatible macro names (kept for compatibility; prefer PARAM_REG / PARAM_BIND).
#define AUROX_REG   PARAM_REG
#define AUROX_BIND  PARAM_BIND
} // namespace Space
} // namespace AuroX

// Namespace alias: write PSpace::X instead of AuroX::Space::X.
//   PSpace::Param<double>, PSpace::PStruct<...>, PSpace::TParam<double>, ...
namespace PSpace = AuroX::Space;

// Template implementations (NumericTraits specializations, TypedParameter<T>,
// ParameterSpace template members, Param<T>, PStruct<Derived>).
#include "ParameterSpace.ipp"
