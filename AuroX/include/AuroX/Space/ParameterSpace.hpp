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
//   2. StructuredSpace<Derived>
//      - strongly typed user-defined parameter structs;
//      - IDE member completion;
//      - automatic synchronization with ParameterSpace;
//      - automatic serialization / deserialization;
//      - automatic vectorization / unvectorization.
//
// Example:
//
//   struct PIDParameters
//       : public AuroX::Space::StructuredSpace<PIDParameters>
//   {
//       AuroX::Space::Param<double> Kp;
//       AuroX::Space::Param<double> Ki;
//       AuroX::Space::Param<double> Kd;
//       AuroX::Space::EnumParam mode;
//
//       AUROX_STRUCTURED_PARAMS(Kp, Ki, Kd, mode)
//
//       PIDParameters()
//       {
//           registerParams([this](auto& space)
//           {
//               Kp.bind(space, "Kp", 1.0, 0.0, 10.0);
//               Ki.bind(space, "Ki", 0.1, 0.0, 5.0);
//               Kd.bind(space, "Kd", 0.05);
//               mode.bind(
//                   space,
//                   "mode",
//                   "auto",
//                   {"auto", "manual", "adaptive"}
//               );
//           });
//       }
//   };
//
// File layout (declaration / implementation split):
//   ParameterSpace.hpp - declarations only (this file)
//   ParameterSpace.ipp - template implementations, included at the end of
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
//  ParameterSpace.ipp)
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
// ParameterSpace
// ============================================================================

class AUROX_API ParameterSpace {
public:
    struct ComponentMeta {
        std::string name;
        ParameterTypeId type;
        std::size_t components;
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

    // Typed vectorization
    template <typename T>
    std::vector<T> vectorize() const;
    template <typename T>
    void unvectorize(const std::vector<T>& v);

    // Unified optimizer vectorization
    std::vector<double> vectorizeDouble() const;
    void unvectorizeDouble(const std::vector<double>& v);
    std::vector<ComponentMeta> componentLayout() const;

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
    T value{};

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
    TypedParameter<T>* raw();
    const TypedParameter<T>* raw() const;

private:
    std::string name_;
    TypedParameter<T>* raw_ = nullptr;
};

// ============================================================================
// EnumParam (non-template, implementation in ParameterSpace.cpp)
// ============================================================================

class EnumParam {
public:
    std::string value;
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
    EnumParameter* raw();
    const EnumParameter* raw() const;

private:
    std::string name_;
    EnumParameter* raw_ = nullptr;
};

// ============================================================================
// StructuredSpace<Derived>
// ============================================================================

template <typename Derived>
class StructuredSpace {
public:
    StructuredSpace() = default;
    virtual ~StructuredSpace() = default;
    StructuredSpace(const StructuredSpace&) = delete;
    StructuredSpace& operator=(const StructuredSpace&) = delete;
    StructuredSpace(StructuredSpace&&) noexcept = default;
    StructuredSpace& operator=(StructuredSpace&&) noexcept = default;

    template <typename Fn>
    void registerParams(Fn&& fn);
    void sync_to_space();
    void sync_from_space();
    bool rebind();

    ParameterSpace& space();
    const ParameterSpace& space() const;

    json toJson() const;
    void fromJson(const json& j);

    std::vector<double> vectorize() const;
    std::vector<double> vectorizeDouble() const;
    void unvectorize(const std::vector<double>& v);
    void unvectorizeDouble(const std::vector<double>& v);

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

// ============================================================================
// Structured parameter registration macro
// (must stay in the header: it generates for_each_param inside the user's
//  derived struct at the macro use site)
// ============================================================================

#define AUROX_STRUCTURED_PARAMS(...)                         \
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

} // namespace Space
} // namespace AuroX

// Template implementations (NumericTraits specializations, TypedParameter<T>,
// ParameterSpace template members, Param<T>, StructuredSpace<Derived>).
#include "ParameterSpace.ipp"
