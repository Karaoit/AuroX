#pragma once

// ----------- Space -------------

#if __has_include (<AuroX/Space/StateSpace.hpp>)
    #include "Space/StateSpace.hpp"
#endif

#if __has_include (<AuroX/Space/ActionSpace.hpp>)
    #include "Space/ActionSpace.hpp"
#endif

#if __has_include (<AuroX/Space/ObservationSpace.hpp>)
    #include "Space/ObservationSpace.hpp"
#endif

#if __has_include (<AuroX/Space/ParameterSpace.hpp>)
    #include "Space/ParameterSpace.hpp"
#endif

// ----------- Memory -------------

#if __has_include (<AuroX/Memory/RunHistory.hpp>)
    #include "Memory/RunHistory.hpp"
#endif

// ----------- Safety -------------

#if __has_include (<Safety/SafetyManager.hpp>)
    #include "Safety/SafetyManager.hpp"
#endif

// ----------- Runtime -------------

#if __has_include (<Runtime/Task.hpp>)
    #include "Runtime/Task.hpp"
#endif

#if __has_include (<Runtime/Runtime.hpp>)
    #include "Runtime/Runtime.hpp"
#endif

// ----------- Optimizer -------------
#if __has_include (<AuroX/Optimizer/Optimizer.hpp>)
    #include "AuroX/Optimizer/Optimizer.hpp"
#endif

#if __has_include (<AuroX/Optimizer/OptimizerSession.hpp>)
    #include "AuroX/Optimizer/OptimizerSession.hpp"
#endif

// ----------- Evaluation -------------
#if __has_include (<AuroX/Evaluation/Evaluator.hpp>)
    #include "Evaluation/Evaluator.hpp"
#endif

// ----------- Dynamics -------------
#if __has_include (<AuroX/Dynamics/Dynamics.hpp>)
    #include "Dynamics/Dynamics.hpp"
#endif