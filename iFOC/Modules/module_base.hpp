#ifndef _FOC_MODULE_BASE_HPP
#define _FOC_MODULE_BASE_HPP

#include "foc_header.h"

// A module can represent a set of specific control laws
class ModuleBase
{
public:
    // @return: true if this module allows other modules to work, false if monopolized
    virtual bool Preprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts)  { return true; }; // will be called before estimator updates
    // @return: true if this module allows other modules to work, false if monopolized
    virtual bool Postprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts) { return true; }; // will be called after estimator updated
    virtual ~ModuleBase() = default;
};

template<typename T>
concept ModuleImpl = std::is_base_of<ModuleBase, T>::value;

#endif