#ifndef _FOC_MODULE_BASE_HPP
#define _FOC_MODULE_BASE_HPP

#include "foc_header.h"

// A module can represent a set of specific control laws
class ModuleBase
{
public:
    virtual void Preprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts)  {}; // will be called before estimator updates
    virtual void Postprocess(foc_state_input_t* in, foc_state_output_t* out, float Ts) {}; // will be called after estimator updated
    virtual ~ModuleBase(){};
};

#endif