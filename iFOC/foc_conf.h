#ifndef _IFOC_CONF_H
#define _IFOC_CONF_H

// If use external LED for indicating driver states, deannotate it and call FOC.SetIndicator(GPIOBase &base)
#define FOC_USING_INDICATOR

// If use temp probes, you should call FOC.Attach___TempProbe(float *ptr)
// ___ can be MCU, Motor, or FET. The float ptr points to a temperature variable in degree.
// #define FOC_USING_TEMP_PROBE 

// If use auxiliary estimator(AuxEstimator), deannotate it and call FOC.AttachAuxEstimator<EstimatorBase>()
// #define FOC_USING_AUX_ESTIMATOR

// If use extra module, deannotate it and call FOC.AppendModule<ModuleBase>()
// #define FOC_USING_EXTRA_MODULE

#endif