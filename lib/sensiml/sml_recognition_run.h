#ifndef __SML_RECOGNITION_RUN_H__
#define __SML_RECOGNITION_RUN_H__
#include "kb.h"

class SensiML
{
    public:

    SensiML();

    int RunModel(short* data, int* model_number);
    void ResetModel(int model_number);
    void GetFeatures(int model_number, uint8_t* feature_length, uint8_t* feature_array);
};

#endif //__SML_RECOGNITION_RUN_H__
