#include "sml_recognition_run.h"

    SensiML::SensiML()
    {
        kb_model_init();
    }

    int SensiML::RunModel(short* data, int *model_number)
    {
        int ret = 0;
        ret = kb_run_model((SENSOR_DATA_T*)data, 1, *model_number);
        if(ret >=0)
        {
            return ret;
        }
        return -1;

    }

    void SensiML::ResetModel(int model_number)
    {
        kb_reset_model(model_number);
    }

    void SensiML::GetFeatures(int model_number, uint8_t* feature_length, uint8_t* feature_array)
    {
        kb_get_feature_vector(model_number, feature_array, feature_length );
    }
