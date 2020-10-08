#include "sensor_config.h"
#include "kb.h"



int sml_recognition_run(signed short *data)
{
    int num_sensors = 1;
    int ret;
#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG

        ret  = kb_run_model((SENSOR_DATA_T*) data, num_sensors, KB_MODEL_jp1_rank_0_INDEX);
        if (ret >= 0)
        {
            sml_output_results(KB_MODEL_jp1_rank_0_INDEX, ret);
            kb_reset_model(0);
        };

#endif //ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG


#if ENABLE_AUDIO
    //This is not enabled in the model provided. Generate a model with Audio on SensiML Cloud.
    return -1;

#endif //ENABLE_AUDIO

}
