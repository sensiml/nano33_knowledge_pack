#ifndef __MODEL_JSON_H__
#define __MODEL_JSON_H__

const char recognition_model_string_json[] = {"{\"NumModels\":1,\"ModelIndexes\":{\"0\":\"TestModel\"},\"ModelDescriptions\":[{\"Name\":\"TestModel\",\"ClassMaps\":{\"1\":0,\"2\":1,\"3\":2,\"4\":4,\"0\":\"Unknown\"},\"ModelType\":\"PME\",\"FeatureFunctions\":[\"AverageofMovementIntensity\",\"VarianceofMovementIntensity\",\"AverageSignalMagnitudeArea\"]}]}"};

int recognition_model_string_json_len = sizeof(recognition_model_string_json);

#endif /* __MODEL_JSON_H__ */
