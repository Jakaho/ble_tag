#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#define DATA_LENGTH 60  // Define a constant for the length of your arrays


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

// Define the structure for accelerometer features
typedef struct {
    float mean_x;
    float mean_y;
    float mean_z;

    float max_x;
    float max_y;
    float max_z;

    float min_x;
    float min_y;
    float min_z;

    float std_x;
    float std_y;
    float std_z;

    float Q5_x;
    float Q5_y;
    float Q5_z;

    float Q95_x;
    float Q95_y;
    float Q95_z;

    float kurt_dyn_x;
    float kurt_dyn_y;
    float kurt_dyn_z;

    float skew_dyn_x;
    float skew_dyn_y;
    float skew_dyn_z;

    float Q5_dyn_x;
    float Q5_dyn_y;
    float Q5_dyn_z;

    float Q95_dyn_x;
    float Q95_dyn_y;
    float Q95_dyn_z;

    float mean_dyn_x;
    float mean_dyn_y;
    float mean_dyn_z;


    // Additional features can be added here
} AccFeatures;

typedef struct {
    float threshold;           // The threshold value for the decision
    int feature_index;         // Index to the specific feature in the feature array
    int next_node_if_true;     // Index of the next node if the condition is true
    int next_node_if_false;    // Index of the next node if the condition is false
    int decision;              // Decision value (used only for leaf nodes, -1 for others)
} DecisionNode;

typedef enum {
    MEAN_X,
    MEAN_Y,
    MEAN_Z,
    MAX_X,
    MAX_Y,
    MAX_Z,
    MIN_X,
    MIN_Y,
    MIN_Z,
    STD_X,
    STD_Y,
    STD_Z,
    Q5_X,
    Q5_Y,
    Q5_Z,
    Q95_X,
    Q95_Y,
    Q95_Z,
    KURT_DYN_X,
    KURT_DYN_Y,
    KURT_DYN_Z,
    SKEW_DYN_X,
    SKEW_DYN_Y,
    SKEW_DYN_Z,
    Q5_DYN_X,
    Q5_DYN_Y,
    Q5_DYN_Z,
    Q95_DYN_X,
    Q95_DYN_Y,
    Q95_DYN_Z,
    MEAN_DYN_X,
    MEAN_DYN_Y,
    MEAN_DYN_Z,

} FeatureIndex;


// Function prototypes
void separate_axes(float x_data[DATA_LENGTH], float y_data[DATA_LENGTH], float z_data[DATA_LENGTH], uint8_t data[3 * DATA_LENGTH]);
float calculate_mean(float* data, size_t length);
float calculate_max(float* data, size_t length);
float calculate_min(float* data, size_t length);
float calculate_variance(float* data, size_t length, float mean);
float calculate_standard_deviation(float* data, size_t length);
float calculate_kurtosis(float* data, size_t length);
float calculate_skewness(float* data, size_t length);
float quickSelect(float arr[], int low, int high, int k);

void calculate_dynamic_components(float* dyn_components_x, float* dyn_components_y, float* dyn_components_z, float* x_data, float* y_data, float* z_data, size_t length);
AccFeatures calculate_features(float* x_data, float* y_data, float* z_data, size_t length);
float get_feature_value(const AccFeatures* features, FeatureIndex index);
int evaluate_decision(const AccFeatures* features);
int process_classifier(const uint8_t* data);



#endif // CLASSIFIER_H
