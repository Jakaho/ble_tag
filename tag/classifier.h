#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#define DATA_LENGTH 60  // Define a constant for the length of your arrays


#include <stdint.h>
#include <stddef.h>
#include <math.h>


// Define the structure for accelerometer features
typedef struct {
    float mean_x;
    float mean_y;
    float mean_z;

    int16_t max_x;
    int16_t max_y;
    int16_t max_z;

    int16_t min_x;
    int16_t min_y;
    int16_t min_z;

    float std_x;
    float std_y;
    float std_z;

    int16_t Q5_x;
    int16_t Q5_y;
    int16_t Q5_z;

    int16_t Q95_x;
    int16_t Q95_y;
    int16_t Q95_z;

    int16_t Q95_dyn_x;
    int16_t Q95_dyn_y;
    int16_t Q95_dyn_z;
    
    int16_t Q5_dyn_x;
    int16_t Q5_dyn_y;
    int16_t Q5_dyn_z;
    
    float kurt_dyn_x;
    float kurt_dyn_y;
    float kurt_dyn_z;

    float skew_dyn_x;
    float skew_dyn_y;
    float skew_dyn_z;


    // Additional features can be added here
} AccFeatures;

// Function prototypes
void separate_axes(int16_t x_data[DATA_LENGTH], int16_t y_data[DATA_LENGTH], int16_t z_data[DATA_LENGTH]);
float calculate_mean(int16_t* data, size_t length);
int calculate_max(int16_t* data, size_t length);
int calculate_min(int16_t* data, size_t length);
void swap(int16_t* a, int16_t* b);
int partition(int16_t arr[], int low, int high);
int16_t quickSelect(int16_t arr[], int low, int high, int k);
void calculate_dynamic_components(int16_t* dyn_components_x, int16_t* dyn_components_y, int16_t* dyn_components_z, int16_t* x_data, int16_t* y_data, int16_t* z_data, size_t length);
AccFeatures calculate_features(int16_t* x_data, int16_t* y_data, int16_t* z_data, int16_t* dyn_components_x, int16_t* dyn_components_y, int16_t* dyn_components_z, size_t length);
void process_classifier(void);
float calculate_variance(int16_t* data, size_t length, float mean);
float calculate_kurtosis(int16_t* data, size_t length);
float calculate_skewness(int16_t* data, size_t length);

#endif // CLASSIFIER_H
