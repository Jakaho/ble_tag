#include "classifier.h"
#include "LIS2DS12.h"
#include <string.h>
#include "accelerometer_data.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_power.h"
#include "accelerometer_data.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "lis2ds12.h"
#include "tag.h"
#include "classifier.h"


DecisionNode decision_nodes[] = {
// Threshold, feature, node(true), node(false), decision(only if leaf, otherwise -1).
// Behaviors: 0 = eating, 1 = laying, 2 = laying_ruminating, 3 = standing,
// 4 = standing_ruminating, walking = 5. 

    {40.7224,KURT_DYN_Z, 1, 52, -1}, // Node 0 (root)
    {73.3000, Q5_Y, 2, 25, -1},
    {-182.9750, Q5_X, 3, 12, -1},
    {-117.1333, MEAN_Y, 4, 5, -1},
    {-1,-1,-1,-1, 2},
    {37.5000, MAX_Y, 6, 9, -1},
    {5.6708, STD_Y, 7, 8, -1},
    {-1,-1,-1,-1, 4},
    {-1,-1,-1,-1, 3},
    {47.7250, Q5_Y, 10, 11, -1},
    {-1,-1,-1,-1, 3}, // Node 10
    {-1,-1,-1,-1, 3},
    {-61.1333, MEAN_X, 13, 20, -1},
    {-147.4250, Q95_X, 14, 17, -1},
    {-9.5000, MIN_Y, 15, 16, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 4},
    {-67.1750, Q5_Y, 18, 19, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 0},
    {123.2578, STD_X, 21, 22, -1}, // Node 20
    {-1,-1,-1,-1, 2},
    {-2.0583, MEAN_DYN_Y, 23, 24, -1},
    {-1,-1,-1,-1, 2},
    {-1,-1,-1,-1, 3},
    {11.2000, Q95_DYN_X, 26, 37, -1},
    {10.5476,STD_X, 27, 32, -1},
    {123.0000, MAX_Y, 28, 31, -1},
    {-188.8417, MEAN_X, 29, 30, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 4}, // Node 30
    {-1,-1,-1,-1, 3},
    {11.4899, STD_Y, 33, 34, -1},
    {-1,-1,-1,-1, 2},
    {40.0512, KURT_DYN_X, 35, 36, -1},
    {-1,-1,-1,-1, 5},
    {-1,-1,-1,-1, 0},
    {43.8482, STD_Y, 38, 45, -1},
    {5.2479, SKEW_DYN_Z, 39, 42, -1},
    {1.9917, MEAN_DYN_Y, 40, 41, -1},
    {-1,-1,-1,-1, 0}, // Node 40
    {-1,-1,-1,-1, 0},
    {-6.0815, SKEW_DYN_X, 43, 44, -1},
    {-1,-1,-1,-1, 0},
    {-1,-1,-1,-1, 3},
    {-1.7828, SKEW_DYN_X, 46, 49, -1},
    {217.7750, Q95_Y, 47, 48, -1},
    {-1,-1,-1,-1, 5},
    {-1,-1,-1,-1, 0},
    {-0.1498, SKEW_DYN_X, 50, 51, -1},
    {-1,-1,-1,-1, 3}, // Node 50
    {-1,-1,-1,-1, 0},
    {-7.1526, SKEW_DYN_X, 53, 82, -1},
    {-174.0250, Q5_X, 54, 67, -1},
    {-8.1000, Q5_DYN_X, 55, 60, -1},
    {6.9816, SKEW_DYN_Z, 56, 57, -1},
    {-1,-1,-1,-1, 5},
    {6.5500, Q95_DYN_X, 58, 59, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 2},
    {43.3602, KURT_DYN_Z, 61, 64, -1}, // Node 60
    {-215.0750, Q5_X, 62, 63, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 0},
    {-7.3601, SKEW_DYN_X, 65, 66, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 3},
    {38.5000, MIN_Y, 68, 75, -1},
    {-14.5000, MAX_Y, 69, 72, -1},
    {-132.9833, MEAN_X, 70, 71, -1},
    {-1,-1,-1,-1, 3}, // Node 70
    {-1,-1,-1,-1, 1},
    {8.9750, Q5_Y, 73, 74, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 3},
    {73.5000, MIN_Y, 76, 79, -1},
    {-147.9250, Q95_X, 77, 78, -1},
    {-1,-1,-1,-1, 3},
    {-1,-1,-1,-1, 4},
    {-137.9750, Q95_X, 80, 81, -1},
    {-1,-1,-1,-1, 3}, // Node 80
    {-1,-1,-1,-1, 2},
    {-143.0250, Q5_X, 83, 98, -1},
    {-4.5000, MIN_Y, 84, 91, -1},
    {5.4711, STD_Y, 85, 88, -1},
    {-17.5500, Q5_Y, 86, 87, -1},
    {-1,-1,-1,-1, 2},
    {-1,-1,-1,-1, 4},
    {-15.0250, Q5_DYN_Z, 89, 90, -1},
    {-1,-1,-1,-1, 0},
    {-1,-1,-1,-1, 3}, // Node 90
    {63.5000, MIN_Y, 92, 95, -1},
    {10.8779, STD_Y, 93, 94, -1},
    {-1,-1,-1,-1, 4},
    {-1,-1,-1,-1, 3},
    {-155.5000, Q5_X, 96, 97, -1},
    {-1,-1,-1,-1, 4},
    {-1,-1,-1,-1, 2},
    {55.2307, KURT_DYN_Z, 99, 106, -1},
    {-131.4083, MEAN_X, 100, 103, -1},
    {59.4417, MEAN_Y, 101, 102, -1}, // Node 100
    {-1,-1,-1,-1, 4},
    {-1,-1,-1,-1, 2},
    {-7.0250, Q5_DYN_X, 104, 105, -1},
    {-1,-1,-1,-1, 2},
    {-1,-1,-1,-1, 2},
    {8.5000, MAX_Y, 107, 110, -1},
    {4.5335, STD_Y, 108, 109, -1},
    {-1,-1,-1,-1, 1},
    {-1,-1,-1,-1, 1},
    {62.5000, MIN_Y, 111, 112, -1}, // Node 110
    {-1,-1,-1,-1, 4},
    {-1,-1,-1,-1, 2}
};


void separate_axes(float x_data[DATA_LENGTH], float y_data[DATA_LENGTH], float z_data[DATA_LENGTH], int16_t data[3 * DATA_LENGTH]) {

    // Separate the data into x, y, z components and convert to float
    for (size_t i = 0, j = 0; i < 3 * DATA_LENGTH; i += 3, j++) {
        x_data[j] = (float)data[i];
        y_data[j] = (float)data[i + 1];
        z_data[j] = (float)data[i + 2];
    }
}


float calculate_mean(float* data, size_t length) {
    float sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum / length;
}

float calculate_max(float* data, size_t length) {
    float current_max = data[0]; // start with the first element
    for (size_t i = 1; i < length; ++i) {
        if (data[i] > current_max) {
            current_max = data[i];
        }
    }
    return current_max;
}

float calculate_min(float* data, size_t length) {
    float current_min = data[0]; // start with the first element
    for (size_t i = 1; i < length; ++i) {
        if (data[i] < current_min) {
            current_min = data[i];
        }
    }
    return current_min;
}

float calculate_variance(float* data, size_t length, float mean) {
    float variance = 0.0;
    for (size_t i = 0; i < length; ++i) {
        variance += (data[i] - mean) * (data[i] - mean);
    }
    return variance / length;
}

float calculate_standard_deviation(float* data, size_t length) {
    float mean = calculate_mean(data, length);
    float variance = calculate_variance(data, length, mean);
    return sqrt(variance);
}

int partition(float arr[], int low, int high) {
    float pivot = arr[high];
    int i = (low - 1);
    for (int j = low; j < high; j++) {
        if (arr[j] <= pivot) {
            i++;
            float temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }
    float temp = arr[i + 1];
    arr[i + 1] = arr[high];
    arr[high] = temp;
    return (i + 1);
}

float quickSelect(float arr[], int low, int high, int k) {
    if (k > 0 && k <= high - low + 1) {
        int index = partition(arr, low, high);

        if (index - low == k - 1)
            return arr[index];
        else if (index - low > k - 1)
            return quickSelect(arr, low, index - 1, k);
        else
            return quickSelect(arr, index + 1, high, k - index + low - 1);
    }
    return -1;  // Return an error value
}


float calculate_kurtosis(float* data, size_t length) {
    if (length < 4) return -1.0;  // Error handling, as kurtosis calculation requires at least 4 data points

    float mean = calculate_mean(data, length);
    float variance = calculate_variance(data, length, mean);
    float standard_deviation = sqrt(variance);

    float kurtosis = 0.0;
    for (size_t i = 0; i < length; i++) {
        kurtosis += pow((data[i] - mean) / standard_deviation, 4);
    }

    kurtosis = (kurtosis * length * (length + 1) / ((length - 1) * (length - 2) * (length - 3))) -
               (3 * pow(length - 1, 2) / ((length - 2) * (length - 3)));

    return kurtosis;
}

float calculate_skewness(float* data, size_t length) {
    if (length < 3) return -1.0;  // Error handling, as skewness calculation requires at least 3 data points

    float mean = calculate_mean(data, length);
    float variance = calculate_variance(data, length, mean);
    float standard_deviation = sqrt(variance);

    float skewness = 0.0;
    for (size_t i = 0; i < length; i++) {
        skewness += pow((data[i] - mean) / standard_deviation, 3);
    }

    skewness *= (float)length / ((length - 1) * (length - 2));

    return skewness;
}


void calculate_dynamic_components(float* dyn_components_x, float* dyn_components_y, float* dyn_components_z, float* x_data, float* y_data, float* z_data, size_t length) {
    for (size_t i = 0; i < length - 1; i++) {
        dyn_components_x[i] = x_data[i + 1] - x_data[i];
        dyn_components_y[i] = y_data[i + 1] - y_data[i];
        dyn_components_z[i] = z_data[i + 1] - z_data[i];
    }
}

AccFeatures calculate_features(float* x_data, float* y_data, float* z_data, size_t length) {
    AccFeatures features;
    const int Q5 = 3;  // For DATA_LENGTH = 60, 5th percentile is at index 3 when array is zero-indexed
    const int Q95 = 57; // For DATA_LENGTH = 60, 95th percentile is at index 57

      // Prepare dynamic component arrays
    float dyn_x[length - 1], dyn_y[length - 1], dyn_z[length - 1];
    calculate_dynamic_components(dyn_x, dyn_y, dyn_z, x_data, y_data, z_data, length);

    // Calculate raw features
    features.mean_x = calculate_mean(x_data, length);
    features.mean_y = calculate_mean(y_data, length);
    features.mean_z = calculate_mean(z_data, length);

    features.max_x = calculate_max(x_data, length);
    features.max_y = calculate_max(y_data, length);
    features.max_z = calculate_max(z_data, length);

    features.min_x = calculate_min(x_data, length);
    features.min_y = calculate_min(y_data, length);
    features.min_z = calculate_min(z_data, length);

    features.std_x = calculate_standard_deviation(x_data, length);
    features.std_y = calculate_standard_deviation(y_data, length);
    features.std_z = calculate_standard_deviation(z_data, length);

    // Using quickSelect to find quantiles, array must be sorted or copied to not disturb the original
    float x_sorted[DATA_LENGTH], y_sorted[DATA_LENGTH], z_sorted[DATA_LENGTH];
    memcpy(x_sorted, x_data, length * sizeof(float));
    memcpy(y_sorted, y_data, length * sizeof(float));
    memcpy(z_sorted, z_data, length * sizeof(float));

    features.Q5_x = quickSelect(x_sorted, 0, length - 1, Q5);
    features.Q5_y = quickSelect(y_sorted, 0, length - 1, Q5);
    features.Q5_z = quickSelect(z_sorted, 0, length - 1, Q5);

    features.Q95_x = quickSelect(x_sorted, 0, length - 1, Q95);
    features.Q95_y = quickSelect(y_sorted, 0, length - 1, Q95);
    features.Q95_z = quickSelect(z_sorted, 0, length - 1, Q95);

    features.kurt_dyn_x = calculate_kurtosis(dyn_x, length - 1);
    features.kurt_dyn_y = calculate_kurtosis(dyn_y, length - 1);
    features.kurt_dyn_z = calculate_kurtosis(dyn_z, length - 1);

    features.skew_dyn_x = calculate_skewness(dyn_x, length - 1);
    features.skew_dyn_y = calculate_skewness(dyn_y, length - 1);
    features.skew_dyn_z = calculate_skewness(dyn_z, length - 1);

    // Use quickSelect to compute dynamic quantiles if needed
    float sorted_dyn_x[length - 1], sorted_dyn_y[length - 1], sorted_dyn_z[length - 1];
    memcpy(sorted_dyn_x, dyn_x, (length - 1) * sizeof(float));
    memcpy(sorted_dyn_y, dyn_y, (length - 1) * sizeof(float));
    memcpy(sorted_dyn_z, dyn_z, (length - 1) * sizeof(float));

    features.Q5_dyn_x = quickSelect(sorted_dyn_x, 0, length - 2, (length - 1) / 20); // 5th percentile
    features.Q95_dyn_x = quickSelect(sorted_dyn_x, 0, length - 2, (length - 1) * 19 / 20); // 95th percentile
    features.Q5_dyn_y = quickSelect(sorted_dyn_y, 0, length - 2, (length - 1) / 20);
    features.Q95_dyn_y = quickSelect(sorted_dyn_y, 0, length - 2, (length - 1) * 19 / 20);
    features.Q5_dyn_z = quickSelect(sorted_dyn_z, 0, length - 2, (length - 1) / 20);
    features.Q95_dyn_z = quickSelect(sorted_dyn_z, 0, length - 2, (length - 1) * 19 / 20);
    features.mean_dyn_x = calculate_mean(dyn_x, length);
    features.mean_dyn_y = calculate_mean(dyn_y, length);
    features.mean_dyn_z = calculate_mean(dyn_z, length);

    // Return the populated features structure
    return features;
}



int process_classifier(const int16_t* data) {
    int decision = -1;
    // Ensure arrays are the correct size
    float x_data[DATA_LENGTH], y_data[DATA_LENGTH], z_data[DATA_LENGTH];

    // Call separate_axes to populate x_data, y_data, and z_data from sensor_data
    separate_axes(x_data, y_data, z_data, data);

    // Call calculate_features to process the data and store the results
    AccFeatures features = calculate_features(x_data, y_data, z_data, DATA_LENGTH);

    // Continue with classifier logic using the 'features' structure
    // For example, you might log these features, make decisions based on them, etc.

    decision = evaluate_decision(&features);

    return decision;

}

float get_feature_value(const AccFeatures* features, FeatureIndex index) {
    switch (index) {
        case MEAN_X: return features->mean_x;
        case MEAN_Y: return features->mean_y;
        case MEAN_Z: return features->mean_z;
        case MAX_X: return features->max_x;
        case MAX_Y: return features->max_y;
        case MAX_Z: return features->max_z;
        case MIN_X: return features->min_x;
        case MIN_Y: return features->min_y;
        case MIN_Z: return features->min_z;
        case STD_X: return features->std_x;
        case STD_Y: return features->std_y;
        case STD_Z: return features->std_z;
        case Q5_X: return features->Q5_x;
        case Q5_Y: return features->Q5_y;
        case Q5_Z: return features->Q5_z;
        case Q95_X: return features->Q95_x;
        case Q95_Y: return features->Q95_y;
        case Q95_Z: return features->Q95_z;
        case KURT_DYN_X: return features->kurt_dyn_x;
        case KURT_DYN_Y: return features->kurt_dyn_y;
        case KURT_DYN_Z: return features->kurt_dyn_z;
        case SKEW_DYN_X: return features->skew_dyn_x;
        case SKEW_DYN_Y: return features->skew_dyn_y;
        case SKEW_DYN_Z: return features->skew_dyn_z;
        case Q5_DYN_X: return features->Q5_dyn_x;
        case Q5_DYN_Y: return features->Q5_dyn_y;
        case Q5_DYN_Z: return features->Q5_dyn_z;
        case Q95_DYN_X: return features->Q95_dyn_x;
        case Q95_DYN_Y: return features->Q95_dyn_y;
        case Q95_DYN_Z: return features->Q95_dyn_z;
        case MEAN_DYN_X: return features->mean_dyn_x;
        case MEAN_DYN_Y: return features->mean_dyn_y;
        case MEAN_DYN_Z: return features->mean_dyn_z;
        default: return 0.0; // Default case
    }
}

int evaluate_decision(const AccFeatures* features) {
    int current_node = 0;
    while (true) {
        const DecisionNode* node = &decision_nodes[current_node];
        if (node->decision != -1) {
            //NRF_LOG_INFO("SISTA %d", current_node);
            return node->decision; // Return the decision if it's a leaf node
        }

        // Retrieve the current feature value based on the feature index
        float feature_value = get_feature_value(features, node->feature_index);

        // Determine the next node
        if (feature_value <= node->threshold) {
            current_node = node->next_node_if_true;
        } else {
            current_node = node->next_node_if_false;
        }
    }
    return -1; // Should not be reached; indicates an error
}



