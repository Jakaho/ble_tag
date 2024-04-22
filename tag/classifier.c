#include "classifier.h"
#include "LIS2DS12.h"


void separate_axes(int16_t x_data[DATA_LENGTH], int16_t y_data[DATA_LENGTH], int16_t z_data[DATA_LENGTH]) {
    int16_t data[] = {-10,77,304,3,54,373,6,46,488,77,-114,511,-113,181,511,-9,-18,209,-68,-81,242,41,-88,66,-13,
                      -55,135,-20,-19,169,-46,27,212,-64,51,275,-52,112,356,-51,208,449,-66,439,479,-131,498,284,-148,511,93,
                      -117,445,222,415,-512,-288,160,505,194,-125,389,384,36,201,231,-27,78,245,0,93,237,-6,79,245,2,83,238,-27,
                      75,240,-40,88,236,-3,96,235,-28,116,218,-37,148,209,-35,148,221,-15,159,288,-19,132,337,22,72,462,-37,-85,
                      511,-72,167,511,-127,-2,59,101,-282,272,52,-88,141,97,-22,196,75,28,275,68,77,342,57,132,406,37,240,500,-60,
                      495,466,-130,511,221,-159,511,123,-227,148,511,145,511,-196,101,-148,163,7,60,232,-30,80,246,-21,69,254,-12,
                      62,241,-21,54,253,-16,50,246,-19,48,245,-14,57,244,-12,57,254};

    // Separate the data into x, y, z components
    for (size_t i = 0, j = 0; i < sizeof(data) / sizeof(data[0]); i += 3, j++) {
        x_data[j] = data[i];
        y_data[j] = data[i + 1];
        z_data[j] = data[i + 2];
    }
}

float calculate_mean(int16_t* data, size_t length) {
    float sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum / length;
}

float calculate_variance(int16_t* data, size_t length, float mean) {
    float variance = 0.0;
    for (size_t i = 0; i < length; ++i) {
        variance += pow(data[i] - mean, 2);
    }
    return variance / length;

}

float calculate_standard_deviation(int16_t* data, size_t length) {

  float mean = calculate_mean(data, length);
  float variance = calculate_variance(data, length, mean);

  float standard_dev = sqrt(variance);

  return standard_dev;
}


int calculate_max(int16_t* data, size_t length){
    int16_t current_max = 0;
    for (size_t i = 0; i < length; ++i){
        if(data[i] > current_max){
        current_max = data[i];
        }
    }
    return current_max;
}

int calculate_min(int16_t* data, size_t length){
    int16_t current_min = 1000;
    for (size_t i = 0; i < length; ++i){
        if(data[i] < current_min){
        current_min = data[i];
        }
    }
    return current_min;
}
void swap(int16_t* a, int16_t* b) {
    int16_t temp = *a;
    *a = *b;
    *b = temp;
}

int partition(int16_t arr[], int low, int high) {
    int16_t pivot = arr[high];
    int i = (low - 1);

    for (int j = low; j <= high - 1; j++) {
        if (arr[j] < pivot) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[high]);
    return (i + 1);
}

int16_t quickSelect(int16_t arr[], int low, int high, int k) {
    if (k > 0 && k <= high - low + 1) {
        int index = partition(arr, low, high);

        if (index - low == k - 1)
            return arr[index];
        if (index - low > k - 1)
            return quickSelect(arr, low, index - 1, k);

        return quickSelect(arr, index + 1, high, k - index + low - 1);
    }

    return INT16_MAX;
}

float calculate_kurtosis(int16_t* data, size_t length){
  float kurtosis = 0.0;
  float mean = calculate_mean(data, length);
  float variance = calculate_variance(data, length, mean);

  for (size_t i =0; i < length; ++i){
      kurtosis += pow((data[i] - mean) / sqrt(variance), 4);
  }
  
  kurtosis *= length * (length -1) / ((length -1) * (length -2) * (length -3));
  kurtosis -= 3* pow((length-1),2) / ((length-2) * (length -3));

  return kurtosis;
}


float calculate_skewness(int16_t* data, size_t length){
  float skewness = 0.0;
  float mean = calculate_mean(data, length);
  float variance = calculate_variance(data, length, mean);

  for (size_t i = 0; i < length; ++i){
      skewness += pow((data[i] - mean ) / sqrt(variance), 3);
  }

  skewness *= length / ((length - 1) * (length -2));
  return skewness;
}

void calculate_dynamic_components(int16_t* dyn_components_x, int16_t* dyn_components_y, int16_t* dyn_components_z, int16_t* x_data, int16_t* y_data, int16_t* z_data, size_t length) {
    if (length < 2) return;  // Not enough data

    for (size_t i = 0; i < length - 1; i++) {
        dyn_components_x[i] = (x_data[i + 1] - x_data[i]);
        dyn_components_y[i] = (y_data[i + 1] - y_data[i]);
        dyn_components_z[i] = (z_data[i + 1] - z_data[i]);
    }
}


AccFeatures calculate_features(int16_t* x_data, int16_t* y_data, int16_t* z_data, int16_t* dyn_components_x, int16_t* dyn_components_y, int16_t* dyn_components_z, size_t length) {
    AccFeatures features;
    int Q5 = 3;
    int Q95 = 58;

//----------RAW ACC FEATURES---------------
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

    features.Q5_x = quickSelect(x_data, 0, length, Q5);
    features.Q5_y = quickSelect(y_data, 0, length, Q5);
    features.Q5_z = quickSelect(z_data, 0, length, Q5);

    features.Q95_x = quickSelect(x_data, 0, length, Q95);
    features.Q95_y = quickSelect(y_data, 0, length, Q95);
    features.Q95_z = quickSelect(z_data, 0, length, Q95);

//----------DYNAMIC COMPONENT FEATURES---------------

    features.Q5_dyn_x = quickSelect(dyn_components_x, 0, length, Q5);
    features.Q5_dyn_y = quickSelect(dyn_components_y, 0, length, Q5);
    features.Q5_dyn_z = quickSelect(dyn_components_z, 0, length, Q5);

    features.Q95_dyn_x = quickSelect(dyn_components_x, 0, length, Q95);
    features.Q95_dyn_y = quickSelect(dyn_components_y, 0, length, Q95);
    features.Q95_dyn_z = quickSelect(dyn_components_z, 0, length, Q95);

    features.kurt_dyn_x = calculate_kurtosis(dyn_components_x, length);
    features.kurt_dyn_y = calculate_kurtosis(dyn_components_y, length);
    features.kurt_dyn_z = calculate_kurtosis(dyn_components_z, length);

    features.skew_dyn_x = calculate_skewness(dyn_components_x, length);
    features.skew_dyn_y = calculate_skewness(dyn_components_y, length);
    features.skew_dyn_z = calculate_skewness(dyn_components_z, length);



    // Calculate and assign more features as needed
    return features;
}

void process_classifier(void) {
    int16_t x_data[DATA_LENGTH], y_data[DATA_LENGTH], z_data[DATA_LENGTH];
    separate_axes(x_data, y_data, z_data);

    int16_t dyn_components_x[DATA_LENGTH - 1]; 
    int16_t dyn_components_y[DATA_LENGTH - 1]; 
    int16_t dyn_components_z[DATA_LENGTH - 1];  

    calculate_dynamic_components(dyn_components_x, dyn_components_y, dyn_components_z, x_data, y_data, z_data, DATA_LENGTH);


    AccFeatures features = calculate_features(x_data, y_data, z_data, dyn_components_x, dyn_components_y, dyn_components_z, DATA_LENGTH);
    


}

/*void simplified_decision_tree(AccFeatures features){

if (features.kurt_acz <= -0.0686) {
    if (features.Q5_y <= 0.5875) {
        if (features.Q5_x <= -0.4074) {
            if (features.mean_acc_y <= -2.4438) {
                // Leaf node 4: Decision: 2
            } else {
                if (features.Q5_y <= 0.2376) {
                    if (features.Q95_x <= -0.7272) {
                        // Leaf node 7: Decision: 3
                    } else {
                        // Leaf node 8: Decision: 3
                    }
                } else {
                    if (features.Q5_x <= -0.5844) {
                        // Leaf node 10: Decision: 0
                    } else {
                        // Leaf node 11: Decision: 3
                    }
                }
            }
        } else {
            if (features.mean_acc_x <= 1.1297) {
                if (features.Q95_x <= -0.3807) {
                    if (features.Q5_y <= -0.3410) {
                        // Leaf node 15: Decision: 0
                    } else {
                        // Leaf node 16: Decision: 4
                    }
                } else {
                    if (features.Q5_y <= -1.3342) {
                        // Leaf node 18: Decision: 3
                    } else {
                        // Leaf node 19: Decision: 0
                    }
                }
            } else {
                if (features.Q95_y <= 3.7592) {
                    if (features.Q95_ac_x <= 3.6630) {
                        // Leaf node 22: Decision: 2
                    } else {
                        // Leaf node 23: Decision: 2
                    }
                } else {
                    // Leaf node 24: Decision: 3
                }
            }
        }
    } else {
        if (features.Q95_ac_x <= -0.1423) {
            if (features.Q95_x <= -0.1990) {
                if (features.Q5_y <= 0.8772) {
                    if (features.min_acc_x <= -0.3969) {
                        // Leaf node 29: Decision: 3
                    } else {
                        // Leaf node 30: Decision: 4
                    }
                } else {
                    if (features.Q95_ac_x <= -0.1641) {
                        // Leaf node 32: Decision: 3
                    } else {
                        // Leaf node 33: Decision: 0
                    }
                }
            } else {
                if (features.std_acc_y <= 1.4615) {
                    if (features.kurt_acz <= -0.2981) {
                        if (features.skew_acx <= 1.9058) {
                            // Leaf node 38: Decision: 0
                        } else {
                            // Leaf node 39: Decision: 5
                        }
                    } else {
                        // Leaf node 41: Decision: 3
                    }
                } else {
                    if (features.skew_acx <= 1.0421) {
                        if (features.min_acc_y <= 0.1247) {
                            // Leaf node 45: Decision: 5
                        } else {
                            // Leaf node 46: Decision: 0
                        }
                    } else {
                        if (features.skew_acx <= 1.5506) {
                            // Leaf node 48: Decision: 3
                        } else {
                            // Leaf node 49: Decision: 0
                        }
                    }
                }
            }
        } else {
            // Remaining nodes...
        }
    }
} else {
    // Remaining nodes...
}


}*/


