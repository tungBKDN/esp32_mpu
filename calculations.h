#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace std;

double calculate_mean(const vector<double>& v) {
    double sum = accumulate(v.begin(), v.end(), 0.0);
    return sum / v.size();
}

// Function to calculate standard deviation
double calculate_std(const vector<double>& v, double mean) {
    double sq_sum = inner_product(v.begin(), v.end(), v.begin(), 0.0);
    return sqrt(sq_sum / v.size() - mean * mean);
}

// Function to standardize features
vector<double> standardize_features(const vector<double>& features, const vector<double>& means, const vector<double>& std_devs) {
    vector<double> standardized_features(features.size());
    for (size_t i = 0; i < features.size(); ++i) {
        standardized_features[i] = (features[i] - means[i]) / std_devs[i];
    }
    return standardized_features;
}

// Function to calculate median
double median(const vector<double>& v) { // Changed to const reference
    size_t n = v.size();
    vector<double> copy_v = v; // Create a copy to avoid modifying the input
    nth_element(copy_v.begin(), copy_v.begin() + n / 2, copy_v.end()); // O(n) median selection
    if (n % 2 == 0) {
        nth_element(copy_v.begin(), copy_v.begin() + n / 2 - 1, copy_v.end());
        return (copy_v[n/2 - 1] + copy_v[n/2]) / 2;
    }
    return copy_v[n/2];
}

// Helper function to calculate MAD (Median Absolute Deviation)
double mad(const vector<double>& ds) {
    double med = median(const_cast<vector<double>&>(ds));
    vector<double> abs_devs(ds.size());
    transform(ds.begin(), ds.end(), abs_devs.begin(), [&med](double d) { return abs(d - med); });
    return median(abs_devs);
}

// Function to extract statistical features from raw signals
vector<double> stat_features_for_raw_signal(const vector<double>& ds) {
    if (ds.empty()) return {0, 0, 0, 0, 0};

    double max_ft = *max_element(ds.begin(), ds.end());
    double min_ft = *min_element(ds.begin(), ds.end());
    double sum = accumulate(ds.begin(), ds.end(), 0.0);
    double mean = sum / ds.size();
    
    double sq_sum = inner_product(ds.begin(), ds.end(), ds.begin(), 0.0);
    double var_ft = (sq_sum / ds.size()) - (mean * mean);
    double std_ft = sqrt(var_ft);

    vector<double> sorted_ds = ds;
    sort(sorted_ds.begin(), sorted_ds.end());

    // Correctly calculate Q1 and Q3
    double q1 = median(vector<double>(sorted_ds.begin(), sorted_ds.begin() + sorted_ds.size() / 2));
    double q3 = median(vector<double>(sorted_ds.begin() + (sorted_ds.size() + 1) / 2, sorted_ds.end()));
    double iqr_ft = q3 - q1;

    double mad_ft = mad(ds);

    return {max_ft, std_ft, var_ft, iqr_ft, mad_ft};
}

// Function to extract statistical features from jerk signals
vector<double> stat_features_for_jerk_signal(const vector<double>& ds) {
    if (ds.empty()) return {0, 0, 0, 0, 0, 0};

    double max_ft = *max_element(ds.begin(), ds.end());
    double min_ft = *min_element(ds.begin(), ds.end());
    double range_ft = max_ft - min_ft;
    double sum = accumulate(ds.begin(), ds.end(), 0.0);
    double mean = sum / ds.size();
    
    double sq_sum = inner_product(ds.begin(), ds.end(), ds.begin(), 0.0);
    double var_ft = (sq_sum / ds.size()) - (mean * mean);
    double std_ft = sqrt(var_ft);

    double mad_ft = mad(ds);

    return {max_ft, min_ft, range_ft, std_ft, var_ft, mad_ft};
}

// Function to create feature vector
vector<double> make_feature_vector(const vector<vector<double>>& data) {
    vector<double> acc_re{}, gyro_re{};

    for (const auto& row : data) {
        double acc_magnitude = sqrt(pow(row[1], 2) + pow(row[2], 2) + pow(row[3], 2));
        double gyro_magnitude = sqrt(pow(row[4], 2) + pow(row[5], 2) + pow(row[6], 2));
        acc_re.push_back(acc_magnitude);
        gyro_re.push_back(gyro_magnitude);
    }
    
    vector<double> features_acc_re{0};
    features_acc_re = stat_features_for_raw_signal(acc_re);
    vector<double> features_gyro_re{0};
    features_gyro_re = stat_features_for_raw_signal(gyro_re);

    vector<double> acc_re_diff{0}, gyro_re_diff{0};
    for (size_t i = 1; i < acc_re.size(); ++i) {
        acc_re_diff.push_back(acc_re[i] - acc_re[i-1]);
        gyro_re_diff.push_back(gyro_re[i] - gyro_re[i-1]);
    }

    vector<double> features_acc_re_jerk = stat_features_for_jerk_signal(acc_re_diff);
    vector<double> features_gyro_re_jerk = stat_features_for_jerk_signal(gyro_re_diff);

    features_acc_re.insert(features_acc_re.end(), features_acc_re_jerk.begin(), features_acc_re_jerk.end());
    features_acc_re.insert(features_acc_re.end(), features_gyro_re.begin(), features_gyro_re.end());
    features_acc_re.insert(features_acc_re.end(), features_gyro_re_jerk.begin(), features_gyro_re_jerk.end());
    
    return features_acc_re;
}

// Function to parse raw CSV data into a vector of vectors
vector<vector<double>> parse_csv(const string& csv_content) {
    vector<vector<double>> data;
    istringstream sstream(csv_content);
    string line;

    // Ignore the first line (header)
    getline(sstream, line);

    while (getline(sstream, line)) {
        istringstream line_stream(line);
        string cell;
        vector<double> row;
        try {
            while (getline(line_stream, cell, ';')) {
                row.push_back(stod(cell));
            }
            data.push_back(row);
        }
        catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument: " << e.what() << std::endl;
        } 
    }

    return data;
}

// Function to calculate the RBF kernel
double rbf_kernel(const vector<double>& x, const vector<double>& y, double gamma) {
    double sum = 0.0;
    for (size_t i = 0; i < x.size(); ++i) {
        sum += pow(x[i] - y[i], 2);
    }
    return exp(-gamma * sum);
}

// Function to predict using the SVM model
int svm_predict(const vector<vector<double>>& support_vectors, const vector<double>& dual_coefficients, double intercept, double gamma, const vector<double>& features) {
    double result = 0.0;
    for (size_t i = 0; i < support_vectors.size(); ++i) {
        result += dual_coefficients[i] * rbf_kernel(support_vectors[i], features, gamma);
    }
    result += intercept;
    return result > 0 ? 1 : -1;
}

int predict(const string& csv_content, const vector<vector<double>>& support_vectors, const vector<double>& dual_coefficients, double intercept, double gamma, const vector<double>& means, const vector<double>& scales) {
    vector<vector<double>> data = parse_csv(csv_content);
    vector<double> feature_vector = make_feature_vector(data);
    vector<double> standardized_features = standardize_features(feature_vector, means, scales);
    return svm_predict(support_vectors, dual_coefficients, intercept, gamma, standardized_features);
}

#endif // CALCULATIONS_H