/*
 * welford.h
 *
 * Created on: Apr 20, 2025
 * Author: hp
 */

#ifndef WELFORD_H_
#define WELFORD_H_

// Define the window size
#define WINDOW_SIZE 10

// Welford structure to store the mean, M2, and the data in the sliding window
typedef struct {
    double values[WINDOW_SIZE];  // Store the most recent N values
    double mean;                 // Mean of the window
    double m2;                   // Sum of squared differences from the mean
    int n;                       // Number of values processed
    int idx;                     // Index for the circular buffer
} Welford;

// Initialize the Welford struct
void welford_init(Welford *w);

// Update the Welford model with a new value
void welford_update(Welford *w, double x);

// Calculate the standard deviation of the values in the window
double welford_stddev(Welford *w);

// Check if a value is an anomaly based on the Welford model
int is_welford_anomaly(Welford *w, double value, double threshold);

#endif /* WELFORD_H_ */
