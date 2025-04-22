#include <math.h>
#include <stdio.h>
#include "welford.h"

// Define the window size

// Initialize the Welford struct
void welford_init(Welford *w) {
    w->mean = 0.0;
    w->m2 = 0.0;
    w->n = 0;
    w->idx = 0;
    // Initialize the sliding window values
    for (int i = 0; i < WINDOW_SIZE; i++) {
        w->values[i] = 0.0;
    }
}

// Update the Welford model with a new value
void welford_update(Welford *w, double x) {
    // Store the new value in the window
    w->values[w->idx] = x;

    // Update the mean and M2 using Welford's method
    if (w->n < WINDOW_SIZE) {
        // If the window isn't full yet, just update the mean and M2
        double delta = x - w->mean;
        w->mean += delta / (w->n + 1);
        w->m2 += delta * (x - w->mean);
        w->n++;
    } else {
        // If the window is full, update by removing the oldest value
        double old_val = w->values[w->idx];
        double delta = x - w->mean;
        double old_delta = old_val - w->mean;

        w->mean += delta / WINDOW_SIZE;
        w->m2 += delta * (x - w->mean) - old_delta * (old_val - w->mean);
    }

    // Move the index in the circular buffer
    w->idx = (w->idx + 1) % WINDOW_SIZE;
}

// Calculate the standard deviation of the values in the window
double welford_stddev(Welford *w) {
    return (w->n > 1) ? sqrt(w->m2 / (w->n - 1)) : 0.0;
}

// Check if a value is an anomaly based on the Welford model
int is_welford_anomaly(Welford *w, double value, double threshold) {
    if (w->n < 2) return 0; // Not enough data
    double stddev = welford_stddev(w);
    return fabs(value - w->mean) > threshold * stddev;
}
