#pragma once
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <vector>

namespace phys::bench {

// Accumulates per-step durations (ms) and reports mean/percentiles/max.
class DurationStats
{
public:
    struct Summary
    {
        double mean = 0.0;
        double p50 = 0.0;
        double p95 = 0.0;
        double p99 = 0.0;
        double max = 0.0;
    };

    void record(double ms) { samples.push_back(ms); }

    Summary summarize() const
    {
        if (samples.empty()) return {};

        std::vector<double> sorted = samples;
        std::sort(sorted.begin(), sorted.end());

        auto percentile = [&](double p) {
            std::size_t index = static_cast<std::size_t>(p * (sorted.size() - 1));
            return sorted[index];
        };

        double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
        return {sum / sorted.size(), percentile(0.50), percentile(0.95), percentile(0.99), sorted.back()};
    }

private:
    std::vector<double> samples;
};

}
