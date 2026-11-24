#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

constexpr int MAX_POINTS = 4096;

extern "C" void ground_classify(
    const float *x, const float *y, const float *z,
    int num_points, float cell_size,
    float height_threshold, unsigned char *labels
);

struct Point {
    float x, y, z, intensity;
};

struct Metrics {
    uint64_t tp = 0, tn = 0;
    uint64_t fp = 0, fn = 0;
    uint64_t ignored = 0;
    uint64_t batches = 0;
    double kernel_ms = 0;
    double total_ms = 0;

    void add(const Metrics& m) {
        tp += m.tp;
        tn += m.tn;
        fp += m.fp;
        fn += m.fn;
        ignored += m.ignored;
        batches += m.batches;
        kernel_ms += m.kernel_ms;
        total_ms += m.total_ms;
    }
};

template<typename T>
std::vector<T> read_binary(const fs::path& path) {
    auto bytes = fs::file_size(path);

    if (bytes % sizeof(T) != 0)
        throw std::runtime_error("Invalid file size");

    std::vector<T> data(bytes / sizeof(T));

    std::ifstream file(path, std::ios::binary);

    if (!file.read(reinterpret_cast<char*>(data.data()),
                   bytes))
        throw std::runtime_error("Cannot read " + path.string());

    return data;
}

int ground_truth(uint32_t raw_label) {
    uint32_t cls = raw_label & 0xFFFF;

    if (cls == 0 || cls == 1)
        return -1;

    if (cls == 40 || cls == 44 || cls == 48 ||
        cls == 49 || cls == 60 || cls == 72)
        return 1;

    return 0;
}

Metrics evaluate(
    const std::vector<Point>& points,
    const std::vector<uint32_t>& labels,
    float cell_size,
    float threshold
) {
    Metrics m;
    auto total_start = Clock::now();

    using Cell = std::pair<int, int>;
    std::map<Cell, std::vector<size_t>> cells;

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];

        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
            throw std::runtime_error("Non-finite coordinate");

        int cx = static_cast<int>(
            std::floor(p.x / cell_size));
        int cy = static_cast<int>(
            std::floor(p.y / cell_size));

        cells[{cx, cy}].push_back(i);
    }

    std::vector<float> x, y, z;
    std::vector<size_t> indices;
    std::vector<unsigned char> predictions(MAX_POINTS);

    x.reserve(MAX_POINTS);
    y.reserve(MAX_POINTS);
    z.reserve(MAX_POINTS);
    indices.reserve(MAX_POINTS);

    auto flush_batch = [&]() {
        if (indices.empty())
            return;

        auto start = Clock::now();

        ground_classify(
            x.data(), y.data(), z.data(),
            static_cast<int>(indices.size()),
            cell_size, threshold, predictions.data()
        );

        auto end = Clock::now();

        m.kernel_ms +=
            std::chrono::duration<double, std::milli>(
                end - start).count();

        ++m.batches;

        for (size_t j = 0; j < indices.size(); ++j) {
            int truth = ground_truth(labels[indices[j]]);
            int pred = predictions[j];

            if (truth == -1)
                ++m.ignored;
            else if (truth == 1 && pred == 1)
                ++m.tp;
            else if (truth == 0 && pred == 0)
                ++m.tn;
            else if (truth == 0 && pred == 1)
                ++m.fp;
            else
                ++m.fn;
        }

        x.clear();
        y.clear();
        z.clear();
        indices.clear();
    };

    for (const auto& entry : cells) {
        const auto& members = entry.second;

        if (members.size() > MAX_POINTS)
            throw std::runtime_error(
                "Single cell exceeds kernel capacity");

        if (indices.size() + members.size() > MAX_POINTS)
            flush_batch();

        for (size_t i : members) {
            x.push_back(points[i].x);
            y.push_back(points[i].y);
            z.push_back(points[i].z);
            indices.push_back(i);
        }
    }

    flush_batch();

    m.total_ms =
        std::chrono::duration<double, std::milli>(
            Clock::now() - total_start).count();

    return m;
}

double ratio(double a, double b) {
    return b > 0 ? a / b : 0.0;
}

void print_metrics(
    const std::string& name, const Metrics& m
) {
    double accuracy = ratio(
        m.tp + m.tn, m.tp + m.tn + m.fp + m.fn);

    double precision = ratio(m.tp, m.tp + m.fp);
    double recall = ratio(m.tp, m.tp + m.fn);
    double f1 = ratio(
        2.0 * precision * recall, precision + recall);

    double iou = ratio(m.tp, m.tp + m.fp + m.fn);

    std::cout << std::fixed << std::setprecision(3)
              << name << ","
              << m.tp << "," << m.tn << ","
              << m.fp << "," << m.fn << ","
              << m.ignored << ","
              << accuracy * 100 << ","
              << precision * 100 << ","
              << recall * 100 << ","
              << f1 * 100 << ","
              << iou * 100 << ","
              << m.batches << ","
              << m.kernel_ms << ","
              << m.total_ms << "\n";
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <sequence_path> <cell_size>"
                  << " <height_threshold>\n";
        return 1;
    }

    try {
        fs::path root = argv[1];
        float cell_size = std::stof(argv[2]);
        float threshold = std::stof(argv[3]);

        if (!std::isfinite(cell_size) ||
            !std::isfinite(threshold) ||
            cell_size <= 0 || threshold < 0)
            throw std::runtime_error("Invalid parameters");

        std::vector<fs::path> scans;

        for (const auto& entry :
             fs::directory_iterator(root / "velodyne")) {
            if (entry.path().extension() == ".bin")
                scans.push_back(entry.path());
        }

        std::sort(scans.begin(), scans.end());

        if (scans.empty())
            throw std::runtime_error("No scans found");

        std::cout << "Cell size: " << cell_size
                  << " m | Threshold: " << threshold
                  << " m\n";

        std::cout
            << "Frame,TP,TN,FP,FN,Ignored,"
            << "Accuracy(%),Precision(%),Recall(%),"
            << "F1(%),IoU(%),Batches,"
            << "KernelCPU(ms),TotalCPU(ms)\n";

        Metrics total;

        for (const auto& scan : scans) {
            auto points = read_binary<Point>(scan);

            auto labels = read_binary<uint32_t>(
                root / "labels" /
                (scan.stem().string() + ".label"));

            if (points.size() != labels.size())
                throw std::runtime_error(
                    "Point and label counts differ");

            Metrics m = evaluate(
                points, labels, cell_size, threshold);

            if (m.tp + m.tn + m.fp + m.fn + m.ignored
                != points.size())
                throw std::runtime_error(
                    "Evaluation point count mismatch");

            print_metrics(scan.stem().string(), m);
            total.add(m);
        }

        print_metrics("TOTAL", total);

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
