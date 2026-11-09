// Link against the selected middle package's objects; no ROS node or device is created.
#include "controller/ArmController.h"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    try {
        const auto model_file = std::filesystem::path(argv[2]) / "gravity-test.urdf";
        std::filesystem::copy_file(argv[1], model_file,
                                  std::filesystem::copy_options::overwrite_existing);
        path_options[2] = model_file.string();
        KDL::JntArray first(7), next(7);
        for (unsigned i = 0; i < 7; ++i) { first(i) = .01 * i; next(i) = -.02 * i; }
        ForceComp subject, reference;
#ifdef PREPARE_MODEL
        subject.prepare_model(2);
        reference.prepare_model(2);
#endif
        if (!subject.init(first, 0, first, 0, 2) || !reference.init(next, 0, next, 0, 2)) return 3;
        const std::vector<double> q(7, .13), v(7, 0.);
        const auto expected = reference.gravComp(q, v, q, v);
        subject.gravComp(q, v, q, v);
        // The model is an immutable startup input. Reinitializing the measured
        // state must keep working without reading/parsing the URDF again.
        std::filesystem::remove(model_file);
        double maximum_ms = 0.;
        for (unsigned repeat = 0; repeat < 3; ++repeat) {
            const auto begin = std::chrono::steady_clock::now();
            if (!subject.init(next, 0, next, 0, 2)) return 4;
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count();
            maximum_ms = std::max(maximum_ms, ms);
            const auto actual = subject.gravComp(q, v, q, v);
            if (actual.size() != expected.size()) return 5;
            for (unsigned i = 0; i < actual.size(); ++i)
                if (!std::isfinite(actual[i]) || std::abs(actual[i] - expected[i]) > 1e-12) return 6;
        }
        std::cout << std::setprecision(17) << "GRAVITY";
        for (double value : expected) std::cout << ' ' << value;
        std::cout << "\nPASS model reuse, measured-state reset and gravity equality; max_init_ms="
                  << maximum_ms << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
