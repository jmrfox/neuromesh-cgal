#pragma once

#include <cstddef>

namespace neuromesh {

// Defaults tuned on the bundled cell morphology mesh (~330k faces).
// Merge defaults favor preserving partitions (~90 merged segments) over aggressive coalescing.

struct RepairDefaults {
    static constexpr std::size_t keep_components = 1;
};

struct SdfDefaults {
    static constexpr std::size_t rays = 25;
    static constexpr double cone_angle = 2.0 / 3.0 * 3.14159265358979323846;
    static constexpr bool postprocess = true;
};

struct SegmentDefaults {
    static constexpr std::size_t clusters = 3;
    static constexpr double lambda = 0.22;
};

struct MergeDefaults {
    static constexpr std::size_t min_faces = 30;
    static constexpr std::size_t min_spine_faces = 10;
    static constexpr std::size_t bridge_max_faces = 250;
    static constexpr double spine_sdf_percentile = 80.0;
    static constexpr std::size_t max_passes = 64;
};

struct PartitionDefaults {
    static constexpr bool require_closed = true;
    static constexpr std::size_t min_output_faces = 0;
    static constexpr std::size_t filename_width_auto = 0;
};

}  // namespace neuromesh
