#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "cli_common.hpp"
#include "defaults.hpp"
#include "segment_merge.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
typedef Surface_mesh::Face_index Face_index;

namespace PMP = CGAL::Polygon_mesh_processing;

void print_usage(const char* program) {
    neuromesh::print_help(program, R"(Usage:
  Phase 1 (compute SDF):
    mesh_segmentation --sdf --input <mesh> --output-prefix <prefix> [options]

  Phase 2 (segment from SDF):
    mesh_segmentation --segment --input <mesh> --output-prefix <prefix> [options]

  Phase 3 (merge segment labels):
    mesh_segmentation --merge --input <mesh> --output-prefix <prefix> [options]

Common options:
  --input PATH              Input mesh file (required)
  --output-prefix PREFIX    Output path prefix (required; .obj/.sdf/.seg/.ply derived)
  --help                    Show this help

SDF options (--sdf):
  --rays N                  Ray count per face (default: 25)
  --cone-angle A            Cone angle in radians (default: 2pi/3)
  --no-postprocess          Disable CGAL SDF postprocessing (default: postprocess on)

Segment options (--segment):
  --sdf-file PATH           SDF sidecar path (default: <input_basename>.sdf)
  --clusters N              Soft-cluster count (default: 3)
  --lambda L                Graph-cut smoothing (default: 0.22)

Merge options (--merge):
  --seg-file PATH           Segment sidecar (default: <input_basename>.seg)
  --sdf-file PATH           SDF sidecar (default: <input_basename>.sdf)
  --min-faces N             Artifact merge threshold (default: 30)
  --min-spine-faces N       Minimum preserved spine size (default: 10)
  --bridge-max-faces N      Bridge/neck merge threshold (default: 250)
  --spine-sdf-percentile P  Thin-spine preservation percentile (default: 80)
  --max-passes N            Merge iteration limit (default: 64)
  --soma-id ID              Override auto-detected soma segment

Examples:
  mesh_segmentation --sdf --input data/repaired.obj --output-prefix data/repaired
  mesh_segmentation --segment --input data/repaired.obj --output-prefix data/segmented --clusters 3 --lambda 0.22
  mesh_segmentation --merge --input data/segmented.obj --output-prefix data/merged --seg-file data/segmented.seg
)");
}

bool has_known_mesh_extension(const std::string& path) {
    static const char* extensions[] = {".obj", ".off", ".ply", ".stl", ".vtp", ".ts"};
    for (const char* ext : extensions) {
        const std::size_t ext_len = std::strlen(ext);
        if (path.size() >= ext_len &&
            path.compare(path.size() - ext_len, ext_len, ext) == 0) {
            return true;
        }
    }
    return false;
}

std::string strip_mesh_extension(const std::string& path) {
    if (has_known_mesh_extension(path)) {
        const std::size_t dot = path.find_last_of('.');
        return path.substr(0, dot);
    }
    return path;
}

std::string mesh_output_path(const std::string& output_prefix) {
    if (has_known_mesh_extension(output_prefix)) {
        return output_prefix;
    }
    return output_prefix + ".obj";
}

std::string sidecar_path(const std::string& output_prefix, const char* extension) {
    return strip_mesh_extension(output_prefix) + extension;
}

std::string default_sdf_path(const std::string& mesh_path) {
    return strip_mesh_extension(mesh_path) + ".sdf";
}

std::string default_seg_path(const std::string& mesh_path) {
    return strip_mesh_extension(mesh_path) + ".seg";
}

bool load_mesh(const std::string& path, Surface_mesh& mesh) {
    std::cout << "Reading mesh from: " << path << std::endl;

    if (!PMP::IO::read_polygon_mesh(path, mesh)) {
        std::cerr << "Error: Cannot read mesh file: " << path << std::endl;
        return false;
    }

    if (mesh.is_empty()) {
        std::cerr << "Error: Mesh is empty: " << path << std::endl;
        return false;
    }

    if (!CGAL::is_triangle_mesh(mesh)) {
        std::cout << "Triangulating non-triangle faces..." << std::endl;
        PMP::triangulate_faces(mesh);
    }

    if (!CGAL::is_triangle_mesh(mesh)) {
        std::cerr << "Error: Mesh is not a triangle mesh after triangulation" << std::endl;
        return false;
    }

    bool has_boundary = false;
    for (Surface_mesh::Halfedge_index h : mesh.halfedges()) {
        if (CGAL::is_border(h, mesh)) {
            has_boundary = true;
            break;
        }
    }

    if (has_boundary) {
        std::cout << "Warning: Mesh has boundary edges. SDF quality may degrade." << std::endl;
        std::cout << "         Consider running mesh_repair before segmentation." << std::endl;
    }

    std::cout << "Loaded mesh: " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces" << std::endl;
    return true;
}

bool write_sdf_sidecar(
    const std::string& path,
    const Surface_mesh& mesh,
    const Surface_mesh::Property_map<Face_index, double>& sdf_map) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: Cannot write SDF file: " << path << std::endl;
        return false;
    }

    out << "# neuromesh-sdf v1\n";
    out << "faces " << mesh.number_of_faces() << "\n";
    for (Face_index f : mesh.faces()) {
        out << sdf_map[f] << "\n";
    }
    return true;
}

bool read_sdf_sidecar(
    const std::string& path,
    const Surface_mesh& mesh,
    Surface_mesh::Property_map<Face_index, double>& sdf_map) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Error: Cannot read SDF file: " << path << std::endl;
        return false;
    }

    std::string line;
    if (!std::getline(in, line) || line != "# neuromesh-sdf v1") {
        std::cerr << "Error: Invalid SDF file header: " << path << std::endl;
        return false;
    }

    if (!std::getline(in, line)) {
        std::cerr << "Error: Missing face count in SDF file: " << path << std::endl;
        return false;
    }

    std::istringstream header(line);
    std::string label;
    std::size_t expected_faces = 0;
    if (!(header >> label >> expected_faces) || label != "faces") {
        std::cerr << "Error: Invalid SDF face count line: " << path << std::endl;
        return false;
    }

    if (expected_faces != mesh.number_of_faces()) {
        std::cerr << "Error: SDF face count (" << expected_faces
                  << ") does not match mesh face count ("
                  << mesh.number_of_faces() << ")" << std::endl;
        return false;
    }

    for (Face_index f : mesh.faces()) {
        if (!std::getline(in, line)) {
            std::cerr << "Error: Unexpected end of SDF file: " << path << std::endl;
            return false;
        }
        std::istringstream value_stream(line);
        double value = 0.0;
        if (!(value_stream >> value)) {
            std::cerr << "Error: Invalid SDF value in file: " << path << std::endl;
            return false;
        }
        sdf_map[f] = value;
    }

    return true;
}

bool read_seg_sidecar(
    const std::string& path,
    const Surface_mesh& mesh,
    Surface_mesh::Property_map<Face_index, std::size_t>& segment_map) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Error: Cannot read segment file: " << path << std::endl;
        return false;
    }

    std::string line;
    if (!std::getline(in, line) || line != "# neuromesh-seg v1") {
        std::cerr << "Error: Invalid segment file header: " << path << std::endl;
        return false;
    }

    if (!std::getline(in, line)) {
        std::cerr << "Error: Missing face count in segment file: " << path << std::endl;
        return false;
    }

    std::istringstream header(line);
    std::string label;
    std::size_t expected_faces = 0;
    if (!(header >> label >> expected_faces) || label != "faces") {
        std::cerr << "Error: Invalid segment face count line: " << path << std::endl;
        return false;
    }

    if (expected_faces != mesh.number_of_faces()) {
        std::cerr << "Error: Segment face count (" << expected_faces
                  << ") does not match mesh face count ("
                  << mesh.number_of_faces() << ")" << std::endl;
        return false;
    }

    for (Face_index f : mesh.faces()) {
        if (!std::getline(in, line)) {
            std::cerr << "Error: Unexpected end of segment file: " << path << std::endl;
            return false;
        }
        std::istringstream value_stream(line);
        std::size_t value = 0;
        if (!(value_stream >> value)) {
            std::cerr << "Error: Invalid segment value in file: " << path << std::endl;
            return false;
        }
        segment_map[f] = value;
    }

    return true;
}

bool write_seg_sidecar(
    const std::string& path,
    const Surface_mesh& mesh,
    const Surface_mesh::Property_map<Face_index, std::size_t>& segment_map) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: Cannot write segment file: " << path << std::endl;
        return false;
    }

    out << "# neuromesh-seg v1\n";
    out << "faces " << mesh.number_of_faces() << "\n";
    for (Face_index f : mesh.faces()) {
        out << segment_map[f] << "\n";
    }
    return true;
}

CGAL::IO::Color segment_to_color(std::size_t segment_id, std::size_t num_segments) {
    if (num_segments == 0) {
        return CGAL::IO::Color(128, 128, 128);
    }

    const double hue = 360.0 * static_cast<double>(segment_id) / static_cast<double>(num_segments);
    const double saturation = 0.65;
    const double lightness = 0.55;

    const double c = (1.0 - std::abs(2.0 * lightness - 1.0)) * saturation;
    const double x = c * (1.0 - std::abs(std::fmod(hue / 60.0, 2.0) - 1.0));
    const double m = lightness - c / 2.0;

    double r = 0.0;
    double g = 0.0;
    double b = 0.0;

    if (hue < 60.0) {
        r = c; g = x; b = 0.0;
    } else if (hue < 120.0) {
        r = x; g = c; b = 0.0;
    } else if (hue < 180.0) {
        r = 0.0; g = c; b = x;
    } else if (hue < 240.0) {
        r = 0.0; g = x; b = c;
    } else if (hue < 300.0) {
        r = x; g = 0.0; b = c;
    } else {
        r = c; g = 0.0; b = x;
    }

    return CGAL::IO::Color(
        static_cast<unsigned char>(std::round((r + m) * 255.0)),
        static_cast<unsigned char>(std::round((g + m) * 255.0)),
        static_cast<unsigned char>(std::round((b + m) * 255.0)));
}

void apply_segment_colors(
    Surface_mesh& mesh,
    const Surface_mesh::Property_map<Face_index, std::size_t>& segment_map) {
    auto color_map = mesh.add_property_map<Face_index, CGAL::IO::Color>("f:color").first;

    std::size_t max_segment = 0;
    for (Face_index f : mesh.faces()) {
        max_segment = std::max(max_segment, segment_map[f]);
    }

    const std::size_t num_segments = max_segment + 1;
    for (Face_index f : mesh.faces()) {
        color_map[f] = segment_to_color(segment_map[f], num_segments);
    }
}

struct SdfOptions {
    std::string input_mesh;
    std::string output_prefix;
    std::size_t number_of_rays = neuromesh::SdfDefaults::rays;
    double cone_angle = neuromesh::SdfDefaults::cone_angle;
    bool postprocess = neuromesh::SdfDefaults::postprocess;
};

struct SegmentOptions {
    std::string input_mesh;
    std::string output_prefix;
    std::string sdf_file;
    std::size_t clusters = neuromesh::SegmentDefaults::clusters;
    double lambda = neuromesh::SegmentDefaults::lambda;
};

struct MergePhaseOptions {
    std::string input_mesh;
    std::string output_prefix;
    std::string seg_file;
    std::string sdf_file;
    MergeOptions merge;
};

int run_sdf_phase(const SdfOptions& opts) {
    Surface_mesh mesh;
    if (!load_mesh(opts.input_mesh, mesh)) {
        return 1;
    }

    auto sdf_map = mesh.add_property_map<Face_index, double>("f:sdf", 0.0).first;

    std::cout << "Computing SDF values (rays=" << opts.number_of_rays
              << ", cone_angle=" << opts.cone_angle
              << ", postprocess=" << (opts.postprocess ? "on" : "off") << ")..." << std::endl;

    const std::pair<double, double> min_max_sdf =
        CGAL::sdf_values(mesh, sdf_map, opts.cone_angle, opts.number_of_rays, opts.postprocess);

    const std::string mesh_path = mesh_output_path(opts.output_prefix);
    const std::string sdf_path = sidecar_path(opts.output_prefix, ".sdf");

    if (!write_sdf_sidecar(sdf_path, mesh, sdf_map)) {
        return 1;
    }

    if (!CGAL::IO::write_polygon_mesh(mesh_path, mesh)) {
        std::cerr << "Error: Cannot write mesh file: " << mesh_path << std::endl;
        return 1;
    }

    std::cout << "\n=== SDF Summary ===" << std::endl;
    std::cout << "Minimum SDF: " << min_max_sdf.first << std::endl;
    std::cout << "Maximum SDF: " << min_max_sdf.second << std::endl;
    std::cout << "Faces: " << mesh.number_of_faces() << std::endl;
    std::cout << "Wrote mesh: " << mesh_path << std::endl;
    std::cout << "Wrote SDF:  " << sdf_path << std::endl;
    std::cout << "===================" << std::endl;
    return 0;
}

int run_segment_phase(const SegmentOptions& opts) {
    Surface_mesh mesh;
    if (!load_mesh(opts.input_mesh, mesh)) {
        return 1;
    }

    auto sdf_map = mesh.add_property_map<Face_index, double>("f:sdf", 0.0).first;

    const std::string sdf_path = opts.sdf_file.empty()
        ? default_sdf_path(opts.input_mesh)
        : opts.sdf_file;

    std::cout << "Reading SDF from: " << sdf_path << std::endl;
    if (!read_sdf_sidecar(sdf_path, mesh, sdf_map)) {
        return 1;
    }

    auto segment_map = mesh.add_property_map<Face_index, std::size_t>("f:segment", 0).first;

    std::cout << "Segmenting mesh (clusters=" << opts.clusters
              << ", lambda=" << opts.lambda << ")..." << std::endl;

    const std::size_t number_of_segments =
        CGAL::segmentation_from_sdf_values(
            mesh, sdf_map, segment_map, opts.clusters, opts.lambda);

    apply_segment_colors(mesh, segment_map);

    const std::string mesh_path = mesh_output_path(opts.output_prefix);
    const std::string colored_path = sidecar_path(opts.output_prefix, ".ply");
    const std::string seg_path = sidecar_path(opts.output_prefix, ".seg");

    if (!write_seg_sidecar(seg_path, mesh, segment_map)) {
        return 1;
    }

    if (!CGAL::IO::write_polygon_mesh(mesh_path, mesh)) {
        std::cerr << "Error: Cannot write mesh file: " << mesh_path << std::endl;
        return 1;
    }

    if (!CGAL::IO::write_polygon_mesh(colored_path, mesh)) {
        std::cerr << "Error: Cannot write colored mesh file: " << colored_path << std::endl;
        return 1;
    }

    std::cout << "\n=== Segmentation Summary ===" << std::endl;
    std::cout << "Number of segments: " << number_of_segments << std::endl;
    std::cout << "Faces: " << mesh.number_of_faces() << std::endl;
    std::cout << "Wrote mesh:          " << mesh_path << std::endl;
    std::cout << "Wrote colored mesh:  " << colored_path << std::endl;
    std::cout << "Wrote segment IDs:   " << seg_path << std::endl;
    std::cout << "============================" << std::endl;
    return 0;
}

int run_merge_phase(const MergePhaseOptions& opts) {
    Surface_mesh mesh;
    if (!load_mesh(opts.input_mesh, mesh)) {
        return 1;
    }

    auto segment_map = mesh.add_property_map<Face_index, std::size_t>("f:segment", 0).first;
    auto sdf_map = mesh.add_property_map<Face_index, double>("f:sdf", 0.0).first;

    const std::string seg_path = opts.seg_file.empty()
        ? default_seg_path(opts.input_mesh)
        : opts.seg_file;
    const std::string sdf_path = opts.sdf_file.empty()
        ? default_sdf_path(opts.input_mesh)
        : opts.sdf_file;

    std::cout << "Reading segment labels from: " << seg_path << std::endl;
    if (!read_seg_sidecar(seg_path, mesh, segment_map)) {
        return 1;
    }

    std::cout << "Reading SDF from: " << sdf_path << std::endl;
    if (!read_sdf_sidecar(sdf_path, mesh, sdf_map)) {
        return 1;
    }

    std::cout << "Merging segments (min_faces=" << opts.merge.min_faces
              << ", min_spine_faces=" << opts.merge.min_spine_faces
              << ", bridge_max_faces=" << opts.merge.bridge_max_faces
              << ", spine_sdf_percentile=" << opts.merge.spine_sdf_percentile << ")..." << std::endl;

    const MergeResult merge_result = merge_segments(mesh, segment_map, sdf_map, opts.merge);

    apply_segment_colors(mesh, segment_map);

    const std::string mesh_path = mesh_output_path(opts.output_prefix);
    const std::string colored_path = sidecar_path(opts.output_prefix, ".ply");
    const std::string out_seg_path = sidecar_path(opts.output_prefix, ".seg");
    const std::string merge_log_path = sidecar_path(opts.output_prefix, "_merge_log.txt");

    if (!write_seg_sidecar(out_seg_path, mesh, segment_map)) {
        return 1;
    }

    if (!write_merge_log(merge_log_path, merge_result, opts.merge)) {
        std::cerr << "Error: Cannot write merge log: " << merge_log_path << std::endl;
        return 1;
    }

    if (!CGAL::IO::write_polygon_mesh(mesh_path, mesh)) {
        std::cerr << "Error: Cannot write mesh file: " << mesh_path << std::endl;
        return 1;
    }

    if (!CGAL::IO::write_polygon_mesh(colored_path, mesh)) {
        std::cerr << "Error: Cannot write colored mesh file: " << colored_path << std::endl;
        return 1;
    }

    std::cout << "\n=== Merge Summary ===" << std::endl;
    std::cout << "Segments before:      " << merge_result.segments_before << std::endl;
    std::cout << "Segments after:       " << merge_result.segments_after << std::endl;
    std::cout << "Merge operations:     " << merge_result.merges.size() << std::endl;
    std::cout << "Soma segment ID:      " << merge_result.soma_id << std::endl;
    std::cout << "Spine SDF threshold:  " << merge_result.spine_sdf_threshold << std::endl;
    std::cout << "Wrote mesh:           " << mesh_path << std::endl;
    std::cout << "Wrote colored mesh:   " << colored_path << std::endl;
    std::cout << "Wrote segment IDs:    " << out_seg_path << std::endl;
    std::cout << "Wrote merge log:      " << merge_log_path << std::endl;
    std::cout << "=====================" << std::endl;
    return 0;
}

enum class SegmentationMode { None, Sdf, Segment, Merge };

struct ParsedSegmentationArgs {
    SegmentationMode mode = SegmentationMode::None;
    SdfOptions sdf;
    SegmentOptions segment;
    MergePhaseOptions merge;
};

bool parse_common_io(int& i, int argc, char** argv, std::string& input, std::string& output_prefix) {
    const std::string arg = argv[i];
    if (arg == "--input") {
        if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
            return false;
        }
        input = argv[i];
        return true;
    }
    if (arg == "--output-prefix") {
        if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
            return false;
        }
        output_prefix = argv[i];
        return true;
    }
    return false;
}

bool parse_args(int argc, char** argv, ParsedSegmentationArgs& parsed) {
    if (argc < 2) {
        return false;
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (neuromesh::is_help_flag(arg)) {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (arg == "--sdf") {
            parsed.mode = SegmentationMode::Sdf;
        } else if (arg == "--segment") {
            parsed.mode = SegmentationMode::Segment;
        } else if (arg == "--merge") {
            parsed.mode = SegmentationMode::Merge;
        } else if (parse_common_io(i, argc, argv, parsed.sdf.input_mesh, parsed.sdf.output_prefix)) {
            parsed.segment.input_mesh = parsed.sdf.input_mesh;
            parsed.segment.output_prefix = parsed.sdf.output_prefix;
            parsed.merge.input_mesh = parsed.sdf.input_mesh;
            parsed.merge.output_prefix = parsed.sdf.output_prefix;
        } else if (arg == "--rays") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.sdf.number_of_rays = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--cone-angle") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.sdf.cone_angle = std::stod(argv[i]);
        } else if (arg == "--no-postprocess") {
            parsed.sdf.postprocess = false;
        } else if (arg == "--sdf-file") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.segment.sdf_file = argv[i];
            parsed.merge.sdf_file = argv[i];
        } else if (arg == "--clusters") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.segment.clusters = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--lambda") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.segment.lambda = std::stod(argv[i]);
        } else if (arg == "--seg-file") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.merge.seg_file = argv[i];
        } else if (arg == "--min-faces") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.merge.merge.min_faces = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--min-spine-faces") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.merge.merge.min_spine_faces = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--bridge-max-faces") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.merge.merge.bridge_max_faces = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--spine-sdf-percentile") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.merge.merge.spine_sdf_percentile = std::stod(argv[i]);
        } else if (arg == "--max-passes") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.merge.merge.max_passes = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--soma-id") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            parsed.merge.merge.soma_id = static_cast<std::size_t>(std::stoul(argv[i]));
        } else {
            neuromesh::print_error("Unknown flag: " + arg);
            return false;
        }
    }

    if (parsed.mode == SegmentationMode::None) {
        neuromesh::print_error("One of --sdf, --segment, or --merge is required.");
        return false;
    }

    const std::string& input = parsed.sdf.input_mesh;
    const std::string& output_prefix = parsed.sdf.output_prefix;
    if (input.empty() || output_prefix.empty()) {
        neuromesh::print_error("--input and --output-prefix are required.");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    ParsedSegmentationArgs parsed;
    if (!parse_args(argc, argv, parsed)) {
        print_usage(argv[0]);
        return 1;
    }

    switch (parsed.mode) {
        case SegmentationMode::Sdf:
            return run_sdf_phase(parsed.sdf);
        case SegmentationMode::Segment:
            return run_segment_phase(parsed.segment);
        case SegmentationMode::Merge:
            return run_merge_phase(parsed.merge);
        case SegmentationMode::None:
            break;
    }

    print_usage(argv[0]);
    return 1;
}
