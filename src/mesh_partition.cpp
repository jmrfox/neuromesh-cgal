#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "cli_common.hpp"
#include "defaults.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
typedef Surface_mesh::Face_index Face_index;
typedef Surface_mesh::Vertex_index Vertex_index;
typedef Surface_mesh::Halfedge_index Halfedge_index;

namespace PMP = CGAL::Polygon_mesh_processing;
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// I/O helpers (minimal duplication from mesh_segmentation.cpp)
// ---------------------------------------------------------------------------

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

std::string default_seg_path(const std::string& mesh_path) {
    return strip_mesh_extension(mesh_path) + ".seg";
}

bool load_mesh(const std::string& path, Surface_mesh& mesh, bool require_closed) {
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
    for (Halfedge_index h : mesh.halfedges()) {
        if (CGAL::is_border(h, mesh)) {
            has_boundary = true;
            break;
        }
    }

    if (has_boundary) {
        if (require_closed) {
            std::cerr << "Error: Mesh has boundary edges. Run mesh_repair first." << std::endl;
            return false;
        }
        std::cout << "Warning: Mesh has boundary edges." << std::endl;
    }

    std::cout << "Loaded mesh: " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_faces() << " faces" << std::endl;
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

// ---------------------------------------------------------------------------
// Partition data structures
// ---------------------------------------------------------------------------

struct LoopKey {
    std::vector<std::size_t> sorted_vertex_ids;

    explicit LoopKey(const std::vector<Vertex_index>& cycle) {
        sorted_vertex_ids.reserve(cycle.size());
        for (Vertex_index v : cycle) {
            sorted_vertex_ids.push_back(static_cast<std::size_t>(v.idx()));
        }
        std::sort(sorted_vertex_ids.begin(), sorted_vertex_ids.end());
    }

    bool operator<(const LoopKey& other) const {
        return sorted_vertex_ids < other.sorted_vertex_ids;
    }
};

struct SegmentStats {
    std::size_t segment_id = 0;
    std::size_t face_count = 0;
    std::size_t caps_applied = 0;
    std::size_t cap_failures = 0;
    bool watertight = false;
    bool valid_mesh = false;
};

// ---------------------------------------------------------------------------
// Mesh extraction and hole capping
// ---------------------------------------------------------------------------

void extract_faces(
    const Surface_mesh& mesh,
    const std::vector<Face_index>& faces,
    Surface_mesh& sub,
    std::unordered_map<Vertex_index, Vertex_index>& orig_to_sub,
    std::unordered_map<Vertex_index, Vertex_index>& sub_to_orig) {
    sub.clear();
    orig_to_sub.clear();
    sub_to_orig.clear();

    for (Face_index f : faces) {
        std::vector<Vertex_index> face_vertices;
        face_vertices.reserve(3);
        for (Vertex_index v : vertices_around_face(mesh.halfedge(f), mesh)) {
            auto [it, inserted] = orig_to_sub.emplace(v, Surface_mesh::null_vertex());
            if (inserted) {
                it->second = sub.add_vertex(mesh.point(v));
            }
            face_vertices.push_back(it->second);
        }
        sub.add_face(face_vertices);
    }

    for (const auto& entry : orig_to_sub) {
        sub_to_orig[entry.second] = entry.first;
    }
}

std::vector<Vertex_index> border_cycle_orig_vertices(
    Halfedge_index start,
    const Surface_mesh& sub,
    const std::unordered_map<Vertex_index, Vertex_index>& sub_to_orig) {
    std::vector<Vertex_index> cycle;
    Halfedge_index current = start;
    do {
        cycle.push_back(sub_to_orig.at(source(current, sub)));
        current = next(current, sub);
    } while (current != start);
    return cycle;
}

bool fan_triangulate_hole(Surface_mesh& sub, Halfedge_index border_halfedge) {
    std::vector<Vertex_index> vertices;
    Halfedge_index current = border_halfedge;
    do {
        vertices.push_back(source(current, sub));
        current = next(current, sub);
    } while (current != border_halfedge);

    if (vertices.size() < 3) {
        return false;
    }

    for (std::size_t i = 1; i + 1 < vertices.size(); ++i) {
        const auto face = sub.add_face(vertices[0], vertices[i], vertices[i + 1]);
        if (face == Surface_mesh::null_face()) {
            return false;
        }
    }
    return true;
}

bool centroid_triangulate_hole(Surface_mesh& sub, Halfedge_index border_halfedge) {
    std::vector<Vertex_index> vertices;
    Halfedge_index current = border_halfedge;
    do {
        vertices.push_back(source(current, sub));
        current = next(current, sub);
    } while (current != border_halfedge);

    if (vertices.size() < 3) {
        return false;
    }

    Kernel::Vector_3 sum(0, 0, 0);
    for (Vertex_index v : vertices) {
        sum = sum + (sub.point(v) - CGAL::ORIGIN);
    }
    const double inv_n = 1.0 / static_cast<double>(vertices.size());
    const Point_3 centroid(
        sum.x() * inv_n,
        sum.y() * inv_n,
        sum.z() * inv_n);

    const Vertex_index center = sub.add_vertex(centroid);
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const Vertex_index v0 = vertices[i];
        const Vertex_index v1 = vertices[(i + 1) % vertices.size()];
        const auto face = sub.add_face(v0, v1, center);
        if (face == Surface_mesh::null_face()) {
            return false;
        }
    }
    return true;
}

bool cap_border_halfedge(Surface_mesh& sub, Halfedge_index border_halfedge) {
    if (!CGAL::is_border(border_halfedge, sub)) {
        return true;
    }

    std::vector<Face_index> patch_faces;
    PMP::triangulate_hole(sub, border_halfedge, std::back_inserter(patch_faces));
    if (!patch_faces.empty()) {
        return true;
    }

    if (fan_triangulate_hole(sub, border_halfedge)) {
        return true;
    }

    if (centroid_triangulate_hole(sub, border_halfedge)) {
        return true;
    }

    return false;
}

bool mesh_is_closed(const Surface_mesh& mesh) {
    for (Halfedge_index h : mesh.halfedges()) {
        if (CGAL::is_border(h, mesh)) {
            return false;
        }
    }
    return true;
}

bool cap_segment_holes(
    Surface_mesh& sub,
    const std::unordered_map<Vertex_index, Vertex_index>& sub_to_orig,
    std::set<LoopKey>& unique_loops,
    std::size_t& caps_applied,
    std::size_t& cap_failures) {
    std::set<LoopKey> failed_loops;

    while (true) {
        Halfedge_index border_halfedge = Surface_mesh::null_halfedge();
        std::vector<Vertex_index> cycle_orig;

        for (Halfedge_index h : sub.halfedges()) {
            if (!CGAL::is_border(h, sub)) {
                continue;
            }

            const std::vector<Vertex_index> candidate =
                border_cycle_orig_vertices(h, sub, sub_to_orig);
            const LoopKey key(candidate);
            if (failed_loops.count(key) != 0) {
                continue;
            }

            border_halfedge = h;
            cycle_orig = std::move(candidate);
            break;
        }

        if (border_halfedge == Surface_mesh::null_halfedge()) {
            break;
        }

        unique_loops.insert(LoopKey(cycle_orig));

        if (cap_border_halfedge(sub, border_halfedge)) {
            ++caps_applied;
            continue;
        }

        failed_loops.insert(LoopKey(cycle_orig));
        ++cap_failures;
        std::cerr << "  Warning: Failed to triangulate cut boundary loop ("
                  << cycle_orig.size() << " vertices)" << std::endl;
    }

    return failed_loops.empty();
}

std::string segment_filename(std::size_t segment_id, std::size_t width) {
    std::ostringstream oss;
    oss << "segment_" << std::setfill('0') << std::setw(static_cast<int>(width))
        << segment_id << ".obj";
    return oss.str();
}

struct PartitionOptions {
    std::string input_mesh;
    std::string output_dir;
    std::string seg_file;
    bool require_closed = neuromesh::PartitionDefaults::require_closed;
    std::size_t min_output_faces = neuromesh::PartitionDefaults::min_output_faces;
    std::size_t filename_width = neuromesh::PartitionDefaults::filename_width_auto;
};

void print_usage(const char* program) {
    neuromesh::print_help(program, R"(Usage:
  mesh_partition --input <mesh> --output-dir <dir> [options]

Options:
  --input PATH              Segmented input mesh (required)
  --output-dir PATH         Output directory for segment_NNN.obj files (required)
  --seg-file PATH           Segment sidecar (default: <input_basename>.seg)
  --allow-open              Allow open input meshes (warn instead of abort)
  --min-output-faces N      Skip writing segments with fewer faces (default: 0)
  --filename-width N        Zero-pad width for filenames (default: auto, min 3)
  --help                    Show this help

Example:
  mesh_partition --input data/merged.obj --output-dir data/parts/ --seg-file data/merged.seg
)");
}

bool parse_args(int argc, char** argv, PartitionOptions& opts) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (neuromesh::is_help_flag(arg)) {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (arg == "--input") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.input_mesh = argv[i];
        } else if (arg == "--output-dir") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.output_dir = argv[i];
        } else if (arg == "--seg-file") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.seg_file = argv[i];
        } else if (arg == "--allow-open") {
            opts.require_closed = false;
        } else if (arg == "--min-output-faces") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.min_output_faces = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--filename-width") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.filename_width = static_cast<std::size_t>(std::stoul(argv[i]));
        } else {
            neuromesh::print_error("Unknown flag: " + arg);
            return false;
        }
    }

    if (opts.input_mesh.empty() || opts.output_dir.empty()) {
        neuromesh::print_error("--input and --output-dir are required.");
        return false;
    }

    if (opts.seg_file.empty()) {
        opts.seg_file = default_seg_path(opts.input_mesh);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    PartitionOptions opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    Surface_mesh mesh;
    if (!load_mesh(opts.input_mesh, mesh, opts.require_closed)) {
        return 1;
    }

    auto segment_map = mesh.add_property_map<Face_index, std::size_t>("f:segment", 0).first;

    std::cout << "Reading segment labels from: " << opts.seg_file << std::endl;
    if (!read_seg_sidecar(opts.seg_file, mesh, segment_map)) {
        return 1;
    }

    std::size_t max_segment_id = 0;
    for (Face_index f : mesh.faces()) {
        max_segment_id = std::max(max_segment_id, segment_map[f]);
    }
    const std::size_t num_segment_ids = max_segment_id + 1;

    std::vector<std::vector<Face_index>> faces_per_segment(num_segment_ids);
    for (Face_index f : mesh.faces()) {
        faces_per_segment[segment_map[f]].push_back(f);
    }

    const std::size_t filename_width = opts.filename_width > 0
        ? std::max<std::size_t>(3, opts.filename_width)
        : std::max<std::size_t>(3, std::to_string(max_segment_id).size());

    fs::create_directories(opts.output_dir);

    std::set<LoopKey> unique_loops;
    std::vector<SegmentStats> all_stats;
    std::size_t segments_written = 0;
    std::size_t segments_skipped = 0;
    std::size_t total_cap_failures = 0;

    std::cout << "\nPartitioning into watertight segment meshes..." << std::endl;
    std::cout << "Segment ID range: 0.." << max_segment_id << std::endl;

    for (std::size_t segment_id = 0; segment_id < num_segment_ids; ++segment_id) {
        const std::vector<Face_index>& faces = faces_per_segment[segment_id];
        if (faces.empty()) {
            ++segments_skipped;
            continue;
        }

        if (faces.size() < opts.min_output_faces) {
            ++segments_skipped;
            continue;
        }

        Surface_mesh sub;
        std::unordered_map<Vertex_index, Vertex_index> orig_to_sub;
        std::unordered_map<Vertex_index, Vertex_index> sub_to_orig;
        extract_faces(mesh, faces, sub, orig_to_sub, sub_to_orig);

        SegmentStats stats;
        stats.segment_id = segment_id;
        stats.face_count = faces.size();

        cap_segment_holes(
            sub, sub_to_orig, unique_loops,
            stats.caps_applied, stats.cap_failures);
        total_cap_failures += stats.cap_failures;

        sub.collect_garbage();
        stats.watertight = mesh_is_closed(sub);
        stats.valid_mesh = CGAL::is_valid_polygon_mesh(sub);

        const std::string out_path =
            (fs::path(opts.output_dir) / segment_filename(segment_id, filename_width)).string();
        if (!CGAL::IO::write_polygon_mesh(out_path, sub)) {
            std::cerr << "Error: Cannot write segment mesh: " << out_path << std::endl;
            return 1;
        }

        ++segments_written;
        all_stats.push_back(stats);

        if (!stats.watertight || !stats.valid_mesh) {
            std::cout << "  segment " << segment_id << ": "
                      << stats.face_count << " faces, "
                      << stats.caps_applied << " caps"
                      << (stats.watertight ? "" : " [NOT WATERTIGHT]")
                      << (stats.valid_mesh ? "" : " [INVALID]")
                      << std::endl;
        }
    }

    const std::string log_path = (fs::path(opts.output_dir) / "partition_log.txt").string();
    std::ofstream log(log_path);
    if (log) {
        log << "# neuromesh-partition v1\n";
        log << "input_mesh " << opts.input_mesh << "\n";
        log << "seg_file " << opts.seg_file << "\n";
        log << "unique_cut_loops " << unique_loops.size() << "\n";
        log << "segments_written " << segments_written << "\n";
        log << "segments_skipped " << segments_skipped << "\n";
        log << "total_cap_failures " << total_cap_failures << "\n";
        log << "\n# segment_id faces caps cap_failures watertight valid\n";
        for (const SegmentStats& stats : all_stats) {
            log << stats.segment_id << " "
                << stats.face_count << " "
                << stats.caps_applied << " "
                << stats.cap_failures << " "
                << (stats.watertight ? 1 : 0) << " "
                << (stats.valid_mesh ? 1 : 0) << "\n";
        }
    }

    std::size_t watertight_count = 0;
    for (const SegmentStats& stats : all_stats) {
        if (stats.watertight) {
            ++watertight_count;
        }
    }

    std::cout << "\n=== Partition Summary ===" << std::endl;
    std::cout << "Unique cut loops:   " << unique_loops.size() << std::endl;
    std::cout << "Segments written:   " << segments_written << std::endl;
    std::cout << "Segments skipped:   " << segments_skipped << " (empty or below min-output-faces)" << std::endl;
    std::cout << "Watertight segments: " << watertight_count << " / " << segments_written << std::endl;
    std::cout << "Cap failures:       " << total_cap_failures << std::endl;
    std::cout << "Output directory:   " << opts.output_dir << std::endl;
    std::cout << "Log file:           " << log_path << std::endl;
    std::cout << "=========================" << std::endl;

    return (watertight_count < segments_written) ? 1 : 0;
}
