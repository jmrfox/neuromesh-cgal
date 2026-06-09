#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/shape_predicates.h>

#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "cli_common.hpp"
#include "defaults.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
typedef Surface_mesh::Face_index Face_index;
typedef Surface_mesh::Halfedge_index Halfedge_index;

namespace PMP = CGAL::Polygon_mesh_processing;

struct RepairStats {
    int initial_vertices = 0;
    int initial_faces = 0;
    int initial_components = 0;
    int components_removed = 0;
    bool had_self_intersections = false;
    int holes_filled = 0;
    int degenerate_faces_removed = 0;
    int final_vertices = 0;
    int final_faces = 0;
};

struct RepairOptions {
    std::string input;
    std::string output;
    std::size_t keep_components = neuromesh::RepairDefaults::keep_components;
    bool stitch_borders = true;
    bool fix_non_manifold = true;
    bool fill_holes = true;
    bool continue_on_self_intersections = false;
};

void print_usage(const char* program) {
    neuromesh::print_help(program, R"(Usage:
  mesh_repair --input <path> --output <path> [options]

Options:
  --input PATH                       Input mesh file (required)
  --output PATH                      Output mesh file (required)
  --keep-components N                Largest components to keep (default: 1)
  --no-stitch-borders                Skip duplicate boundary stitching
  --no-fix-nonmanifold               Skip non-manifold vertex repair
  --no-fill-holes                    Skip boundary hole filling
  --continue-on-self-intersections   Report self-intersections but exit 0
  --help                             Show this help

Example:
  mesh_repair --input data/in.obj --output data/out.obj
)");
}

void print_stats(const RepairStats& stats) {
    std::cout << "\n=== Repair Summary ===" << std::endl;
    std::cout << "Initial: " << stats.initial_vertices << " vertices, "
              << stats.initial_faces << " faces, "
              << stats.initial_components << " components" << std::endl;

    if (stats.components_removed > 0) {
        std::cout << "Removed " << stats.components_removed << " small components" << std::endl;
    }

    if (stats.had_self_intersections) {
        std::cout << "Detected and reported self-intersections" << std::endl;
    }

    if (stats.holes_filled > 0) {
        std::cout << "Filled " << stats.holes_filled << " boundary holes" << std::endl;
    }

    if (stats.degenerate_faces_removed > 0) {
        std::cout << "Removed " << stats.degenerate_faces_removed << " degenerate faces" << std::endl;
    }

    std::cout << "Final: " << stats.final_vertices << " vertices, "
              << stats.final_faces << " faces" << std::endl;
    std::cout << "======================" << std::endl;
}

bool parse_args(int argc, char** argv, RepairOptions& opts) {
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
            opts.input = argv[i];
        } else if (arg == "--output") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.output = argv[i];
        } else if (arg == "--keep-components") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.keep_components = static_cast<std::size_t>(std::stoul(argv[i]));
        } else if (arg == "--no-stitch-borders") {
            opts.stitch_borders = false;
        } else if (arg == "--no-fix-nonmanifold") {
            opts.fix_non_manifold = false;
        } else if (arg == "--no-fill-holes") {
            opts.fill_holes = false;
        } else if (arg == "--continue-on-self-intersections") {
            opts.continue_on_self_intersections = true;
        } else {
            neuromesh::print_error("Unknown flag: " + arg);
            return false;
        }
    }

    if (opts.input.empty() || opts.output.empty()) {
        neuromesh::print_error("--input and --output are required.");
        return false;
    }

    if (opts.keep_components < 1) {
        neuromesh::print_error("--keep-components must be >= 1.");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    RepairOptions opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    Surface_mesh mesh;
    RepairStats stats;

    std::cout << "Reading mesh from: " << opts.input << std::endl;

    std::vector<Point_3> points;
    std::vector<std::vector<std::size_t>> polygons;

    if (!CGAL::IO::read_OBJ(opts.input, points, polygons)) {
        neuromesh::print_error("Cannot read OBJ file: " + opts.input);
        return 1;
    }

    std::cout << "Read polygon soup: " << points.size() << " points, "
              << polygons.size() << " polygons" << std::endl;

    PMP::repair_polygon_soup(points, polygons);
    PMP::orient_polygon_soup(points, polygons);
    PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);

    stats.initial_vertices = static_cast<int>(mesh.number_of_vertices());
    stats.initial_faces = static_cast<int>(mesh.number_of_faces());

    std::cout << "Input mesh: " << stats.initial_vertices << " vertices, "
              << stats.initial_faces << " faces" << std::endl;

    std::cout << "\n[1/5] Pre-processing..." << std::endl;

    if (opts.stitch_borders) {
        std::cout << "  - Stitching borders..." << std::endl;
        PMP::stitch_borders(mesh);
    } else {
        std::cout << "  - Skipping border stitching" << std::endl;
    }

    if (opts.fix_non_manifold) {
        std::cout << "  - Checking for non-manifold vertices..." << std::endl;
        const std::size_t duplicated = PMP::duplicate_non_manifold_vertices(mesh);
        if (duplicated > 0) {
            std::cout << "    Fixed " << duplicated << " non-manifold vertices" << std::endl;
        }
    } else {
        std::cout << "  - Skipping non-manifold vertex repair" << std::endl;
    }

    std::cout << "\n[2/5] Selecting connected components..." << std::endl;

    Surface_mesh::Property_map<Face_index, std::size_t> fccmap =
        mesh.add_property_map<Face_index, std::size_t>("f:CC").first;
    stats.initial_components = static_cast<int>(PMP::connected_components(mesh, fccmap));

    std::cout << "  Found " << stats.initial_components << " connected component(s)" << std::endl;

    if (stats.initial_components > static_cast<int>(opts.keep_components)) {
        std::cout << "  Keeping " << opts.keep_components << " largest component(s)..." << std::endl;
        PMP::keep_largest_connected_components(mesh, opts.keep_components);
        stats.components_removed = stats.initial_components - static_cast<int>(opts.keep_components);
        std::cout << "  Removed " << stats.components_removed << " component(s)" << std::endl;
    }

    std::cout << "\n[3/5] Checking for self-intersections..." << std::endl;

    const bool has_self_intersections = PMP::does_self_intersect(mesh);

    if (has_self_intersections) {
        std::cout << "  WARNING: Mesh has self-intersections!" << std::endl;

        std::vector<std::pair<Face_index, Face_index>> intersected_faces;
        PMP::self_intersections(mesh, std::back_inserter(intersected_faces));
        std::cout << "  Found " << intersected_faces.size() << " pairs of intersecting faces" << std::endl;

        stats.had_self_intersections = true;

        std::cout << "  NOTE: Self-intersection resolution requires manual intervention or"
                  << std::endl;
        std::cout << "        specialized tools. Continuing with other repairs..." << std::endl;
    } else {
        std::cout << "  No self-intersections detected" << std::endl;
    }

    std::cout << "\n[4/5] Filling boundary holes..." << std::endl;

    if (opts.fill_holes) {
        std::vector<Halfedge_index> border_cycles;
        for (Halfedge_index h : mesh.halfedges()) {
            if (CGAL::is_border(h, mesh)) {
                border_cycles.push_back(h);
            }
        }

        if (border_cycles.empty()) {
            std::cout << "  No boundary holes found - mesh is already closed" << std::endl;
        } else {
            std::cout << "  Found boundary halfedges, identifying holes..." << std::endl;

            std::set<Halfedge_index> processed;

            for (Halfedge_index h : border_cycles) {
                if (processed.find(h) != processed.end()) {
                    continue;
                }

                Halfedge_index current = h;
                do {
                    processed.insert(current);
                    current = next(current, mesh);
                } while (current != h);

                std::cout << "  Filling hole..." << std::endl;
                std::vector<Face_index> patch_faces;
                PMP::triangulate_hole(mesh, h, std::back_inserter(patch_faces));

                ++stats.holes_filled;
                std::cout << "    Added " << patch_faces.size() << " faces" << std::endl;
            }

            std::cout << "  Filled " << stats.holes_filled << " hole(s)" << std::endl;
        }
    } else {
        std::cout << "  Skipping hole filling" << std::endl;
    }

    bool is_closed = true;
    for (Halfedge_index h : mesh.halfedges()) {
        if (CGAL::is_border(h, mesh)) {
            is_closed = false;
            break;
        }
    }

    if (!is_closed) {
        std::cout << "  WARNING: Mesh still has boundaries after hole filling!" << std::endl;
    } else {
        std::cout << "  Mesh is now closed (no boundaries)" << std::endl;
    }

    std::cout << "\n[5/5] Post-processing..." << std::endl;

    std::cout << "  - Removing degenerate faces..." << std::endl;
    stats.degenerate_faces_removed = static_cast<int>(PMP::remove_degenerate_faces(mesh));
    if (stats.degenerate_faces_removed > 0) {
        std::cout << "    Removed " << stats.degenerate_faces_removed << " degenerate face(s)" << std::endl;
    }

    mesh.collect_garbage();

    stats.final_vertices = static_cast<int>(mesh.number_of_vertices());
    stats.final_faces = static_cast<int>(mesh.number_of_faces());

    std::cout << "\n[Validation]" << std::endl;

    const bool is_valid = CGAL::is_valid_polygon_mesh(mesh);
    std::cout << "  Valid polygon mesh: " << (is_valid ? "YES" : "NO") << std::endl;

    const bool is_triangle = CGAL::is_triangle_mesh(mesh);
    std::cout << "  Triangle mesh: " << (is_triangle ? "YES" : "NO") << std::endl;

    std::cout << "  Closed (no boundaries): " << (is_closed ? "YES" : "NO") << std::endl;
    std::cout << "  Components kept: " << opts.keep_components << std::endl;
    std::cout << "  Self-intersections: "
              << (has_self_intersections ? "PRESENT (see warnings)" : "NONE") << std::endl;

    std::cout << "\nWriting repaired mesh to: " << opts.output << std::endl;
    if (!CGAL::IO::write_polygon_mesh(opts.output, mesh)) {
        neuromesh::print_error("Cannot write output file: " + opts.output);
        return 1;
    }

    print_stats(stats);

    if (!is_valid || !is_triangle || !is_closed) {
        std::cout << "\nWARNING: Repaired mesh may still have issues. Review validation results above."
                  << std::endl;
        return 1;
    }

    if (has_self_intersections && !opts.continue_on_self_intersections) {
        std::cout << "\nWARNING: Mesh still has self-intersections." << std::endl;
        return 1;
    }

    std::cout << "\nMesh repair completed successfully!" << std::endl;
    return 0;
}
