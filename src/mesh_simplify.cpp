#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_stop_predicate.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "cli_common.hpp"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace SMS = CGAL::Surface_mesh_simplification;
namespace PMP = CGAL::Polygon_mesh_processing;

struct SimplifyOptions {
    std::string input;
    std::string output;
    std::optional<double> fraction;
    std::optional<int> edge_count;
};

void print_usage(const char* program) {
    neuromesh::print_help(program, R"(Usage:
  mesh_simplify --input <path> --output <path> (--fraction F | --edge-count N)

Options:
  --input PATH         Input mesh file (required)
  --output PATH        Output mesh file (required)
  --fraction F         Keep fraction of original edges (0 < F < 1)
  --edge-count N       Target edge count (integer >= 1)
  --help               Show this help

Examples:
  mesh_simplify --input data/in.obj --output data/out.obj --fraction 0.5
  mesh_simplify --input data/in.obj --output data/out.obj --edge-count 1000
)");
}

bool parse_args(int argc, char** argv, SimplifyOptions& opts) {
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
        } else if (arg == "--fraction") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.fraction = std::stod(argv[i]);
        } else if (arg == "--edge-count") {
            if (!neuromesh::require_flag_value(i, argc, argv, arg.c_str())) {
                return false;
            }
            opts.edge_count = std::stoi(argv[i]);
        } else {
            neuromesh::print_error("Unknown flag: " + arg);
            return false;
        }
    }

    if (opts.input.empty() || opts.output.empty()) {
        neuromesh::print_error("--input and --output are required.");
        return false;
    }

    if (opts.fraction.has_value() == opts.edge_count.has_value()) {
        neuromesh::print_error("Specify exactly one of --fraction or --edge-count.");
        return false;
    }

    if (opts.fraction.has_value()) {
        const double value = opts.fraction.value();
        if (!(value > 0.0 && value < 1.0)) {
            neuromesh::print_error("--fraction must be between 0 and 1.");
            return false;
        }
    }

    if (opts.edge_count.has_value() && opts.edge_count.value() < 1) {
        neuromesh::print_error("--edge-count must be >= 1.");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    SimplifyOptions opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    Surface_mesh mesh;

    std::vector<Point_3> points;
    std::vector<std::vector<std::size_t>> polygons;

    if (!CGAL::IO::read_OBJ(opts.input, points, polygons)) {
        neuromesh::print_error("Cannot read input file: " + opts.input);
        return 1;
    }

    PMP::repair_polygon_soup(points, polygons);
    PMP::orient_polygon_soup(points, polygons);
    PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);

    std::cout << "Input mesh: " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_edges() << " edges, "
              << mesh.number_of_faces() << " faces" << std::endl;

    int target_edge_count = 0;
    if (opts.fraction.has_value()) {
        target_edge_count = static_cast<int>(mesh.number_of_edges() * opts.fraction.value());
        std::cout << "Target fraction: " << opts.fraction.value()
                  << " (" << target_edge_count << " edges)" << std::endl;
    } else {
        target_edge_count = opts.edge_count.value();
        std::cout << "Target edge count: " << target_edge_count << std::endl;
    }

    SMS::Edge_count_stop_predicate<Surface_mesh> stop(target_edge_count);

    std::cout << "Simplifying mesh to " << target_edge_count << " edges..." << std::endl;
    const int removed_edges = SMS::edge_collapse(mesh, stop);

    std::cout << "Removed " << removed_edges << " edges" << std::endl;
    std::cout << "Output mesh: " << mesh.number_of_vertices() << " vertices, "
              << mesh.number_of_edges() << " edges, "
              << mesh.number_of_faces() << " faces" << std::endl;

    if (!CGAL::IO::write_polygon_mesh(opts.output, mesh)) {
        neuromesh::print_error("Cannot write output file: " + opts.output);
        return 1;
    }

    std::cout << "Mesh saved to: " << opts.output << std::endl;
    return 0;
}
