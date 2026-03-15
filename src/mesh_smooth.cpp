#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <iostream>
#include <string>
#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <input.obj> <output.obj> <time_step> <iterations>" << std::endl;
        std::cerr << "Example: " << argv[0] << " data/input.obj data/output.obj 0.0001 10" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    double time_step = std::stod(argv[3]);
    unsigned int iterations = std::stoi(argv[4]);

    Surface_mesh mesh;
    
    // Read OBJ file as polygon soup for better compatibility
    std::vector<Point_3> points;
    std::vector<std::vector<std::size_t>> polygons;
    
    if (!CGAL::IO::read_OBJ(input_file, points, polygons)) {
        std::cerr << "Error: Cannot read input file: " << input_file << std::endl;
        return 1;
    }
    
    // Repair and convert to mesh
    PMP::repair_polygon_soup(points, polygons);
    PMP::orient_polygon_soup(points, polygons);
    PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);

    std::cout << "Input mesh: " << mesh.number_of_vertices() << " vertices, " 
              << mesh.number_of_faces() << " faces" << std::endl;

    std::cout << "Smoothing mesh with time_step=" << time_step << ", iterations=" << iterations << std::endl;
    PMP::smooth_shape(mesh, time_step, CGAL::parameters::number_of_iterations(iterations));
    
    std::cout << "Smoothing complete" << std::endl;

    if (!CGAL::IO::write_polygon_mesh(output_file, mesh)) {
        std::cerr << "Error: Cannot write output file: " << output_file << std::endl;
        return 1;
    }

    std::cout << "Mesh saved to: " << output_file << std::endl;
    return 0;
}
