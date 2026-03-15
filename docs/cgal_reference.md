# CGAL API Reference

Quick reference for CGAL functions used in this project. All syntax verified against CGAL 6.1.1 documentation.

**📚 Local Documentation:** Full CGAL 6.1.1 HTML documentation is available in `cgal_docs_html/` directory for detailed reference.

## Best Practices for OBJ Files

### Complete OBJ Reading Workflow (Recommended)

This workflow is the most robust way to read OBJ files in CGAL:

```cpp
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

// 1. Read OBJ as polygon soup
std::vector<Point_3> points;
std::vector<std::vector<std::size_t>> polygons;

if (!CGAL::IO::read_OBJ(input_file, points, polygons)) {
    std::cerr << "Error reading OBJ file" << std::endl;
    return 1;
}

// 2. Repair the polygon soup (removes duplicates, isolated points)
PMP::repair_polygon_soup(points, polygons);

// 3. Orient polygons consistently
PMP::orient_polygon_soup(points, polygons);

// 4. Convert to mesh
Surface_mesh mesh;
PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
```

**Why this workflow:**
- Handles OBJ files with various format variations (CRLF vs LF line endings, etc.)
- `repair_polygon_soup()` fixes common issues before mesh creation
- `orient_polygon_soup()` ensures consistent face orientation
- More robust than `CGAL::IO::read_polygon_mesh()` which may fail on some OBJ files

**Key Function Behaviors:**
- `CGAL::IO::read_OBJ()` returns `bool` (true on success)
- `repair_polygon_soup()` returns `void`
- `orient_polygon_soup()` returns `void`
- `polygon_soup_to_polygon_mesh()` returns `void` (NOT bool!)

---

## Common Setup

### Includes and Typedefs
```cpp
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <iostream>
#include <fstream>
#include <string>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
```

### File I/O

#### Method 1: OBJ-Specific I/O (Recommended for OBJ files)

**Includes:**
```cpp
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <vector>
```

**Read OBJ as Polygon Soup:**
```cpp
// Read OBJ file as polygon soup (most robust for OBJ files)
std::vector<Point_3> points;
std::vector<std::vector<std::size_t>> polygons;

if (!CGAL::IO::read_OBJ(input_file, points, polygons)) {
    std::cerr << "Error: Cannot read OBJ file" << std::endl;
    return 1;
}
```

**⚠️ Important:** `CGAL::IO::read_OBJ()` **appends** data to the vectors - it does NOT clear them first. If you're reading multiple files or reusing vectors, clear them first:
```cpp
points.clear();
polygons.clear();
CGAL::IO::read_OBJ(input_file, points, polygons);
```

// Repair and orient the polygon soup
PMP::repair_polygon_soup(points, polygons);
PMP::orient_polygon_soup(points, polygons);

// Convert to polygon mesh
Surface_mesh mesh;
PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
```

**Write OBJ:**
```cpp
if (!CGAL::IO::write_polygon_mesh("output.obj", mesh)) {
    std::cerr << "Error writing file" << std::endl;
    return 1;
}
```

**Why use this method:**
- `CGAL::IO::read_OBJ()` handles various OBJ format variations better
- Works with files that have different line endings (CRLF vs LF)
- `repair_polygon_soup()` fixes duplicate points, duplicate polygons, and isolated points
- `orient_polygon_soup()` ensures consistent face orientation
- More robust than direct `read_polygon_mesh()` for problematic OBJ files

#### Method 2: Generic Polygon Mesh I/O

**Include:**
```cpp
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
```

**Read/Write Mesh:**
```cpp
// Read mesh (supports .obj, .off, .stl, .ply, etc.)
Surface_mesh mesh;
if (!CGAL::IO::read_polygon_mesh("input.obj", mesh)) {
    std::cerr << "Error reading file" << std::endl;
    return 1;
}

// Write mesh
if (!CGAL::IO::write_polygon_mesh("output.obj", mesh)) {
    std::cerr << "Error writing file" << std::endl;
    return 1;
}
```

**Note:** This method may fail on some OBJ files with format variations. Use Method 1 for better OBJ compatibility.

### Mesh Properties
```cpp
mesh.number_of_vertices()  // Returns number of vertices
mesh.number_of_edges()     // Returns number of edges
mesh.number_of_faces()     // Returns number of faces
```

---

## Surface Mesh Simplification

### Includes
```cpp
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_stop_predicate.h>

namespace SMS = CGAL::Surface_mesh_simplification;
```

### Edge Collapse Simplification
```cpp
// Stop when target edge count is reached
SMS::Edge_count_stop_predicate<Surface_mesh> stop(target_edge_count);

// Perform simplification
int removed_edges = SMS::edge_collapse(mesh, stop);
```

**Parameters:**
- `mesh`: Surface_mesh to simplify (modified in place)
- `stop`: Stop predicate (e.g., Edge_count_stop_predicate)

**Returns:** Number of edges removed

**Note:** `Count_stop_predicate` is DEPRECATED. Use `Edge_count_stop_predicate` instead.

### Other Stop Predicates
```cpp
// Stop at edge count ratio
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
SMS::Edge_count_ratio_stop_predicate<Surface_mesh> stop(0.5); // Keep 50% of edges
```

---

## Polygon Mesh Processing

### Includes
```cpp
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>

namespace PMP = CGAL::Polygon_mesh_processing;
```

### Shape Smoothing

**⚠️ REQUIRES EIGEN3 3.2+**

```cpp
// Smooth using mean curvature flow
double time_step = 0.0001;  // Amount vertices can move
unsigned int iterations = 10;

PMP::smooth_shape(mesh, time_step, 
    CGAL::parameters::number_of_iterations(iterations));
```

**Parameters:**
- `mesh`: Surface_mesh to smooth (modified in place)
- `time_step`: Time step for mean curvature flow (typically 0.0001 to 0.001)
- Named parameters:
  - `number_of_iterations`: Number of smoothing iterations (default: 1)
  - `vertex_is_constrained_map`: Property map for constrained vertices

**Effect:** Smooths shape using mean curvature flow, tends to shrink surface

**CMake Setup:**
```cmake
find_package(CGAL REQUIRED)
find_package(Eigen3 3.2 REQUIRED NO_MODULE)
add_definitions(-DCGAL_EIGEN3_ENABLED)
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(your_tool CGAL::CGAL)
```

### Isotropic Remeshing
```cpp
// Remesh to uniform edge length
double target_edge_length = 0.05;

PMP::isotropic_remeshing(
    faces(mesh),
    target_edge_length,
    mesh,
    CGAL::parameters::number_of_iterations(3)
);
```

**Parameters:**
- `faces(mesh)`: Face range to remesh
- `target_edge_length`: Target uniform edge length
- `mesh`: Surface_mesh (modified in place)
- Named parameters:
  - `number_of_iterations`: Number of remeshing iterations (default: 1)
  - `protect_constraints`: Protect boundary edges (default: false)
  - `edge_is_constrained_map`: Property map for constrained edges
  - `vertex_is_constrained_map`: Property map for constrained vertices

**Effect:** Creates uniform edge length distribution through splits, collapses, flips, and smoothing

### Other PMP Functions

#### Mesh Smoothing (Tangential Relaxation)
```cpp
#include <CGAL/Polygon_mesh_processing/tangential_relaxation.h>

PMP::tangential_relaxation(
    vertices(mesh),
    mesh,
    CGAL::parameters::number_of_iterations(5)
);
```

#### Angle and Area Smoothing
```cpp
#include <CGAL/Polygon_mesh_processing/angle_and_area_smoothing.h>

PMP::angle_and_area_smoothing(
    mesh,
    CGAL::parameters::number_of_iterations(10)
        .use_area_smoothing(true)
        .use_angle_smoothing(true)
);
```

---

## Named Parameters Pattern

CGAL uses named parameters extensively:

```cpp
// Single parameter
CGAL::parameters::number_of_iterations(10)

// Chained parameters
CGAL::parameters::number_of_iterations(10)
    .vertex_is_constrained_map(vcmap)
    .protect_constraints(true)
```

Common named parameters:
- `number_of_iterations(n)`: Number of iterations
- `vertex_point_map(vpm)`: Vertex point property map
- `vertex_is_constrained_map(vcm)`: Constrained vertex map
- `edge_is_constrained_map(ecm)`: Constrained edge map
- `protect_constraints(bool)`: Protect boundary/constrained edges
- `geom_traits(traits)`: Geometric traits class

---

## Common Patterns

### Constraining Border Vertices
```cpp
#include <CGAL/Polygon_mesh_processing/border.h>

std::set<Surface_mesh::Vertex_index> constrained_vertices;
for(auto v : vertices(mesh)) {
    if(CGAL::is_border(v, mesh)) {
        constrained_vertices.insert(v);
    }
}

CGAL::Boolean_property_map<std::set<Surface_mesh::Vertex_index>> vcmap(constrained_vertices);
```

### Checking Mesh Validity
```cpp
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

bool valid = CGAL::is_triangle_mesh(mesh);
bool has_self_intersections = PMP::does_self_intersect(mesh);
```

---

## Mesh Repair Operations

### Includes
```cpp
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
```

### Polygon Soup Repair Functions

**Include:**
```cpp
#include <CGAL/Polygon_mesh_processing/repair.h>
```

#### repair_polygon_soup()
```cpp
template<typename PointRange, typename PolygonRange>
void repair_polygon_soup(PointRange& points, PolygonRange& polygons);
```

**Purpose:** Repairs a polygon soup by applying multiple cleaning operations:
- Merges duplicate points
- Merges duplicate polygons  
- Removes isolated points (not referenced by any polygon)

**Parameters:**
- `points`: Vector of Point_3 (modified in place)
- `polygons`: Vector of vectors of indices (modified in place)

**Returns:** void

**Usage:**
```cpp
std::vector<Point_3> points;
std::vector<std::vector<std::size_t>> polygons;
CGAL::IO::read_OBJ("input.obj", points, polygons);
PMP::repair_polygon_soup(points, polygons);
```

**Important:** Always call this before `orient_polygon_soup()` and `polygon_soup_to_polygon_mesh()`.

#### orient_polygon_soup()
```cpp
template<typename PointRange, typename PolygonRange>
void orient_polygon_soup(PointRange& points, PolygonRange& polygons);
```

**Purpose:** Orients polygons consistently so they all face outward or inward.

**Parameters:**
- `points`: Vector of Point_3
- `polygons`: Vector of vectors of indices (modified in place to reverse orientation where needed)

**Returns:** void

**Usage:**
```cpp
PMP::orient_polygon_soup(points, polygons);
```

**Note:** Call after `repair_polygon_soup()` and before `polygon_soup_to_polygon_mesh()`.

#### polygon_soup_to_polygon_mesh()
```cpp
template<typename PolygonMesh, typename PointRange, typename PolygonRange>
void polygon_soup_to_polygon_mesh(
    const PointRange& points,
    const PolygonRange& polygons,
    PolygonMesh& mesh);
```

**Purpose:** Converts a polygon soup (points + polygon indices) into a proper polygon mesh data structure.

**Parameters:**
- `points`: Vector of Point_3 (const)
- `polygons`: Vector of vectors of indices (const)
- `mesh`: Output Surface_mesh (cleared and filled)

**Returns:** void (does not return bool)

**Usage:**
```cpp
Surface_mesh mesh;
PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
```

**Important:** This function returns void, not bool. It will always succeed but may produce an invalid mesh if the soup is malformed.

### Stitch Borders
```cpp
// Merge duplicate boundary edges
PMP::stitch_borders(mesh);
```

**Include:**
```cpp
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
```

**Signature:**
```cpp
template<typename PolygonMesh>
void stitch_borders(PolygonMesh& mesh);
```

**Effect:** Stitches together geometrically identical but topologically separate boundary edges.

**Returns:** void

**When to use:** After converting polygon soup to mesh, or when you have a mesh with duplicate boundary edges that should be merged.

### Fix Non-Manifold Vertices
```cpp
// Split non-manifold vertices
std::size_t duplicated = PMP::duplicate_non_manifold_vertices(mesh);
```

**Returns:** Number of vertices that were duplicated

**Effect:** Creates manifold surface by splitting non-manifold vertices into separate vertices.

### Connected Components
```cpp
// Count components
Surface_mesh::Property_map<Face_index, std::size_t> fccmap = 
    mesh.add_property_map<Face_index, std::size_t>("f:CC").first;
std::size_t num_components = PMP::connected_components(mesh, fccmap);

// Keep only largest component
PMP::keep_largest_connected_components(mesh, 1);
```

**Parameters:**
- `mesh`: Surface_mesh (modified in place)
- `fccmap`: Property map to store component IDs per face
- Second parameter to `keep_largest_connected_components`: number of components to keep

### Self-Intersection Detection
```cpp
// Check if mesh has self-intersections
bool has_si = PMP::does_self_intersect(mesh);

// Get all intersecting face pairs
std::vector<std::pair<Face_index, Face_index>> intersected_faces;
PMP::self_intersections(mesh, std::back_inserter(intersected_faces));
```

**Note:** Detection only - automatic resolution requires specialized approaches (see `docs/repair_strategy.md`).

### Hole Filling

**Include:**
```cpp
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
```

#### Detecting Boundaries
```cpp
// Check if halfedge is on boundary
bool is_boundary = CGAL::is_border(halfedge, mesh);

// Iterate through all border halfedges
for (Halfedge_index h : mesh.halfedges()) {
    if (CGAL::is_border(h, mesh)) {
        // h is a border halfedge
    }
}
```

**Note:** `CGAL::is_border()` is in the BGL namespace, not PMP.

#### triangulate_hole()
```cpp
template<typename PolygonMesh, typename OutputIterator>
OutputIterator triangulate_hole(
    PolygonMesh& mesh,
    typename boost::graph_traits<PolygonMesh>::halfedge_descriptor border_halfedge,
    OutputIterator out);
```

**Purpose:** Fills a hole in a mesh by triangulating the boundary.

**Parameters:**
- `mesh`: The polygon mesh (modified in place)
- `border_halfedge`: A halfedge on the boundary of the hole to fill
- `out`: Output iterator to receive the face indices of the created patch

**Returns:** The output iterator (after insertion)

**Usage:**
```cpp
std::vector<Face_index> patch_faces;
PMP::triangulate_hole(mesh, border_halfedge, std::back_inserter(patch_faces));
std::cout << "Added " << patch_faces.size() << " faces" << std::endl;
```

**Algorithm:** Uses a greedy approach that minimizes a quality metric based on triangle shape and dihedral angles.

**Important Notes:**
- This function is marked as deprecated in CGAL 6.1+
- May return 0 faces if the hole cannot be triangulated (e.g., degenerate cases)
- For better quality, consider using `triangulate_refine_and_fair_hole()` (though also deprecated)
- The hole boundary must form a simple closed curve

**Handling Multiple Holes:**
```cpp
std::set<Halfedge_index> processed;

for (Halfedge_index h : mesh.halfedges()) {
    if (!CGAL::is_border(h, mesh) || processed.count(h)) continue;
    
    // Mark all halfedges in this boundary cycle
    Halfedge_index current = h;
    do {
        processed.insert(current);
        current = next(current, mesh);
    } while (current != h);
    
    // Fill the hole
    std::vector<Face_index> patch;
    PMP::triangulate_hole(mesh, h, std::back_inserter(patch));
}
```

### Remove Degenerate Faces
```cpp
// Remove badly shaped faces
std::size_t removed = PMP::remove_degenerate_faces(mesh);
```

**Returns:** Number of faces removed

**Effect:** Removes faces with nearly collinear vertices or zero area.

### Validation
```cpp
// Check mesh validity
bool is_valid = CGAL::is_valid_polygon_mesh(mesh);
bool is_triangle = CGAL::is_triangle_mesh(mesh);
```

---

## Important Notes

1. **Kernel Choice**: `Simple_cartesian<double>` is fine for most operations. Use `Exact_predicates_inexact_constructions_kernel` for robustness with complex operations.

2. **File I/O**: Stream operators (`<<`, `>>`) work with `.obj` and `.off` files. For other formats, use `CGAL::IO::read_polygon_mesh()` and `CGAL::IO::write_polygon_mesh()`.

3. **Error Handling**: Always check file operations and mesh validity before processing.

4. **Performance**: CGAL operations can be slow on large meshes. Consider progress callbacks for long operations.

---

## References

- [CGAL Documentation](https://doc.cgal.org/latest/Manual/index.html)
- [Surface Mesh Simplification](https://doc.cgal.org/latest/Surface_mesh_simplification/index.html)
- [Polygon Mesh Processing](https://doc.cgal.org/latest/Polygon_mesh_processing/index.html)
- [Surface Mesh](https://doc.cgal.org/latest/Surface_mesh/index.html)
- [Mesh Repair](https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#title36)
