# neuromesh-cgal

C++ mesh processing tools using CGAL library for operations on 3D mesh data.

## Prerequisites

- CMake 3.12 or higher
- CGAL library
- Eigen3 3.2 or higher
- C++17 compatible compiler

### Installing Dependencies (Ubuntu/WSL)
```bash
sudo apt-get update
sudo apt-get install libcgal-dev libeigen3-dev cmake build-essential
```

## Project Structure

```
neuromesh-cgal/
├── src/                    # Source files
│   ├── mesh_simplify.cpp   # Mesh simplification (edge collapse)
│   ├── mesh_smooth.cpp     # Mesh smoothing
│   └── mesh_remesh.cpp     # Isotropic remeshing
├── data/                   # Input/output mesh files (.obj)
├── build/                  # Build directory (generated)
└── CMakeLists.txt          # CMake configuration
```

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### Mesh Simplification
Reduces mesh complexity by collapsing edges. Accepts either a target edge count or a fraction:
```bash
# Target edge count (integer >= 1)
./build/mesh_simplify data/input.obj data/output_simplified.obj 1000

# Target fraction (float 0-1, e.g., 0.5 = keep 50% of edges)
./build/mesh_simplify data/input.obj data/output_simplified.obj 0.5
```

### Mesh Smoothing
Applies shape smoothing with specified time step and iterations:
```bash
./build/mesh_smooth data/input.obj data/output_smooth.obj 0.0001 10
```

### Isotropic Remeshing
Remeshes to uniform edge lengths:
```bash
./build/mesh_remesh data/input.obj data/output_remeshed.obj 0.05
```

### Mesh Repair
Repairs meshes to ensure single component, no boundaries, and detects self-intersections:
```bash
./build/mesh_repair data/input.obj data/output_repaired.obj
```

**Repair operations performed:**
1. Stitches duplicate boundaries
2. Fixes non-manifold vertices
3. Keeps only largest connected component
4. Detects self-intersections (reports but doesn't auto-fix)
5. Fills all boundary holes
6. Removes degenerate faces

See `docs/repair_strategy.md` for detailed methodology.

## Adding New Operations

1. Create a new `.cpp` file in `src/`
2. Add executable and linking in `CMakeLists.txt`:
```cmake
add_executable(your_tool src/your_tool.cpp)
target_link_libraries(your_tool CGAL::CGAL)
```
3. Rebuild the project
