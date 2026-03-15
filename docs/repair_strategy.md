# Mesh Repair Strategy

## Objective
Create a robust mesh repair tool that ensures input meshes meet the following requirements:
1. **Single connected component** - Remove all but the largest component
2. **No self-intersections** - Detect and resolve geometric intersections
3. **No boundaries (holes)** - Fill all boundary cycles to create a closed mesh

**Important:** Topological holes (e.g., genus of a torus) are allowed and must be preserved.

## CGAL Functions Research Summary

### 1. Connected Components
**Goal:** Keep only the largest connected component

**Functions:**
- `CGAL::Polygon_mesh_processing::connected_components()` - Identifies all components
- `CGAL::Polygon_mesh_processing::keep_largest_connected_components(mesh, 1)` - Keeps only the largest component
- `CGAL::Polygon_mesh_processing::split_connected_components()` - Splits into separate meshes

**Strategy:** Use `keep_largest_connected_components(mesh, 1)` to retain only the largest component by face count.

**Include:** `<CGAL/Polygon_mesh_processing/connected_components.h>`

### 2. Self-Intersections
**Goal:** Detect and resolve self-intersecting triangles

**Detection Functions:**
- `CGAL::Polygon_mesh_processing::does_self_intersect(mesh)` - Boolean check
- `CGAL::Polygon_mesh_processing::self_intersections(mesh, output_iterator)` - Reports all pairs

**Resolution Approaches:**

#### Option A: Autorefine (for triangle soups)
- `CGAL::Polygon_mesh_processing::autorefine_triangle_soup()` - Refines at intersections
- **Limitation:** Works on triangle soups (point/polygon arrays), not directly on Surface_mesh
- **Use case:** Convert mesh → soup → autorefine → rebuild mesh

#### Option B: Corefinement
- `CGAL::Polygon_mesh_processing::corefine()` - Refines two meshes at their intersection
- **Limitation:** Requires two meshes; not designed for single mesh self-intersection repair

#### Option C: Remove problematic faces
- Detect self-intersecting face pairs
- Remove them (creates holes)
- Fill resulting holes
- **Risk:** May remove significant geometry

**Chosen Strategy:** 
1. First attempt: Use autorefine approach (convert to soup, autorefine with snap rounding, convert back)
2. Fallback: If autorefine fails or unavailable, detect and report self-intersections without automatic fix
3. The `apply_iterative_snap_rounding` option ensures no new self-intersections from coordinate rounding

**Include:** `<CGAL/Polygon_mesh_processing/self_intersections.h>`, `<CGAL/Polygon_mesh_processing/autorefinement.h>`

### 3. Boundary Holes
**Goal:** Fill all boundary cycles to create a closed mesh

**Detection Functions:**
- `CGAL::is_border(halfedge, mesh)` - Check if halfedge is on boundary
- Iterate through halfedges to find border cycles

**Filling Functions:**
- `CGAL::Polygon_mesh_processing::triangulate_hole(mesh, halfedge)` - Fills single hole
- `CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole()` - Fills with refinement and fairing
- Can iterate through all border halfedges to fill all holes

**Strategy:**
1. Detect all border halfedges
2. Group into boundary cycles
3. Fill each cycle using `triangulate_refine_and_fair_hole()` for better quality
4. Verify mesh is closed after filling

**Include:** `<CGAL/Polygon_mesh_processing/triangulate_hole.h>`, `<CGAL/Polygon_mesh_processing/border.h>`

### 4. Additional Repair Operations

#### Stitching (for duplicate vertices/edges)
- `CGAL::Polygon_mesh_processing::stitch_borders()` - Merges duplicate boundary edges
- **Use case:** Fix meshes with geometrically coincident but topologically separate boundaries

**Include:** `<CGAL/Polygon_mesh_processing/repair.h>`

#### Manifoldness
- `CGAL::Polygon_mesh_processing::is_non_manifold_vertex()` - Detect non-manifold vertices
- `CGAL::Polygon_mesh_processing::duplicate_non_manifold_vertices()` - Split non-manifold vertices

**Strategy:** Check and fix non-manifold vertices before other operations

#### Degenerate Faces
- `CGAL::Polygon_mesh_processing::remove_almost_degenerate_faces()` - Removes badly shaped triangles
- **Parameters:** `cap_threshold`, `needle_threshold` for quality control

**Strategy:** Apply after hole filling to clean up generated triangles

## Repair Pipeline Order

The order of operations is critical for robust repair:

```
1. PRE-PROCESSING
   a. Stitch borders (merge duplicate boundaries)
   b. Fix non-manifold vertices
   c. Remove degenerate faces

2. COMPONENT SELECTION
   a. Identify connected components
   b. Keep only largest component

3. SELF-INTERSECTION RESOLUTION
   a. Check for self-intersections
   b. If found: attempt autorefine approach
   c. Verify no self-intersections remain

4. BOUNDARY FILLING
   a. Detect all boundary cycles
   b. Fill each hole with triangulate_refine_and_fair_hole()
   c. Verify mesh is closed (no border halfedges)

5. POST-PROCESSING
   a. Remove any new degenerate faces
   b. Optional: light smoothing to improve quality
   c. Verify final mesh validity

6. VALIDATION
   a. Check is_valid_polygon_mesh()
   b. Check is_triangle_mesh()
   c. Verify no boundaries
   d. Verify single component
   e. Verify no self-intersections
```

## Implementation Considerations

### Kernel Choice
- Use `CGAL::Exact_predicates_inexact_constructions_kernel` for robustness
- Exact predicates prevent topology errors
- Inexact constructions are faster than exact

### Error Handling
- Each step should report success/failure
- Provide detailed diagnostics for failures
- Allow user to see what was repaired

### Performance
- Large meshes may be slow
- Consider progress reporting for long operations
- Hole filling is O(n³) for n boundary vertices

### Limitations
- Very complex self-intersections may not be fully resolvable
- Large holes may produce poor-quality triangulations
- Some meshes may be fundamentally unrepairable

## Output Options

The tool should provide:
1. **Repaired mesh** - The fixed output mesh
2. **Repair report** - What operations were performed
3. **Statistics** - Before/after vertex/face counts
4. **Warnings** - Any issues that couldn't be fully resolved

## Testing Strategy

Test with meshes having:
- Multiple components
- Self-intersections
- Boundary holes
- Combination of all issues
- Already-valid meshes (should pass through unchanged)

## References

- [CGAL Polygon Mesh Processing Manual](https://doc.cgal.org/latest/Polygon_mesh_processing/index.html)
- [Combinatorial Repair](https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#title36)
- [Hole Filling](https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#title27)
- [Self-Intersections](https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#title31)
- [Connected Components](https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#title50)
