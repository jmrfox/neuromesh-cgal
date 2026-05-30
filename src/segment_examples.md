# 1. COMPUTE SDF VALUES
```c++
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
 
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
 
#include <iostream>
#include <fstream>
 
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;
 
int main()
{
  // create and read Polyhedron
  Polyhedron mesh;
  std::ifstream input(CGAL::data_file_path("meshes/cactus.off"));
  if ( !input || !(input >> mesh) || mesh.empty() || ( !CGAL::is_triangle_mesh(mesh)) )
  {
    std::cerr << "Input is not a triangle mesh" << std::endl;
    return EXIT_FAILURE;
  }
 
  // create a property-map
  typedef std::map<face_descriptor, double> Face_double_map;
  Face_double_map internal_map;
  boost::associative_property_map<Face_double_map> sdf_property_map(internal_map);
 
  // compute SDF values
  std::pair<double, double> min_max_sdf = CGAL::sdf_values(mesh, sdf_property_map);
 
  // It is possible to compute the raw SDF values and post-process them using
  // the following lines:
  // const std::size_t number_of_rays = 25;  // cast 25 rays per face
  // const double cone_angle = 2.0 / 3.0 * CGAL_PI; // set cone opening-angle
  // CGAL::sdf_values(mesh, sdf_property_map, cone_angle, number_of_rays, false);
  // std::pair<double, double> min_max_sdf =
  //  CGAL::sdf_values_postprocessing(mesh, sdf_property_map);
 
  // print minimum & maximum SDF values
  std::cout << "minimum SDF: " << min_max_sdf.first
            << " maximum SDF: " << min_max_sdf.second << std::endl;
 
  // print SDF values
  for(face_descriptor f : faces(mesh))
      std::cout << sdf_property_map[f] << " ";
 
  std::cout << std::endl;
  return EXIT_SUCCESS;
}
```

# 2.0 Segmentation from SDF Values

```c++
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
 
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
 
#include <iostream>
#include <fstream>
 
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;
 
int main()
{
    // create and read Polyhedron
    Polyhedron mesh;
    std::ifstream input(CGAL::data_file_path("meshes/cactus.off"));
    if ( !input || !(input >> mesh) || mesh.empty()  || ( !CGAL::is_triangle_mesh(mesh)))
    {
      std::cerr << "Input is not a triangle mesh." << std::endl;
      return EXIT_FAILURE;
    }
 
    // create a property-map for SDF values
    typedef std::map<face_descriptor, double> Facet_double_map;
    Facet_double_map internal_sdf_map;
    boost::associative_property_map<Facet_double_map> sdf_property_map(internal_sdf_map);
 
    // compute SDF values using default parameters for number of rays, and cone angle
    CGAL::sdf_values(mesh, sdf_property_map);
 
    // create a property-map for segment-ids
    typedef std::map<face_descriptor, std::size_t> Facet_int_map;
    Facet_int_map internal_segment_map;
    boost::associative_property_map<Facet_int_map> segment_property_map(internal_segment_map);
 
    // segment the mesh using default parameters for number of levels, and smoothing lambda
    // Any other scalar values can be used instead of using SDF values computed using the CGAL function
    std::size_t number_of_segments = CGAL::segmentation_from_sdf_values(mesh, sdf_property_map, segment_property_map);
 
    std::cout << "Number of segments: " << number_of_segments << std::endl;
    // print segment-ids
    for(face_descriptor f : faces(mesh)) {
        // ids are between [0, number_of_segments -1]
        std::cout << segment_property_map[f] << " ";
    }
    std::cout << std::endl;
 
    const std::size_t number_of_clusters = 4;       // use 4 clusters in soft clustering
    const double smoothing_lambda = 0.3;  // importance of surface features, suggested to be in-between [0,1]
 
    // Note that we can use the same SDF values (sdf_property_map) over and over again for segmentation.
    // This feature is relevant for segmenting the mesh several times with different parameters.
    CGAL::segmentation_from_sdf_values(
      mesh, sdf_property_map, segment_property_map, number_of_clusters, smoothing_lambda);
 
    return EXIT_SUCCESS;
}
```

# 2.1 Computation of SDF Values and Segmentation

```c++
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
 
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
 
#include <iostream>
#include <fstream>
 
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;
 
int main()
{
  // create and read Polyhedron
  Polyhedron mesh;
  std::ifstream input(CGAL::data_file_path("meshes/cactus.off"));
  if ( !input || !(input >> mesh) || mesh.empty() || ( !CGAL::is_triangle_mesh(mesh)) )
  {
    std::cerr << "Input is not a triangle mesh" << std::endl;
    return EXIT_FAILURE;
  }
 
  // create a property-map for segment-ids
  typedef std::map<face_descriptor, std::size_t> Face_int_map;
  Face_int_map internal_segment_map;
  boost::associative_property_map<Face_int_map> segment_property_map(internal_segment_map);
 
  // calculate SDF values and segment the mesh using default parameters.
  std::size_t number_of_segments = CGAL::segmentation_via_sdf_values(mesh, segment_property_map);
 
  std::cout << "Number of segments: " << number_of_segments << std::endl;
 
  // print segment-ids
  for(face_descriptor f : faces(mesh) ) {
      std::cout << segment_property_map[f] << " ";
  }
  std::cout << std::endl;
  return EXIT_SUCCESS;
}
```

# 3. Using a Polyhedron with an ID per Facet

```c++
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polyhedron_3.h>
 
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
 
#include <iostream>
#include <fstream>
 
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3>  Polyhedron;
typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor;
 
// Property map associating a face with an integer as id to an
// element in a vector stored internally
template<class ValueType>
struct Face_with_id_pmap
    : public boost::put_get_helper<ValueType&,
             Face_with_id_pmap<ValueType> >
{
    typedef face_descriptor key_type;
    typedef ValueType value_type;
    typedef value_type& reference;
    typedef boost::lvalue_property_map_tag category;
 
    Face_with_id_pmap(
      std::vector<ValueType>& internal_vector
    ) : internal_vector(internal_vector) { }
 
    reference operator[](key_type key) const
    { return internal_vector[key->id()]; }
private:
    std::vector<ValueType>& internal_vector;
};
 
int main()
{
    // create and read Polyhedron
    Polyhedron mesh;
    std::ifstream input(CGAL::data_file_path("meshes/cactus.off"));
    if ( !input || !(input >> mesh) || mesh.empty() || ( !CGAL::is_triangle_mesh(mesh)) ) {
      std::cerr << "Input is not a triangle mesh" << std::endl;
      return EXIT_FAILURE;
    }
 
    // assign id field for each face
    std::size_t face_id = 0;
    for(face_descriptor f : faces( mesh) ) {
        f->id() = face_id++;
    }
 
    // create a property-map for SDF values
    std::vector<double> sdf_values(num_faces(mesh));
    Face_with_id_pmap<double> sdf_property_map(sdf_values);
 
    CGAL::sdf_values(mesh, sdf_property_map);
 
    // access SDF values (with constant-complexity)
    for(face_descriptor f : faces(mesh)) {
        std::cout << sdf_property_map[f] << " ";
    }
    std::cout << std::endl;
 
    // create a property-map for segment-ids
    std::vector<std::size_t> segment_ids(num_faces(mesh));
    Face_with_id_pmap<std::size_t> segment_property_map(segment_ids);
 
    CGAL::segmentation_from_sdf_values(mesh, sdf_property_map, segment_property_map);
 
    // access segment-ids (with constant-complexity)
    for(face_descriptor f : faces(mesh)) {
        std::cout << segment_property_map[f] << " ";
    }
    std::cout << std::endl;
    return EXIT_SUCCESS;
}
```
