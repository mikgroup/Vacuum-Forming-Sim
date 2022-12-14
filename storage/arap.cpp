#include <CGAL/Simple_cartesian.h>

#include <CGAL/Surface_mesh.h>

#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>

#include <CGAL/boost/graph/Seam_mesh.h>
#include <CGAL/boost/graph/graph_traits_Seam_mesh.h>

#include <CGAL/Polygon_mesh_processing/measure.h>

#include <iostream>
#include <fstream>

typedef CGAL::Simple_cartesian<double>          Kernel;
typedef Kernel::Point_2                         Point_2;
typedef Kernel::Point_3                         Point_3;
typedef CGAL::Surface_mesh<Kernel::Point_3>     SurfaceMesh;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor     vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor   halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor       face_descriptor;

namespace SMP = CGAL::Surface_mesh_parameterization;

int main(int argc, char * argv[])
{
  std::ifstream in((argc>1) ? argv[1] : "data/nefertiti.off");
  if(!in) {
    std::cerr << "Problem loading the input data" << std::endl;
    return 1;
  }

  SurfaceMesh sm;
  in >> sm;

  // a halfedge on the border
  halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(sm).first;

  // The UV property map that holds the parameterized values
  typedef SurfaceMesh::Property_map<vertex_descriptor, Point_2>  UV_pmap;
  UV_pmap uv_map = sm.add_property_map<vertex_descriptor, Point_2>("h:uv").first;

//  SMP::parameterize(sm, bhd, uv_map);

	float lambda = 100000;

  typedef SMP::ARAP_parameterizer_3<SurfaceMesh> Parameterizer;
	SMP::parameterize(sm, Parameterizer(lambda), bhd, uv_map);

  std::ofstream out("result.off");
  SMP::IO::output_uvmap_to_off(sm, bhd, uv_map, out);

	std::ofstream out2("result2.off");
	BOOST_FOREACH(vertex_descriptor vd, sm.vertices()){
		out2 << get(uv_map, vd) << std::endl;
	}

	out2.close();

  return 0;

}
