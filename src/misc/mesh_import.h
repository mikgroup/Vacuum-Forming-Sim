#ifndef CGL_UTIL_MESHIMPORT_H
#define CGL_UTIL_MESHIMPORT_H

#include <nanogui/nanogui.h>
#include "CGL/CGL.h"
#include "../collision/mesh.h"

using namespace nanogui;
using namespace std;

namespace CGL {
namespace Misc {

void import_mesh(Mesh &mesh, string filename);

} //Misc
} //CGL

#endif // CGL_UTIL_MESHIMPORT_H
