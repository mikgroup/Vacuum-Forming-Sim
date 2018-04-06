#ifndef CGL_UTIL_MESHDRAWING_H
#define CGL_UTIL_MESHDRAWING_H

#include <nanogui/nanogui.h>
#include "CGL/CGL.h"
#include "../collision/mesh.h"

using namespace nanogui;

namespace CGL {
namespace Misc {

void draw_mesh(Mesh &mesh, GLShader &shader);

} //Misc
} //CGL

#endif // CGL_UTIL_MESHDRAWING_H
