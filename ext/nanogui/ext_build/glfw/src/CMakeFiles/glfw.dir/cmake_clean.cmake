file(REMOVE_RECURSE
  "libglfw.3.2.dylib"
  "libglfw.3.dylib"
  "libglfw.dylib"
  "libglfw.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/glfw.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
