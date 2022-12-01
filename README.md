# Vacuum Forming Simulation

# Setup
* `git submodule init && git submodule update`
* Download the latest nanogui from the source and download the latest embree from the source
* `mkdir build && cd build && cmake .. -DEMBREE_TUTORIAL=OFF -DNANOGUI_BUILD_OPTION=OFF`

# Notes:
* Must run from the scene directory since files are defined relative to there
* Use `../clothsim -f <file>`

# Acknowledgement
Ren Ng and CS184 staff for the project template.
