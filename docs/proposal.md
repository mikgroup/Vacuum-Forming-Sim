# Real-time Fracturing of 3D Objects

*Karthik Gopalan, Rachel Lee, James Lin*

We wish to implement a real-time, physically-based simulation for fracturing 3D objects. In the simulation, we should be able to apply force to a 3D object (through collision with another object) and have it shatter realistically.

## Problem Description

3D object fracturing is the simulation of an object breaking, deforming, or otherwise changing shape due to some external stimulus. It is commonly seen in mediums such as animated films or video games. Often times, however, the fracturing process is either done offline (taking anywhere between minutes and hours to perform a single time step) or isn’t physically accurate and doesn’t take into account the material of the object or the force being applied.

Common fracturing algorithms are designed for planar materials such as cloth or sheets. This does not account for the internal properties of 3D structures. For example, wood has anisotropic fibers that would heavily influence the way that it shatters.

The challenge in this project is in balancing performance with adaptive realism. The simulation should differ depending on the shape of the object, the material it’s made out of, and the direction/strength of the external force being applied.

## Goals and Deliverables

### Main goals

- Particles (springs) system for physically-based simulation
- Real-time simulation

### Secondary goals

The project has plenty of room for exploration, in which we list below:

- Surface reconstruction of particles
- Explore deformation of objects
- Explore alternative methods (e.g. Voronoi fracturing/finite tetrahedral elements)
- Compare to real fracturing of objects (e.g. ceramic bowls, glass)
- Create a high-quality render which is not real-time (path traced or similar)
- Open-source Unity asset store package (requires real-time)

## Schedule/Tasks

Week of | Tasks
--- | ---
04/02 | Project proposal, literature review, write renderer and I/O
04/09 | Particle-based simulation
04/16 | Particle-based simulation 2, alternative methods (e.g. Voronoi/finite tetrahedral elements)
04/23 | Milestone due (video, slides), optimizations, rendering
04/30 | Final deliverables (presentation, paper, website, etc.)


## Resources

- Ledoux, Hugo. "Computing the 3d Voronoi diagram robustly: An easy explanation." Voronoi Diagrams in Science and Engineering, 2007. ISVD'07. 4th International Symposium on. IEEE, 2007.
- Levine, Joshua A., et al. "A peridynamic perspective on spring-mass fracture." Proceedings of the ACM SIGGRAPH/Eurographics Symposium on Computer Animation. Eurographics Association, 2014.
- Pfaff, Tobias, et al. "Adaptive tearing and cracking of thin sheets." ACM Transactions on Graphics (TOG) 33.4 (2014): 110.
- Parker, Eric G., and James F. O'Brien. "Real-time deformation and fracture in a game environment." Proceedings of the 2009 ACM SIGGRAPH/Eurographics Symposium on Computer Animation. ACM, 2009.
- Smith, Jeffrey, Andrew Witkin, and David Baraff. "Fast and controllable simulation of the shattering of brittle objects." Computer Graphics Forum. Vol. 20. No. 2. Blackwell Publishers Ltd, 2001.
- Workman, Steven. "A Cracking Algorithm for Destructible 3D Objects." Undergraduate project Dissertation, Department of Computer Science, University of Sheffield 71 (2006).


