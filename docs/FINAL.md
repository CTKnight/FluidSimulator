# CS 284A: Computer Graphics and Imaging, Spring 2020

# Final Project: Position Based Fluid Simulation and Surface Rendering

![Teaser](teaser.png)

#  Team Member

- Jiewen Lai
- Yanda Li
- Kailun Wan

# Links

- **[CS284A Final Report pdf](https://drive.google.com/file/d/14NBuwOkBv4B0xg0AGVYWsEsgpjXjXg6a/view?usp=sharing)**

- **[Final Video](https://youtu.be/SfTIv-HlWFM)**

- [Slides](https://docs.google.com/presentation/d/1VrIeeL3HWLHeoGgKl4LTh8TZWxva0XWIZIpgnCNel8A/edit?usp=sharing) 

- [GitHub Repo](https://github.com/CTKnight/FluidSimulator)

# Abstract

In the course, we learned how to make a simulation of cloth using the mass and spring-based system with some physical constraints and numerical integration. In this final project, we extend our knowledge to make the simulation of another popular topic-fluid. We want to know how the liquid particles work with each other with some physical constraints to create the liquid effect. We used the Position Based Fluids algorithm to simulate the liquid. In addition, we implemented the novel CUDA accelerator to simulate millions of particles in seconds and real-time 60 fps simulation for thousands of particles. At last, we applied the marching cube algorithm to reconstruct the surface meshes of the liquid and rendered the images and videos with Paraview to create realistic liquid effects for particles.

# Technical Approaches

We explained our technical approaches in detail in our final report. You can find the link above in the "Links" section above.

# Results

For our results, here are some rendered images for our liquid surfaces:

![Dam break](large_frame.png)
*A dam break scenario*

![Another dam break](one_frame.png)
*Another dam break scenario*

![1 million particle](million_particle_frame.png)
*A frame of simulating 1 millison particles*

![Dam break gif](large30images.gif)
*A gif of Dam break*

# Final remarks

You can consult [PROPOSAL.md](PROPOSAL.md), [MILESTONE.md](MILESTONE.md) and [BUILDING.md](BUILDING.md) in our repo for more information. Thank you, and hope you like our project!

