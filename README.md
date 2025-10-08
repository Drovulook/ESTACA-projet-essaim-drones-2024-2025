# Simulation and control of a drone swarm on Matlab

This project was developed by a team of 6 students at ESTACA (École Supérieure des Techniques Aéronautiques et de Construction Automobile) engineering school during school the 2024-2025 academic year.
It aims to simulate and control a collaborative drone swarm using MATLAB, drawing inspiration from the collective behaviors observed in fish schools. 
The simulation incorporates key aspects such as drone physical caracteristics, behavioral laws of the swarm, obstacle avoidance, etc.

The project was conducted at ESTACA in partnership with Safran.

All GitHub contributors (B0oster, Baptiste-FM, Drovulook, maxcql and Favysin) were active members of the project.

# Problem statement 

The goal is to coordinate a set of drones to autonomously and collaboratively complete complex missions, while allowing the user to guide the swarm through high-level decisions (at the swarm level, rather than individually for each drone). This approach enables robust group behaviors for applications such as surveillance or mapping.

# Specifications

The simulated drones are aerial, with:
- Fixed-wing or rotary-wing (quadcopters) configurations,
- Limited flight autonomy, type-dependent (variable performance in speed, endurance).
- Trajectory piloting via velocity vector; each drone knows its state (position, velocity).
- Scalable swarm (>20 drones, mixed types).
- Distance-based observation (triangulation for accuracy).

# Behavioral laws

The swarm behaviour is modeled based on natural swarms, particularly schools of fish, to promote cohesion, alignment, and separation. 
At each simulation time ste, each drone identifies its nearest neighbors using a Delaunay triangulation approach. 
Then, A new velocity vector is assigned to each drone, accounting for physical constraints and based on the positions of its nearest neighbors. 
This ensures:
- avoidance of overly close drones (to limit collisions),
- alignment with the overall swarm movement,
- cohesion to remain within the swarm formation.
2 additional guidance components applies high-level user directives to steer the entire swarm, while ensuring obstacle avoidance.

# Simulation settings

The simulation includes numerous configurable parameters, such as: 
- number of drones and individual characteristics (fixed-wing/quadcopter, min/max speed, turn rate, mass, autonomy, etc.),
- target zones that drones must overfly.
- obstacles or no-fly zones to avoid.
- behavioral model parameters for the swarm (e.g., cohesion strength, alignment factors).

# User interface (UI)

The program includes a MATLAB-based interface for real-time visualization, control and monitoring.
After configuring simulation settings, the user can start/pause or reset the simulation, and view details on each drone, obstacles, targets, critical alerts (like a collision between drones), etc.

# Display

**Here is a glimpse of the simulation.**

view at the start of the simulation with UI :

![view at the start of the simulation with UI](screenshot1.png)

view of drone movements during simulation :

![view of the movement of drones during simulation](screenshot2.png)

(Note: These are early builds; the UI displays trajectories, swarm cohesion, and real-time metrics like FPS for debugging.)
