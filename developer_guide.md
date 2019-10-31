# **Jetleg Developer Guide**

### **Nomenclature**

Jetleg uses a couple of names different from the ones used in the literature:
- *IAR* or *compressed_hull* ( or *actuationRegions*): refers to the feasible region(s) in Orsolino. et al. 2019
- *frictionRegion*: refers to the support region in Bretl. et al. 2008;
- *actuation_polytope*: refers to the force polytopes
### **Jetleg Flow**

##### The communication:

- Jetleg publishes to topics such as "/feasible_region/actuation_polygon" and ", "/feasible_region/support_region" , "/feasible_region/force_polygons" in the **"communication.py"** file (defined in init())
  - The publishers are named "*pub_polygon*", "*pub_support_region*", and "*pub_force_polygons*"
  - Polygon types are being published
- CrawlCrontroller subscribes to the topics in the "/common_utilities/fr/src/**FeasibleRegion.cpp"** file.
  - The return variables are called "*actuationRegionSub*", "*supportRegionSub*", and "*forcePolytopesSub*"
- Jetleg subscribes to the topic "*/hyq/debug" which are published in the logger
  - normals, possible foothold options, optimization_started flag, etc... are all received

##### The computation of the polygons:
- talker() in **"communication.py"** calls **"try_iterative_projection_bretl()"** or "iterative_projection_bretl()" directly in **"computational_dynamics.py"**
  - Returns a *compressed_hull* (support/feasible region as requested), the *actuation polygons* (force polytopes), and the *computation_time*
- "iterative_projection_bretl()" calls "setup_iterative_projection()" and then "pypoman.project_polytope()"
  - "setup_iterative_projection()" prepares the **equality constraint** (gravity action), **inequality constraints** (friction and actuation limits), actuation polygons (force polygons)
  - **"pypoman.project_polytope()"** is used to compute the vertices of the polygon. This is Caron's library and includes Bretl's iterative projection algorithm

##### Foothold planning:
- talker() in **"communication.py"** calls **"selectMaximumFeasibleArea()"**
  - Returns foothold with max area (*foothold_params.option_index*), and the feasible regions (*actuationRegions*)
  
### **Important Jetleg Modules**

##### Communication:
- "examples/communication.py"
  - The main module that communicates with the C++ framework and calls the required computations 
- "jet_leg/iterative_projection_parameters.py"
  - Stores iterative projection parameters from "Debug" topic
- "jet_leg/foothold_planning_interface.py"
  - Stores footplanning parameters from "Debug" topic
  
##### Foothold Planning:
- "jet_leg/foothold_planning.py"
  - Selects foothold option with maximum feasible region (**selectMaximumFeasibleArea()**). Uses foothold_planning parameters (including possible foothold options positions) and also IP parameters.
##### IP Algorithm:
- "jet_leg/computational_dynamics.py"
  - Sets up the IP algorithm (**setup_iterative_projection()**) and starts the IP aglorithm (**iterative_projection_bretl()**)

##### Tools:
- "jet_leg/constraints.py"
  - Sets up the inequality constraints (**getInequalities()**)and computes the "actuation polygon" (**computeActuationPolygons()**)
- "jet_leg/computational_geometry.py"
  - Computes halfspace representation for the constraints and also computes polygon area for foothold planning
- "jet_leg/hyq_kinematics.py"
  - Compute forward and inverse kinematics
- "jet_leg/geometry.py"
  - Sorts vertices of a polygon
- "jet_leg/math_tools.py"
  - Performs algebric and geometric computations