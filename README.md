# EE4314_UAV
This is a simulation code for agile flight. It roughly includes three parts: 
1. Trajectory generation 
2. Flight control
3. Dynamic model

---
## 🚀 Quick Run Guidance: Circular Trajectory Demo

This demo generates a **constant-speed circular trajectory** in the NED frame and runs it through the provided Simulink position control model.  

### 1. Setup
- Open **MATLAB**.  
- Make sure the `funct/` and `position_control/` folders are in your project directory.  
- Paths are added automatically by the script (`addpath` is included).  

### 2. Run the Script
Execute the main script:  

```matlab
CIRCLE_TRAJ_SIMPLE

