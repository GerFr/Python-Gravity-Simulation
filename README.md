# Python-Gravity-Simulation

This is an advanced orbital dynamics and n-body simulation program.
Through simple rules of physics complex and beautiful behaviours can occur.

The main file is `main.py` wich has to be run  to be started.
`SolarSystem.py` and `BarnesHut.py` supplies the program with functions.

The Barnes hut algorythm is highly optimised for many bodies and 
makes larger scale n-body simulations possible.

Example runs can be found at YouTube [here](https://youtube.com/@physimdev).

Any help or assistance is greatly appreciated.
Kindly share this project!

# Planned features

1.) clicking on objects

2.) data handling with json

3.) placing objects

4.) ui package 

please ask for features in [discussions](https://github.com/PhySimdev/Python-Gravity-Simulation/discussions)!

# Installation guide
1.) download the code from github (either with .zip or git clone)

2.) install requirements.txt
This can be done with the command 

`py -m pip install -r requirements.txt` 
 
using cmd in the directory of the code.

A python installation is also needed.

# Running the code
Run the file main.py either from the file explorer, an IDE or CMD/shell.

# Tweakables
There are easily exchangeable variables in the init function of the Interface class in the "UI with solar system.py". 
Possible options include a barnes hut overlay or random planet system generation rather than the solar system.

# Sources

Gravity Physics

[The Python Coding Book - Simulating a 3D Solar System In Python Using Matplotlib](https://thepythoncodingbook.com/2021/12/11/simulating-3d-solar-system-python-matplotlib/)

[Stack Exchange - Velocity verlet pseudo code](https://gamedev.stackexchange.com/questions/15708/how-can-i-implement-gravity)

[gereshes - Verlet Integration](https://gereshes.com/2018/07/09/verlet-integration-the-n-body-problem/)

Barnes Hut

[Lewis Cole Blog - Barnes-Hut Algorithm](https://lewiscoleblog.com/barnes-hut)

[beltoforion - The Barnes-Hut Galaxy Simulator](https://beltoforion.de/en/barnes-hut-galaxy-simulator/)

[OpenGenus - Octree data structure](https://iq.opengenus.org/octree/)

Energy

[Stack Exchange - Gravitational potential energy of an n-body](https://physics.stackexchange.com/questions/578071/gravitational-potential-energy-of-an-n-body)

[LibreTexts - Work and Kinetic Energy for a Many-Body System](https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Variational_Principles_in_Classical_Mechanics_(Cline)/02%3A_Review_of_Newtonian_Mechanics/2.10%3A_Work_and_Kinetic_Energy_for_a_Many-Body_System)

[UMD - The Virial Theorem](https://www.astro.umd.edu/~richard/ASTRO620/QM_chap2.pdf)

Collision

[Plasmaphysics.org.uk - Elastic and Inelastic Collision in Three Dimensions](https://www.plasmaphysics.org.uk/collision3d.htm)



