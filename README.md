# Python-Gravity-Simulation

This is an advanced orbital dynamics and n-body simulation program.
Through simple rules of physics complex and beautiful behaviours can occur.

The main file is main.py wich has to be run  to be started.
SolarSystem.py and BarnesHut.py supplies the program with functions.

The Barnes hut algorythm is highly optimised for many bodies and 
makes larger scale n body simulations possible.

Example runs can be found at YouTube with https://youtube.com/@physimdev.

Any help or assistance is greatly appreciated.
Kindly share this project!

# Planned features
1.) docstrings - there is a current branch

2.) Type hints

3.) unit tests

4.) clicking on objects

5.) data handling with json



possible features:

placing objects

ui package 

# Installation guide
1.) download the code from github (either with .zip or git pull/clone)

2.) install requirements.txt
This can be done with the command 

<py -m pip install -r requirements.txt> 
  using cmd in the directory of the code.

A python installation is also needed.

# Running the code
Run the file main.py either from the file explorer, an IDE or CMD/shell.

# Tweakables
There are easily exchangeable variables in the init function of the Interface class in the "UI with solar system.py". 
Possible options include a barnes hut overlay or random planet system generation rather than the solar system.

# Sources



Gravity Physics

https://thepythoncodingbook.com/2021/12/11/simulating-3d-solar-system-python-matplotlib/

https://thepythoncodingbook.com/2021/09/29/simulating-orbiting-planets-in-a-solar-system-using-python-orbiting-planets-series-1/

https://gamedev.stackexchange.com/questions/15708/how-can-i-implement-gravity

https://gereshes.com/2018/07/09/verlet-integration-the-n-body-problem/




Barnes Hut

https://gamedev.stackexchange.com/questions/19393/optimizing-gravity-calculations

https://en.wikipedia.org/wiki/Barnes–Hut_simulation

https://lewiscoleblog.com/barnes-hut

https://beltoforion.de/en/barnes-hut-galaxy-simulator/#idRef2

https://iq.opengenus.org/octree/



Energy


https://physics.stackexchange.com/questions/578071/gravitational-potential-energy-of-an-n-body

https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Variational_Principles_in_Classical_Mechanics_(Cline)/02%3A_Review_of_Newtonian_Mechanics/2.10%3A_Work_and_Kinetic_Energy_for_a_Many-Body_System

https://www.astro.umd.edu/~richard/ASTRO620/QM_chap2.pdf


Collision

https://en.wikipedia.org/wiki/Inelastic_collision

https://www.plasmaphysics.org.uk/collision3d.htm



Other

https://stackoverflow.com/questions/44947505/how-to-make-a-movie-out-of-images-in-python



