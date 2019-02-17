# Roboy Skill Machine

Roboy Skill Machine is a tool designed by the Roboy team, to consolidate Roboy functionality into a series of skills that can
be launched from an easy-to-use user interface, with minimal training required.

## Prerequesites:

1. **ROS:** Roboy runs on ROS, so this must be installed for the Skill Machine to function. You can find installation
instructions for ROS here: http://wiki.ros.org/kinetic/Installation

2. **Python:** The Roboy Skill Machine uses Python 3, which can be found here: https://www.python.org/downloads/

3. **Roboy:** Our code is designed to work with Roboy, so you must have something analogous to Roboy - either a CARDSFlow
simulation (see here: https://cardsflow.readthedocs.io/en/latest/Usage/0_installation.html) or the actual Roboy source
(see here: https://github.com/Roboy/Roboy)

## How to build
1. **Clone the repository:** This can be done by performing the following command:
    ```
    git clone https://github.com/Roboy/roboy_skill_machine.git
    ```
   in your catkin workspace.

2. **Build your workspace:** Use the command:
    ```
    catkin_make
    ````
   in your catkin workspace.

3. **Run Skill Machine:** You can start the skill machine with the following command:
    ```
    rosrun roboy_skill_machine skill_machine.py
    ```
## Adding Skills

### Including Bondpy in the Skill Script
The script should create a bond with Skill Machine. In Python, this is done by including:
```
from bondpy import bondpy
...
bond = bondpy.Bond("skill_machine_bonds", "NODE_NAME_bond")
bond.start()
```

And in C++, this is done by including:
```
#include <bondcpp/bond.h>
...
bond::Bond bond("skill_machine_bonds", "NODE_NAME_bond");
bond.start();
```

### Adding the Roslaunch File
You must add a version of your launch file to the "launch" folder in Skill Machine. See the following format for a basic launch file:
```
<launch>
    <node name="NODE NAME" pkg="PACKAGE NAME" type="EXECUTABLE" />
    <machine name="REMOTE MACHINE NAME" address="REMOTE MACHINE IP ADDRESS" user="USERNAME" password="PASSWORD" env-loader="ROS ENV.SH LOCATION" />
    <node name="NODE NAME" pkg="PACKAGE NAME" type="EXECUTABLE" machine="REMOTE MACHINE NAME" />
</launch>
```
Please refer to the documentation on how to create a roslaunch launch file for more detailed documentation on creating the
launch file here: http://wiki.ros.org/roslaunch/XML
