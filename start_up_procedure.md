Current Start Up Procedure:

- Start 3D slicer
- Load DICOM and do proper adjustments
  - Calibration : Select module ```Volumes``` and adjust as see fit
  - Show 3d rendering : 
    - Select module ```Volume Rendering``` 
    - Choose desired preset
    - Visualize it by toggling on the eye icon
- Start ```OpenIGTLinkIF``` module (may need downloading, or is in folder ```igt```)
  - Open up a new node (the ```+``` sign)
    - Choose correct IP address
    - Set to ```Client```
    - Set to ```Active```
- Open STL file for needle
  - Select module ```Models``` and select the needle model
  - Change the color (in display tab)
  - In ```Slice Display``` tab, select visible, set mode to intersetion and line width (recommend 2px) to desired
- Start ROS interface
  - Go to base ROS workspace (```~/Documents/igr```)
  - Run 
    ```bash
    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch software_interface overlay.launch # look into the choices in launch folder
    ```
  - Send an transform for initialization
- Assign transform to needle
  - Select module ```Transforms```
  - Select transform ```needle_pose``` in the dropdown menu
  - Scroll to bottom and assign the needle model to this transform


*For path planning to work, run the following command when roscore is up.*

```bash
/opt/VREP/vrep.sh -h -GRosInterface.nodeName=vrep_path_planning -s1000000000000000 -q ./src/software_interface/vrep_robot_control/CtRobot_pathplanning.ttt 
```