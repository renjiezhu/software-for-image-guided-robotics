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
  - In ```Slice Display``` tab, select visible, set mode to intersetion and line width to desired
- Start ROS interface
  - Go to base ROS workspace (```~/Documents/igr```)
  - Run 
    ```bash
    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch software_interface overlay.launch
    ```
  - Send an transform for initialization
- Assign transform to needle
  - Select module ```Transforms```
  - Select transform ```needle_pose``` in the dropdown menu
  - Scroll to bottom and assign the needle model to this transform