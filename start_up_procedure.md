Current Start Procedure:

- Start 3D slicer
- Load DICOM and do proper adjustments
  - calibration
  - show 3d rendering
- Start OpenIGTLinkIF module
  - Open up a node
    - Choose correct IP
    - Set to "Client"
    - Set to "active"
- Open STL file for needle
- Start ROS interface
  - Send an transform for initialization
- Assign transform to needle