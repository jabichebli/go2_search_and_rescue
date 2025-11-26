```
~/go2_search_and_rescue/ 
    └── src/
        ├── unitree_ros2/       
        │
        └── go2_sar_pkg/
            ├── package.xml          <-- Dependencies list
            ├── setup.py             <-- Python build setup
            │
            ├── go2_sar_pkg/         <-- Python Source Code
            │   ├── __init__.py
            │   ├── simple_navigator.py  <-- Go to XY
            │   ├── frontier_explorer.py <-- TBD
            │   └── utils.py
            │
            ├── launch/              <-- Launch files
            │   ├── mapping.launch.py    <-- SLAM TBD
            │   └── exploration.launch.py
            │
            ├── config/              <-- Configuration files
            │   ├── nav2_params.yaml     <-- Navigation settings
            │   └── slam_toolbox.yaml    <-- Mapping settings
            │
            └── maps/                <-- Saved .pgm and .yaml maps
```
