```
ros2 launch sp_demo_nav2_bringup cloned_multi_tb3_simulation_launch.py robots:="robot0={x: 0.0, y: 5.0, yaw: 0.0}; robot1={x: 3.0, y: 5.0, yaw: 0.0};"	
```


```
gz service -s /world/warehouse/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf: "<sdf version=\"1.6\"><model name=\"inline_cube\"><pose>1.6 5.6 0 0 0 0</pose><static>true</static><link name=\"link\"><visual name=\"v\"><geometry><box><size>0.2 0.2 1</size></box></geometry></visual><collision name=\"c\"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></sdf>"'
```
