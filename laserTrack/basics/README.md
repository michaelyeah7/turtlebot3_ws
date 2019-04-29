# homework-1-f2018-QLV5645-QLJ

/msg diretory includes the file of the definition of the message needed in the package:
1 complex.msg:float32 real
             float32 imaginary
2 LandmarkDistance.msg:float64 distance


/src directory includes the file of implementstion of the ROS python node:
1 publish_complex_numbers.py:this is the example of publisher
  Shell:$ rosrun basics publish_complex_numbers.py
3 triple_complex_numbers.py: this is the example of subsriber
  Shell:$ rosrun basics triple_complex_numbers.py
3 location_monitor_nodeV1.py:this is the first version of the location_monitor_node
  Shell:$ rosrun basics location_monitor_nodeV1.py
4 location_monitor_nodeV2.py:this is the second version of the location_monitor_node
  Shell:$ rosrun basics location_monitor_nodeV2.py
5 location_monitor_nodeV3.py:this is the third version of the location_monitor_nod
  Shell:$ rosrun basics location_monitor_nodeV3.py
