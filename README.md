# ADAS-Collision-Avoidance-System-on-Indian-Roads
Real-world implementation of ADAS Level 0 - Collision Avoidance System (CAS) on Indian Roads using LIDAR-Camera Low-Level Sensor Fusion


# How to Use?

Please execute the commands in this order, so that by the time the object detection module runs the LIDAR and Flash modules will be up and running. Object detection module is the publish node and other two are subscriber nodes.

To run the LIDAR distance estimation Subscribe module:
```python
python3 lidar_getdist.py 
```

To run the Flash Warning Subscriber Module:
```python
python3 pulse_mqtt.py 
```

To run the main object detection Publish Module:
```python
python3 object_detection_demo.py -i 0 -m ssdlite_mobilenet_v2/ssdlite_mobilenet_v2.xml -at ssd --labels ~/open_model_zoo/data/dataset_classes/coco_91cl.txt -d MYRIAD --output_resolution 640x480
```


