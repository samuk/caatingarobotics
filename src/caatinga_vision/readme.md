# **caatinga\_vision**

A ROS 2 computer vision pipeline developed for smart agricultural robotics, specifically designed for traceability and object detection in the field. This package utilises the **YOLOv4-tiny** architecture to provide real-time inference on resource-constrained hardware.

## **Overview**

The `caatinga_vision` package serves as the primary visual perception layer for the Caatinga Robotics platform. It processes raw camera feeds to detect and track objects—ranging from agricultural equipment to people and livestock—facilitating autonomous navigation and crop traceability.

### **Key Features**

* **ROS 2 Integration:** Fully compatible with ROS 2 (Humble/Foxy) using `rclpy`.  
* **YOLOv4-tiny Inference:** High-speed object detection using a lightweight neural network.  
* **Modular Config:** Easy-to-adjust parameters for different environments (e.g., simulation vs. real-world deployment).  
* **Agro-Focused:** Specifically tuned for the requirements of smart agricultural traceability.

## **Package Structure**

Plaintext
