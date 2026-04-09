# Simulation_of_Networks_Technologies_for_Disinfection_Robots

## Project Overview
This project presents a MATLAB-based simulation of a remote-controlled disinfection robot operating under different wireless network conditions. The objective is to analyze how network performance accuracy and stability of robot control.

The study is motivated by flu and pandemic scenarios, where remote-controlled robots can be deployed in environments such as universities, hospitals, and public facilities to reduce human exposure.


## Objectives
* Model a mobile robot using a unicycle kinematic model
* Simulate the impact of network latency, packet loss, and jitter on control signals
* Compare the performance of Wi-Fi 7, 4G LTE, and 5G URLLC networks
* Evaluate system performance using trajectory tracking error metrics


 ## Methodology
### Robot Model
The robot is modeled using a unicycle kinematic model with its control inputs being; linear velocity(V) and and angular velocity(ω).

### Network Modeling
Network effects are incorporated into the control loop using:
1. **Latency (Delay)**
  * Commands are delayed by a number of discrete time steps.
2. **Packet Loss (Reliability)**
  * Commands are randomly dropped using a Gilbert- Elliot probabilistic model.
3. **Jitter (Delay Variation)**
  * Jitter is varied using a lognormal distribution.

### Simulation Setup
* Time step: ( dt = 0.001 ) seconds
* Simulation length: 5000 steps


##  Performance Metrics
The following metrics are used:
* **Tracking Error**
* **Average Error**
* **Final Position Error**
* **Maximum Error**





## Key Insight

The results demonstrate that communication network performance plays a critical role in remote robotic control. In particular, 5G URLLC enables near real-time and reliable operation, making it highly suitable for applications such as autonomous disinfection during pandemics.



## 🛠️ Tools Used

* MATLAB
* Numerical simulation



##  References

* Y. Siriwardhana et al., *The Fight Against the COVID-19 Pandemic With 5G Technologies*, IEEE, 2020
* 3GPP URLLC specifications
* IEEE 802.11 (Wi-Fi) standards
* ITU-R guidelines on LTE performance


##  Future Work
* Integration with real robot hardware
* More advanced robot dynamics and control
* Real-time communication modeling
* Multi-robot coordination


