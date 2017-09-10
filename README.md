# simpleKalmanFilter
A C++ based Kalman filter for 1D data. Sometimes you need a simple noise filter without any dependencies; for those cases this library should be enough..
## Lightweight c plus plus library for Noise filtering using Kalman filters. 
Filtering noisy measurements can be an extremely difficult endeavor; take brain signals for example. While this is true, there are also many situations where the system is fairly simple and the focus lies on speed, real time computation and ease of use. Filtering RSSI of received signals due to multiple paths noise can be such a case. For these applications it is useful to have a simple, but effective, noise filtering algorithm in your toolkit. Let me introduce this small yet powerfull library implementing the idea of Kalman filters, without any dependencies, to filter out noise in 1D systems.
this figure shows the effect of applying the filter to RSSI values received from BLE Beacons using android mobile. filtering the data is a prestep to calculating distance between the mobile and beacons :
![Alt text](/sample.PNG?raw=true "example of raw data(blue) and kalman filtered data(green)")
