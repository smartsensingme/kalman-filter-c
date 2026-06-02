# Magnitude, Velocity, and Acceleration Estimation with Kalman Filter (AS5600)

*Read in other languages: [Português](README.pt-br.md)*

This document details the conceptual operation and practical implementation of **2nd Order (2D)** and **3rd Order (3D) Linear Kalman Filters** to estimate a magnitude ($\theta$), velocity ($\omega$), and acceleration ($\alpha$) from raw sensor readings.

---

## 1. Conceptual Operation

### A. 2D Kalman Filter (Magnitude and Velocity)
The system state is represented by a two-dimensional state vector:
$$\mathbf{x} = \begin{bmatrix} \theta \\ \omega \end{bmatrix} \begin{matrix} \text{ (Magnitude being measured)} \\ \text{ (Rate of change of the magnitude)} \end{matrix}$$

*   **Prediction (2D Kinematic Model):**
    At each sampling period $\Delta t$ (e.g., 1 ms), we predict the next state based on uniform linear motion:
    $$\theta_{k|k-1} = \theta_{k-1} + \omega_{k-1} \cdot \Delta t$$
    $$\omega_{k|k-1} = \omega_{k-1}$$
    
    The state transition matrix $\mathbf{F}$ is:
    $$\mathbf{F} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$$

*   **Measurement Matrix $\mathbf{H}$:**
    $$\mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

---

### B. 3D Kalman Filter (Magnitude, Velocity, and Acceleration)
To model systems with more aggressive dynamics or to obtain a direct estimate of the instantaneous acceleration, we expand the state vector to 3 dimensions:
$$\mathbf{x} = \begin{bmatrix} \theta \\ \omega \\ \alpha \end{bmatrix} \begin{matrix} \text{ (Measured magnitude)} \\ \text{ (Rate of change of the magnitude)} \\ \text{ (Acceleration of change of the magnitude)} \end{matrix}$$

*   **Prediction (3D Kinematic Model):**
    Using the equation of uniformly varied motion:
    $$\theta_{k|k-1} = \theta_{k-1} + \omega_{k-1} \cdot \Delta t + \frac{1}{2} \alpha_{k-1} \cdot \Delta t^2$$
    $$\omega_{k|k-1} = \omega_{k-1} + \alpha_{k-1} \cdot \Delta t$$
    $$\alpha_{k|k-1} = \alpha_{k-1}$$
    
    The state transition matrix $\mathbf{F}$ is:
    $$\mathbf{F} = \begin{bmatrix} 1 & \Delta t & \frac{1}{2}\Delta t^2 \\ 0 & 1 & \Delta t \\ 0 & 0 & 1 \end{bmatrix}$$

*   **Measurement Matrix $\mathbf{H}$:**
    Since only the magnitude $\theta$ is physically measured by the sensor:
    $$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$$

*   **Prediction Covariance Equations ($\mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}$):**
    To avoid matrix arithmetic at runtime on the microcontroller, the equations were expanded symbolically.
    Letting $h = \frac{1}{2}\Delta t^2$:
    *   $a_0 = P_{00} + P_{10}\Delta t + P_{20}h$
    *   $a_1 = P_{01} + P_{11}\Delta t + P_{21}h$
    *   $a_2 = P_{02} + P_{12}\Delta t + P_{22}h$
    *   $a_4 = P_{11} + P_{21}\Delta t$
    *   $a_5 = P_{12} + P_{22}\Delta t$
    
    The resulting predicted covariance matrix is:
    *   $P_{00} = a_0 + a_1\Delta t + a_2 h + q_\theta$
    *   $P_{01} = a_1 + a_2\Delta t$
    *   $P_{02} = a_2$
    *   $P_{10} = P_{01}$
    *   $P_{11} = a_4 + a_5\Delta t + q_\omega$
    *   $P_{12} = a_5$
    *   $P_{20} = P_{02}$
    *   $P_{21} = P_{12}$
    *   $P_{22} = P_{22} + q_\alpha$

---

## 2. Filter Parameters and Noise

### Measurement Noise ($R$)
The parameter $R$ defines the variance of the electrical/magnetic noise in the sensor reading.
*   **Datasheet Calculation:** For example, in the case of the AS5600 sensor, the raw RMS noise (1-Sigma) is $0.06^\circ$. Thus, the corresponding ideal variance is:
    $$R = (0.06^\circ)^2 = 0.0036\text{ (degrees)}^2$$
*   **Practical Tuning:** To deal with mechanical backlash, vibration, and magnet eccentricities in the real world, the firmware was tuned with:
    $$R \approx 0.018\text{ (degrees)}^2$$

### Process Noise ($\mathbf{Q}$)
The matrix $\mathbf{Q}$ represents the physical model uncertainties.
*   **2D Tuning:**
    *   $q_\theta = 0.001$ (Very low uncertainty in the magnitude estimation)
    *   $q_\omega = 50.0$ (Allows tracking fast system accelerations)
*   **3D Tuning:**
    *   $q_\theta = 0.001$ (Very low uncertainty in the magnitude estimation)
    *   $q_\omega = 10.0$ (Prioritizes noise filtering on velocity)
    *   $q_\alpha = 100.0$ (Allows rapid response to acceleration changes)

---

## 3. Structure Implementation (`src/kalman.h`)

The 2D and 3D Kalman filter structures are declared in the project's header:

```c
struct kalman_2d {
  float x[2];    /* State Vector: [0] = magnitude, [1] = velocity */
  float P[2][2]; /* Error covariance matrix */
  float Q_theta; /* Process noise of the magnitude */
  float Q_omega; /* Process noise of the velocity */
  float R;       /* Measurement noise */
};

struct kalman_3d {
  float x[3];    /* State Vector: [0] = magnitude, [1] = velocity, [2] = acceleration */
  float P[3][3]; /* Error covariance matrix */
  float Q_theta; /* Process noise of the magnitude */
  float Q_omega; /* Process noise of the velocity */
  float Q_alpha; /* Process noise of the acceleration */
  float R;       /* Measurement noise */
};
```
---
![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Description

This project is part of the **SmartSensing.me** ecosystem and goes beyond the basic examples found on the internet. Here, we apply the real fundamentals of instrumentation engineering and high-performance embedded systems.

Unlike shallow content aimed only at clicks, this repository delivers:
- **Originality:** Original implementations based on nearly 30 years of academic experience.
- **Technical Density:** Professional use of the ESP-IDF framework and FreeRTOS.
- **Didactics:** Documented and structured code for those seeking true technical evolution.

> "We transform signals from the physical world into digital intelligence, without shortcuts."

---

## 🛠️ Technologies
- **Target Hardware:** ESP32 / ESP32-S3
- **Framework:** ESP-IDF v5.x / v6.x
- **Language:** C / C++
- **Simulation:** LTSpice (Sensor Modeling)

---

## 👤 About the Author

**José Alexandre de França** *Associate Professor in the Department of Electrical Engineering at UEL*

Electrical Engineer with nearly three decades of experience in undergraduate and postgraduate teaching. PhD in Electrical Engineering, researcher in electronic instrumentation, and developer of embedded systems. SmartSensing.me is my commitment to raising the level of technological education in Brazil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 License

This project is under the MIT License. See the [LICENSE](LICENSE) file for details.
