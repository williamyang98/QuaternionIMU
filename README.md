## Hardware 
Uses the MPU6050 combined gyroscope and accelerometer and GY271 compass as sensors. 
This communicates over Bluetooth to a python client which passes it to an extended Kalman filter for orientation estimation.

## State space model
A state space model can be described with the following equations.

![equaton](https://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%7D%20%5Cdot%7Bx%7D%20%26%3D%20Ax%20&plus;%20Bu%20%3D%20f%28x%2Cu%29%20%5Cnotag%20%5C%5C%20y%20%26%3D%20Cx%20&plus;%20Du%20%3D%20h%28x%2Cu%29%20%5Cnotag%20%5Cend%7Balign%7D)

Where x is our state, u is our input, A the state transition matrix, B the input matrix. Our measurements depend on the current state, and are given by y, where C is our output matrix, D is our feedforward matrix.

Our coordinate system of choice is given in the following diagram. +x faces forward, +y faces right, and +z faces down, relative from the sensor suite. 

![image](docs/coordinate_system.PNG)

For our orientation estimation problem, we have 2 measurements which depend on the current orientation; the gravity vector (Axyz) and magnetic heading vector (Mxyz). 

Additionally we know how the orientation changes based on the gyroscope, which gives us body angular velocity values (pqr).

We can use the gyroscope to formulate our state transition function f(x,u). In addition our state "x" is given as a quaternion, which is more appropriate for orientation estimation. Euler angles in comparison suffer from gimbal lock at singularity conditions.

![equation](https://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%7D%20x%20%26%3D%20%5Be_0%2C%20e_1%2C%20e_2%2C%20e_3%5D%5ET%20%5Cnotag%20%5C%5C%20%5Cdot%7Bx%7D%20%26%3D%20Ax%20&plus;%20Bu%20%3D%20f%28x%2Cu%29%20%5Cnotag%20%5C%5C%20%5Cbegin%7Bbmatrix%7D%20%5Cdot%7Be_0%7D%20%5C%5C%20%5Cdot%7Be_1%7D%20%5C%5C%20%5Cdot%7Be_2%7D%20%5C%5C%20%5Cdot%7Be_3%7D%20%5Cend%7Bbmatrix%7D%20%26%3D%20%5Cdfrac%7B1%7D%7B2%7D%20%5Cbegin%7Bbmatrix%7D%200%20%26%20-p%20%26%20-q%20%26%20-r%20%5C%5C%20p%20%26%200%20%26%20r%20%26%20-q%20%5C%5C%20q%20%26%20-r%20%26%200%20%26%20p%20%5C%5C%20r%20%26%20q%20%26%20-p%20%26%200%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20e_0%20%5C%5C%20e_1%20%5C%5C%20e_2%20%5C%5C%20e_3%20%5Cend%7Bbmatrix%7D%20%5Cnotag%20%5Cend%7Balign%7D)

If we take our original accelerometer or magnetometer measurement at our original orientation, the measurement taken by the sensors after our orientation changes is given in the following equations. "v" represents our original external vector, and "b" represents the measured body vector after our orientation changes from q=[1 0 0 0] to q=[e0 e1 e2 e3].

![equation](https://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%7D%20%5Coverrightarrow%7Bv%7D%20%26%3D%20%5Bv_x%2C%20v_y%2C%20v_z%5D%5ET%20%5Cnotag%20%5C%5C%20%5Coverrightarrow%7Bb%7D%20%26%3D%20%5Bb_x%2C%20b_y%2C%20b_z%5D%5ET%20%5Cnotag%20%5C%5C%20%5Cbegin%7Bbmatrix%7D%20b_x%20%5C%5C%20b_y%20%5C%5C%20b_z%20%5Cend%7Bbmatrix%7D%20%26%3D%20%5Cbegin%7Bbmatrix%7D%20%28e_0%5E2&plus;e_1%5E2-e_2%5E2-e_3%5E2%29%20%26%202%28e_1%20e_2%20&plus;%20e_0%20e_3%29%20%26%202%28e_1%20e_3%20-%20e_0%20e_2%29%20%5C%5C%202%28e_1%20e_2%20-%20e_0%20e_3%29%20%26%20%28e_0%5E2-e_1%5E2&plus;e_2%5E2-e_3%5E2%29%20%26%202%28e_2%20e_3%20&plus;%20e_0%20e_1%29%20%5C%5C%202%28e_1%20e_3%20&plus;%20e_0%20e_2%29%20%26%202%28e_2%20e_3%20-%20e_0%20e_1%29%20%26%20%28e_0%5E2-e_1%5E2-e_2%5E2&plus;e_3%5E2%29%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20v_x%20%5C%5C%20v_y%20%5C%5C%20v_z%20%5Cend%7Bbmatrix%7D%20%5Cnotag%20%5Cend%7Balign%7D)

Our measurements are the body relative acceleration and magnetic heading vectors.
![equation](https://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%7D%20y%20%26%3D%20%5Cbegin%7Bbmatrix%7D%20b_M%20%5C%5C%20b_A%20%5Cend%7Bbmatrix%7D%20%5Cnotag%20%5C%5C%20y%20%26%3D%20h%28x%2Cu%29%20%5Cnotag%20%5Cend%7Balign%7D)

## Extended Kalman Filter
The Kalman filter takes in a state transition function, which updates our state from X(k) to X(k+1). Using Euler integration, this is given as the following equation.

![equation](https://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%7D%20%5Cdot%7Bx%7D%20%26%3D%20f%28x%2Cu%29%20%5Cnotag%20%5C%5C%20x_%7Bk&plus;1%7D%20%26%3D%20x_%7Bk%7D%20&plus;%20%5Cdot%7Bx_%7Bk&plus;1%7D%7D%20%5Ctimes%20%5Cdelta%7Bt%7D%20%5Cnotag%20%5C%5C%20x_%7Bk&plus;1%7D%20%26%3D%20F%28x_k%29%20%5Cnotag%20%5Cend%7Balign%7D)


A regular Kalman filter works with a linear state space model. However as seen in our measurement equations, we cannot represent it as a linear system where y=Cx.

This can be resolved by using an extended Kalman filter as shown below, where we take the Jacobian of our state transition function F(x,u) and measurement function h(x,u) as our Ak and Hk matrices.

![image](docs/ekf_equations.png)

![equation](https://latex.codecogs.com/svg.latex?%5Cbegin%7Balign%7D%20A_k%20%26%3D%20%5Cnabla%7BF%7D_x%20%28x_k%2C%20u_k%29%20%5Cnotag%20%5C%5C%20H_k%20%26%3D%20%5Cnabla%7Bh%7D_x%20%28x_k%2C%20u_k%29%20%5Cnotag%20%5C%5C%20z_K%20%26%3D%20y_k%20%5Cnotag%20%5Cend%7Balign%7D)

Here Qk is our process covariance (how noisy our state transition process is), and Rk is our measurement covariance (how noisy our measurements are). 
Pk is our state covariance (how uncertain our state is at time t=k). 

We could take Qk and Rk as constants, and P(0) as our initial state covariance (uncertainty of initial state estimate). However as discussed [here](https://ahrs.readthedocs.io/en/latest/filters/ekf.html), we can estimate our process noise at t=k using the following formula.


![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Balign%7D%20W_k%20%26%3D%20%5Cdfrac%7B%5Cdelta%7Bt%7D%7D%7B2%7D%20%5Cbegin%7Bbmatrix%7D%20-e_1%20%26%20-e_2%20%26%20-e_3%20%5C%5C%20e_0%20%26%20-e_3%20%26%20-e_2%20%5C%5C%20e_3%20%26%20e_0%20%26%20-e_1%20%5C%5C%20-e_2%20%26%20e_1%20%26%20e_0%20%5Cend%7Bbmatrix%7D%20%5Cnotag%20%5C%5C%20Q_k%20%26%3D%20Q%20W_k%20W_k%5ET%20%5Cnotag%20%5Cend%7Balign%7D)

## Gallery
![image](docs/screenshot.PNG)