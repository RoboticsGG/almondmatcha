# LSM6DSV16X IMU 

The LSM6DSV16X is a high-performance, low-power 6-axis small IMU, featuring a 3-axis digital accelerometer and a 3-axis digital gyroscope, that offers the best IMU sensor with a triple-channel architecture for processing acceleration and angular rate data on three separate channels (user interface, OIS, and EIS) with dedicated configuration, processing, and filtering.

The LSM6DSV16X enables processes in edge computing, leveraging embedded advanced dedicated features such as a finite state machine (FSM) for configurable motion tracking and a machine learning core (MLC) for context awareness with exportable AI features for IoT applications.

The LSM6DSV16X supports the adaptive self-configuration (ASC) feature, which allows the FSM to automatically reconfigure the device in real time based on the detection of a specific motion pattern or based on the output of a specific decision tree configured in the MLC, without any intervention from the host processor.

The LSM6DSV16X embeds Qvar (electric charge variation detection) for user interface functions like tap, double tap, triple tap, long press, or L/R – R/L swipe.

The LSM6DSV16X embeds an analog hub able to connect an external analog input and convert it to a digital signal for processing.

## Examples

- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_000-DataLogTerminal](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_000-DataLogTerminal):** This application shows how to get data from LSM6DSV16X accelerometer and gyroscope and print them on terminal.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_001-SensorFusion](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_001-SensorFusion):** This application shows how to use LSM6DSV16X Sensor Fusion features for reading quaternions.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_002-6D_Orientation](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_002-6D_Orientation):** This application shows how to use LSM6DSV16X accelerometer to find out the 6D orientation and display data on a hyperterminal.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_003-LSM6DSV16X_Qvar_Polling](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_003-Qvar_Polling):** This application shows how to use LSM6DSV16X Qvar features in polling mode.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_004-SingleTap](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_004-SingleTap):** This application shows how to detect the single tap event using the LSM6DSV16X accelerometer.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_005-DoubleTap](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_005-DoubleTap):** This application shows how to detect the double tap event using the LSM6DSV16X accelerometer.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_006-TiltDetection](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_006-TiltDetection):** This application shows how to detect the tilt event using the LSM6DSV16X accelerometer.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_007-Pedometer](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_007-Pedometer):** This application shows how to use LSM6DSV16X accelerometer to count steps.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_008-FreeFallDetection](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_008-FreeFallDetection):** This application shows how to detect the free fall event using the LSM6DSV16X accelerometer.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_009-WakeUpDetection](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_009-WakeUpDetection):** This application shows how to detect the wake-up event using the LSM6DSV16X accelerometer.
- **[xNucleo-IKS4A1_LSM6DSV16X_mbedOS_010-MachineLearningCore](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_010-MachineLearningCore):** This application shows how to detect the activity using the LSM6DSV16X Machine Learning Core.
- **[xNucleo-IKS4A1_LSM6DS1V6X_mbedOS_011-FIFO_Interrupt](https://github.com/Perlatecnica/xNucleo-IKS4A1_LSM6DSV16X_mbedOS_011-FIFO_Interrupt):** This application shows how to get accelerometer and gyroscope data from FIFO using interrupt and print them on terminal.

## Official Documentation
[LSM6DSV16X Documentation](https://www.st.com/en/mems-and-sensors/lsm6dsv16x.html)
