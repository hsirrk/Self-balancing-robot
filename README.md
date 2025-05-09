# Self-Balancing Two-Wheeled Robot

<p align="center">
  <img src="https://github.com/user-attachments/assets/14f84712-e14a-48da-8cdc-3d00c573d219" alt="image" />
</p>



This project focuses on designing and implementing a self-balancing two-wheeled robot capable of maintaining its upright position and recovering from up to a 15° tilt.

Balance Control System
The core functionality of the robot is its ability to self-balance using sensor data and a PID controller:

Angle Measurement: Tilt angle is calculated using data from the onboard gyroscope and accelerometer of the Arduino Nano. A complementary filter fuses the data from both sensors to reduce noise and drift:

θₙ = k(θₙ₋₁ + θg,ₙ) + (1 − k)θa,ₙ

Sensor Considerations:
  Accelerometer readings are noisy due to external acceleration.
  
  Gyroscope data is prone to drift due to integration bias.
  
  Encoders (not used) offer more reliable direct angular position but add system complexity.
  
  Despite the potential improvements with encoders, the current design achieves sufficient balance using only the gyroscope and accelerometer.

Subsystems
  1. Bluetooth Driving Interface
    The robot is remotely controlled via Bluetooth using the Arduino Nano's BLE interface. Python on a PC sends commands using the W/A/S/D keys:
    
    W: Tilt forward by setting the target angle to +1°.
    
    S: Tilt backward by setting the target angle to -1°.
    
    A: Turn left by adjusting PWM signals.
    
    D: Turn right by adjusting PWM signals.
  
    Release any key: Sends a stop command to return to balance.
  
  2. Wireless Camera Streaming
    A wireless video feed is implemented using:
    
    Camera: OmniVision OV7670
    
    Controller: Raspberry Pi Pico W (for Wi-Fi transmission)
    
    Data Transfer: GPIO communication with the camera; video sent to a PC over Wi-Fi.
    
    PC Display: OpenCV used to render the live video.
    
    Wi-Fi was chosen over Bluetooth due to its significantly higher bandwidth (~2.4 GHz vs 400 kHz).
  
  3. Object Avoidance System
    An ultrasonic sensor connected to an ATMega328P detects obstacles. The PulseIn function (which is blocking) required offloading this subsystem from the Arduino Nano to maintain balance.
    
    Avoidance Algorithm:
    
    Reverse for a fixed time.
    
    Turn left 90° using the z-axis angle.
    
    Move forward for a set duration.
    
    Turn right 90° and continue.
    
    The ATMega328P coordinates these movements by sending control signals to the Arduino Nano.
  
  4. Hand Gesture Control
    A hand-gesture interface replaces keyboard commands using a machine learning model built with TensorFlow in Python. A webcam captures gestures and sends BLE commands to the Arduino Nano:
    
    Right open palm: Move forward
    
    Right closed palm: Move backward
    
    Left closed palm: Stop
    
    Left index finger: Turn left
    
    Right index finger: Turn right
  
