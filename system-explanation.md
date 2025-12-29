The device is designed based on the system schematic, with the ESP32 microcontroller serving as the main control unit. All electronic components are connected to the ESP32 through dedicated pins according to their respective functions. The system integrates multiple sensing, actuation, and communication modules to support mobility assistance and safety features.

![System Schematics](images/system-schematics.png)

Four ultrasonic sensors are used for obstacle detection from multiple directions: bottom-front, front, left, and right. The sensors are strategically placed along the walking stick, with the front and side sensors positioned approximately 50 cm from the tip of the stick to align with the average knee height of Indonesian adults. This placement allows effective detection of obstacles at a height that may pose a risk during walking. The bottom-facing sensor is installed near the lower end of the stick to detect ground-level obstacles such as steps or holes.

An omniwheel driven by a DC motor and motor driver is mounted at the lower section of the stick. The motor driver is controlled by the ESP32 to regulate the motion of the omniwheel, which provides directional feedback by gently guiding the user away from detected obstacles. When no obstacle is detected, the omniwheel moves freely without applying force.

The system also includes a panic button as an emergency safety feature. When activated, the ESP32 triggers an audible alarm through a buzzer and sends the user’s location via SMS using the GSM module. The location data is obtained from the GPS module and transmitted as a Google Maps link to a predefined contact number.

Due to the limited number of UART pins on the ESP32, the GPS and GSM modules share communication pins through a pin-switching strategy. During normal operation, the UART pins are assigned to the GPS module for continuous location tracking. When the panic button is pressed, the ESP32 temporarily switches the UART connection to the GSM module to send the emergency SMS. After the message is successfully sent, the UART pins are automatically reassigned back to the GPS module.

Power is supplied by two 18650 Li-ion batteries. A dedicated power management circuit is designed to meet the voltage requirements of each component. Voltage regulation and level conversion are used to provide appropriate operating voltages for the ESP32, sensors, communication modules, and DC motor, ensuring stable and reliable system operation.

![System Workflow](images/system-workflow.png)
