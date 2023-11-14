# NUCLEOL152RE_Study

Welcome to the NUCLEO-L152RE Study GitHub repository! This repository is dedicated to the study and exploration of the STM32 NUCLEO-L152RE development board. It includes various projects and code examples covering different aspects of microcontroller programming using the STM32CubeIDE and HAL (Hardware Abstraction Layer) libraries. The content is designed for university course assignments and extended hobby projects.

## Projects

1. **GPIO: LED_BLINK**
   - LED blinking with a customized delay function measured with an oscilloscope.

2. **USART: Array Transmission**
   - Sending an array of numbers through USART2 (PA2/PA3), enabled/disabled by a button click.

3. **ADC: Voltage Measurement**
   - Using ADC to measure reference voltage from AVDD pin/trimmer-controlled voltage and sending the median measurement via USART2.

4. **TIMER: Distance Measurement**
   - Using HC-SR04 to measure distance, sending the measurement via USART2 in STM32. The delay function is implemented by timer interrupt.

5. **I2C: Digital Thermometer**
   - Using the I2C bus with Nucleo, connecting DS16121 digital thermometer to pins PB8 and PB9. Temperature readings are then printed to an LCD.

6. **WiFi Module: IoT Integration**
   - Measuring temperature and humidity with a DHT22 sensor. Using USART2 (PA2/PA3) to write the measurement results to USART1 (PA9/PA10). Connecting ESP8266 (ESP-05) and configuring it using AT commands. Finally, sending data to the IoT ThingSpeak service.

## How to Use

Include instructions on how to set up the development environment, compile the code, and upload it to the NUCLEO-L152RE board using TrueStudio.

## License

MIT

## Acknowledgments

Jani Ahvonen (VAMK Professor) instructed 



