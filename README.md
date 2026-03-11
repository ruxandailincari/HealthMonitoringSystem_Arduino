# Health Monitoring System 
A real-time biomedical parameter monitoring system that integrates hardware data acquisition (Arduino), server-side processing (Node.js), and interactive visualization (Web).
This project demonstrates the ability to build a complete data pipeline: from physical sensors, through serial communication and WebSocket, to a real-time graphical interface.

## Key Features
-	Real-Time Data Acquisition: Continuous reading of signals from the  MAX30102 sensor (SpO2, Heart Rate, Temperature) and ECG AD8232 sensor 
-	Local Processing: Calculation of oxygen saturation and heart rate algorithms directly on the microcontroller
-	Robust Communication: JSON data transfer via Serial to an intermediate Node.js server
-	Live Streaming: WebSocket implementation for instant updates without manual refresh

##  System Architecture
The system consists of three distinct components communicating through a standardized data flow:
1.	Hardware Layer (Arduino): 
-	Sensors: MAX30102 (heart rate, SPO2, temperature), Analog ECG sensor AD8232
-	Output: JSON data stream via Serial Port
-	Alert: Vibration motor activates when heart rate is invalid, providing immediate feedback
2.	Backend Layer (Node.js): 
-	WebSocket server listening on the serial port
-	Parsing and validation of JSON data
-	Forwarding data to connected clients
3.	Frontend Layer (Web): 
-	HTML interface
-	Real-time graphical visualization using Chart.js
-	Display of parameter values

##  Technologies Used
-	Embedded: C++ (Arduino IDE)
-	Backend: Node.js
-	Frontend: HTML
-	Protocol: JSON over Serial & WebSocket

## Contributors
-	Ruxanda Ilincari 
-	Carla Daniela Bozintan
