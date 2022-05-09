# La_llama
The code makes a PID control to control a follower robot, using Arduino as embedded system.
# Instalación:
* Download Arduino IDE: https://www.arduino.cc/en/software
* Replace motor pins and start button
* Instalar `<Servo.h>`
``` ruby
#define moti                    6                       //Pin del motor izquierdo.
#define motd                    9                       // Pin del motor derecho.
#define moti_pwm                5//es el 7              //PWM moto izquierdo.
#define motd_pwm                10                      // PWM moto derecho.
#define boton_1                 22                      // pin del boton.
#define led                     13
```
# Instructions
* When the robot is turned on, the robot starts to sense the line to calibrate the sensors.
* Then you press the button and after 5s it accelerates.
