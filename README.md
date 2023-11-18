# Cruise_Control
simple cruise control system which support automatic and manual control using stm32f103


using analog joystck used as pedal to press on it generating PWM signal to conrtol the motor 
using ADC in multi conversion mode DMA to send data to array to be processed 
using I2c LCD 16 * 2 to get output of system 


Manual system :: 
starting the system by ebtering mode manual and the pressing the pedal and genrating the PWM signal 
as distance mesured by ICU(Ultrasonic) is less tha the critical distance so the system will stop the motor (stopping the car)

Automatic system ::
starting the system by choosing auto mode in system
As the car is moving based on the nesured distance so the motor PWM signal is controlled 
