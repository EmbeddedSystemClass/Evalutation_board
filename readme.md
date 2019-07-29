The code runs on a ATSAMV71.

## Tasks
### Button.c
- Checking if a button is pressed
### Main.c
- Starting the project. 
- Initializing other components, including the console which is over UART, a shell that the console can be used for CLI, for I2C ( TWI), Ethernet, and the DAC 
- Calling the task scheduler since the code is running on a RTOS system.
### Lcd.c
- Calculating values to display on the LCD display, such as temperature and voltage.
- Sending information over I2C to the LCD display
- Initializing of LCD display
### CLI-commands.c
- Initializing CLI commands to control the system.