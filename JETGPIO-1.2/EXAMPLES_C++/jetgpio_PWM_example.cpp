/* Usage example of the JETGPIO library
 * Compile with: g++ -Wall -o jetgpio_PWM_example jetgpio_PWM_example.cpp -ljetgpio
 * Execute with: sudo ./jetgpio_PWM_example
 */

#include <iostream>
#include <unistd.h>
#include <jetgpio.h>

int main(int argc, char *argv[])
{
  int Init, pin = 33;

  Init = gpioInitialise();
  if (Init < 0)
    {
      /* jetgpio initialisation failed */
      printf("Jetgpio initialisation failed. Error code:  %d\n", Init);
      exit(Init);
    }
  else
    {
      /* jetgpio initialised okay*/
      printf("Jetgpio initialisation OK. Return code:  %d\n", Init);
    }	
	
  /* Setting up PWM frequency=10kHz @ pin*/

  int PWMstat = gpioSetPWMfrequency(pin, 100);

  if (PWMstat < 0)
    {
      /* PWM frequency set up failed */
      printf("PWM frequency set up failed. Error code:  %d\n", PWMstat);
      exit(Init);
    }
  else
    {
      /* PWM frequency set up okay*/
      printf("PWM frequency set up okay at pin 32. Return code:  %d\n", PWMstat);
    }
  
  /* Set up PWM duty cycle to approx 50% (0=0% to 256=100%) @ pin 32*/
  int PWMstat2 = gpioPWM(pin, 45);

  if (PWMstat2 < 0)
    {
      /* PWM start on failed */
      printf("PWM start failed. Error code:  %d\n", PWMstat2);
      exit(Init);
    }
  else
    {
      /* PWM started on okay*/
      printf("PWM started up okay at pin 32. Return code:  %d\n", PWMstat2);
    }

  // Move the right motor forward
  int x = 0;
  printf("PWM going forward at pin for 20 seconds\n");
  while (x<10) {
    sleep(2);
    x++;
  }

  gpioPWM(pin, 30);
  x = 0;
  // Move left motor backwards
  printf("PWM going backwards at pin for 20 seconds\n");
  while (x<10) {
    sleep(2);
    x++;
  }
  // Terminating library
  gpioPWM(pin, 0);
  sleep(2);

  gpioTerminate();
  printf("PWM stopped, bye!\n");
  exit(0);
	
}

