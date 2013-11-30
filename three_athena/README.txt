How do you compile and run your code?
To run my code, cd into the top of the project dircetory and run ./a3 [e|t|r] [stepsize] where "[e|t|r]" are the euler, trapazoidal, and rk4 integrator respectively and stepsize is the desired step size for the integrators. An integrator and step size are required parameters to run the code. A step size of 0.008 works best for the Trapazoidal integrator and the cloth system, and 0.04 works best in the SimpleSystem and pendulum system. A step size of 0.04 works best for Runge Kutta in all systems. A step size of 0.04 works well for the Euler in the simple system but even with very small values Euler will have difficulty with the cloth system and eventually become unstable.

If permissions need to be changed run "chmod +x a3" on the binary. It may also be necessary to recompile the code, which can be done by typing the command: "make clean; make"

The following are instructions to interact with the user interface (also output when running the binary):
*****************************************************************************************************
* To move the cloth and pendulum systems, press and hold the arrow keys                             *
* To toggle particles, press p                                                                      *
* To toggle springs, press s                                                                        *
* To create or restart the SimpleSystem, press 1                                                    *
* To create or restart the PendulumSystem, press 2                                                  *
* To create or restart the ClothSystem, press 3                                                     *
* To toggle wind, press w                                                                           *
* To use ForwardEuler, press e                                                                      *
* To use Trapzoidal, press t                                                                        *
* To use Runge Kutta, press r                                                                       *
* To change the step size, press i and then enter the float value into the terminal and press enter *
* To exit press Escape                                                                              *
*****************************************************************************************************


Did you collaborate with anyone in the class?
I collaborated with Patricia Saylor, Evan Wang, and Rachel Luo. We provided help with debugging and talking about the concepts, but did not share code.


Helpful references:
none


Are there any problems with your code:
Not that I know of!


Did you do any extra credit:
I implemented wind. It can be triggered in the cloth system by pressing w


Comments:
This was a cool assignment! With the long weekend I wish there were extra office hours.
