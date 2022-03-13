# It is an initial try to create a centralized function which controls the OS of the Jetson Robot. #

I will try out possibilities from python's [os](https://docs.python.org/3.6/library/os.html) and [subprocess](https://docs.python.org/3/library/subprocess.html) libraries. 

### TODO ###

- 0. Draw your necessities, define the steps and final goals.
- 1. roscore singleton - to be able to control roscore from python
- 2. automatic ssh in new terminal window through pexpect and monitoring the output
- 3. new terminal windows for basic necessities OR DIRECT ROSLAUNCH API
- 4. dynamic roslaunches and python scripts for communication and optimization...
- 5. look at jetson's performance variables.