# It is an initial try to create a centralized function which controls the OS of the Jetson Robot. #

I will try out possibilities from python's [os](https://docs.python.org/3.6/library/os.html) and [subprocess](https://docs.python.org/3/library/subprocess.html) libraries. 

### TODO ###

- Roscore Singleton.
- Building a state-machine system for roslaunches and sharing information between outputs.
- connecting to jetson-nano and checking it's performance from time to time. 