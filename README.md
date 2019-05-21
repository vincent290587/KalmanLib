

# Project

This is a C++ Kalman filtering library with both a linear and an extended implementation of the filter.  
Interestingly it uses a custom C++ matrix implementation that allows for classical matrix operations:

```C
// z - C*x_est
UDMatrix matI, innov;
matI = descr->ker.matC * descr->ker.matXmi;
innov = feed->matZ - matI;
```



## Roadmap

- [x] Basic unit testing
- [ ] Bike simulation