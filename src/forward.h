#ifndef FORWARD_H
#define FORWARD_H

extern void set_forward( uWS::Hub &h, 
                         PID &pid_steering,
                         PID &pid_throttle,
                         const double &max_velocity,
                         const bool &capture_image );
                         
#endif // FORWARD_H
