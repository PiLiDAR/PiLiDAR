"""
PID tuning:

1. Set all gains to zero: Start with the PID controller in its simplest form, which is a proportional controller. 
This means setting the integral and derivative gains (Ki and Kd) to zero.

2. Tune the Proportional Gain (Kp): Increase Kp until the output oscillates. 
Once the output oscillates, reduce the Kp slightly until the oscillation stops. 
(This is known as the ultimate gain, Ku)

3. Tune the Integral Gain (Ki): Implement integral action to eliminate the steady state error. 
Start increasing the Ki gain from zero, until any offset is corrected in sufficient time for the process. 
If the system becomes unstable (starts to oscillate or becomes too sluggish), reduce Ki.

4. Tune the Derivative Gain (Kd): Implement derivative action to improve the settling time and overshoot. 
Increase Kd, if required, until the loop is quick to reach its reference after a load disturbance. 
If the system becomes unstable or too sensitive to noise, reduce Kd.

PID tuning basics:
- [An Introduction to Proportional-Integral-Derivative (PID) Controllers](https://engineering.purdue.edu/~zak/ECE_382-Fall_2018/IntroPID_16.pdf)
- [PID Tuning via Classical Methods - Engineering LibreTexts](https://eng.libretexts.org/Bookshelves/Industrial_and_Systems_Engineering/Chemical_Process_Dynamics_and_Controls_%28Woolf%29/09%3A_Proportional-Integral-Derivative_%28PID%29_Control/9.03%3A_PID_Tuning_via_Classical_Methods)
- [PID Tuning | How to Tune a PID Controller - RealPars](https://www.realpars.com/blog/pid-tuning)
- [PLC PID Control Tuning: Practical Tips and Methods](https://controlforge.github.io/posts/PLC-PID-Control-Tuning-Practical-Tips-and-Methods/)

PID advanced: Anti-Windup
- [Anti-Windup Control Using PID Controller Block - MathWorks](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)
- [Design and Modeling of Anti Wind Up PID Controllers](https://link.springer.com/chapter/10.1007/978-3-319-12883-2_1)
- [PID Anti-windup Techniques - Erdos Miller](https://info.erdosmiller.com/blog/pid-anti-windup-techniques)
- [Comparative Study of Anti-windup Techniques on Performance ... - Springer](https://link.springer.com/chapter/10.1007/978-981-15-4676-1_10)

"""

from simple_pid import PID
import threading

from lib.lidar_driver import LD06



setpoint = 10  #  RPM target_speed

# PID values: find P by setting set I and D to zero
Ku = 0.05  # initial value for Ku
Ku_delta = 0.01  # step size for Ku
Ki = 0.01
Kd = 0.01
pid_controller = PID(Ku, Ki, Kd, setpoint=setpoint)

dc_limits = [20, 100]  # Duty cycle boundaries in percentage

# Keep a history of the last N speeds
speed_history = []


def Ziegler_Nichols_callback():
    global Ku, Ki, Kd, pid_controller, dc_limits, setpoint

    # Increase Ku slightly
    Ku += Ku_delta

    # Update the PID controller's tunings
    pid_controller.tunings = (Ku, Ki, Kd)

    # Calculate the duty cycle using the PID controller
    dc = pid_controller(lidar.speed)

    # Anti-Windup: Limit the duty cycle within the specified boundaries
    clipped_dc = max(min(dc, dc_limits[1]), dc_limits[0])

    print("current speed:", round(lidar.speed, 2), "dc calculated:", round(dc, 2), "clipped dc:", round(clipped_dc, 2))
    lidar.pwm.change_duty_cycle(clipped_dc) 

    # Check if the system is oscillating
    if is_oscillating():
        print("System is oscillating at Ku =", Ku)
        return


def is_oscillating():
    global speed_history

    # Add the current speed to the history
    speed_history.append(lidar.speed)

    # Only keep the last N speeds
    N = 100
    speed_history = speed_history[-N:]

    # If we don't have enough data yet, we can't tell if the system is oscillating
    if len(speed_history) < N:
        return False

    # Check if the speeds are alternating between increasing and decreasing
    is_increasing = speed_history[1] > speed_history[0]
    for i in range(2, N):
        if (speed_history[i] > speed_history[i-1]) != is_increasing:
            is_increasing = not is_increasing
        else:
            # The speeds are not alternating, so the system is not oscillating
            return False

    # The speeds are alternating, so the system is likely oscillating
    return True


if __name__ == "__main__":
    # LD06: 4500 points per second / (12 points per package * 10 revolutions per second) = 37.5 package per revolution
    # out_len = round(sampling_rate / (12 * speed)) = 38

    # Initialize LiDAR
    lidar = LD06(port = '/dev/ttyS0',
                 speed = 10,
                 format = None,
                 visualization = None,
                 out_len = 38,  # packages per revolution
                 platform = "RaspberryPi")

    try:
        if lidar.serial_connection.is_open:
            read_thread = threading.Thread(target=lidar.read_loop, kwargs={'callback': Ziegler_Nichols_callback})
            read_thread.start()
            read_thread.join()
    finally:
        lidar.close()
