'''
fitting a linear function to PWM vs. Speed measurements of LDRobot Lidar
works with LD06,
TODO: PWM cannot control STL27L for some reason.
'''

import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


# Define the form of the function you want to fit (e.g., a linear function).
def speed_from_pwm(pwm, m, b):
    return m * pwm + b

# Define the inverse function
def pwm_from_speed(speed, m, b):
    return (speed - b) / m

# use curve fitting to estimate the parameters of a linear function
def estimate_pwm_params(pwm_measurements):

    # Define the form of the function you want to fit (e.g., a linear function).
    def func(pwm, m, b):  # = speed_from_pwm()
        return m * pwm + b

    # Plot the data points and the fitted curve
    def plot_fit(pwm, speed, func, params):
        plt.scatter(pwm, speed, label='Data')
        plt.plot(pwm, func(pwm, *params), color='red', label='Fitted function')
        plt.legend(loc='best')
        plt.show()

    pwm, speed = zip(*pwm_measurements)

    # Convert pwm and speed to numpy arrays
    pwm = np.array(pwm)
    speed = np.array(speed)

    # Use curve_fit to find the optimal parameters.
    params, params_covariance = curve_fit(func, pwm, speed)

    # Plot the data and the fitted curve.
    plot_fit(pwm, speed, func, params)
    return params


if __name__ == "__main__":

    sample_values   = False
    calculate       = False
    infer           = True


    ####################################################################################################
    if sample_values:
        import time
        import threading
        from lidar_driver import LD06

        def my_callback():
            # print(round(lidar.pwm_dc, 3), round(lidar.speed, 3))

            if lidar.pwm_dc >= pwm_dc_range[1]:
                measurements.append((round(lidar.pwm_dc, 4), round(lidar.speed, 4)))
                print(measurements[-1])
                lidar.update_pwm_dc(lidar.pwm_dc - decrement)
            else:
                exit()

        # LD06 needs to start at reasonable speed and can then decelerate down to 0.1, but not start there!
        pwm_dc_range = (0.5, 0.12)
        decrement = 0.01
        buffer_len = 5000

        measurements = []

        lidar = LD06(port = '/dev/ttyS0',
                     pwm_dc = 0.4,
                     format = None,
                     visualization = None,
                     out_len = buffer_len,
                     platform = "RaspberryPi")

        lidar.update_pwm_dc(pwm_dc_range[0])
        time.sleep(1)

        try:
            if lidar.serial_connection.is_open:
                read_thread = threading.Thread(target=lidar.read_loop, kwargs={'callback': my_callback})
                read_thread.start()
                read_thread.join()
        finally:
            lidar.close()
            print(measurements)


    ####################################################################################################
    if calculate:
        if not sample_values:
            measurements = [(0.5, 15.8278), (0.49, 15.5194), (0.48, 15.1833), (0.47, 14.8361), (0.46, 14.3389), (0.45, 14.125),
                            (0.44, 13.7972), (0.43, 13.4083), (0.42, 13.1444), (0.41, 12.8194), (0.4, 12.4472), (0.39, 12.15),
                            (0.38, 11.8278), (0.37, 11.5056), (0.36, 11.2056), (0.35, 10.8444), (0.34, 10.4972), (0.33, 10.1528),
                            (0.32, 9.8083), (0.31, 9.4444), (0.3, 9.0889), (0.29, 8.8056), (0.28, 8.4333), (0.27, 8.1), (0.26, 7.7222),
                            (0.25, 7.4111), (0.24, 7.4472), (0.23, 7.0333), (0.22, 6.4917), (0.21, 6.1472), (0.2, 5.8), (0.19, 5.4444),
                            (0.18, 5.0972), (0.17, 4.7833), (0.16, 4.4306), (0.15, 4.1056), (0.14, 3.8194), (0.13, 3.4778)]

        params = tuple(estimate_pwm_params(measurements))
        print(f"m: {params[0]}, b:{params[1]}")


    ####################################################################################################
    if infer:
        if not calculate:
            params = (33.3174, -0.8497)

        speed = 10
        pwm = pwm_from_speed(speed, *params)
        print(f"speed {speed} -> PWM {round(pwm, 3)}")

        pwm = 0.326
        speed = speed_from_pwm(pwm, *params)
        print(f"PWM {pwm} -> speed {round(speed, 3)}")
