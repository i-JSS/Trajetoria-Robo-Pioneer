import csv
import math
import math as mat
import numpy as np
from src.lib import sim

class Pioneer:

    def __init__(self):
        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max_wheels = 15
        self.v_min_wheels = -15
        self.v_linear = 15
        self.Min_error_distance = 0.5


    def connect_pioneer(self, port):
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

        if clientID == 0:
            print("Connect to", port)
        else:
            print("Can not connect to", port)
            return None

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
        returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)

        return clientID, robot, MotorE, MotorD, ball


    def normalize_speed(self, speed):
        return max(self.v_min_wheels, min(speed, self.v_max_wheels))


    def speed_pioneer(self, max_linear_speed, angular_speed, lock_stop_simulation, error_phi):
        if lock_stop_simulation == 1 and error_phi <= 0.08:
            return 0, 0, False

        distance_between_wheels = 381
        wheel_radio = 95

        cinematic_right_speed = ((2 * max_linear_speed + angular_speed * distance_between_wheels) / (2 * wheel_radio))
        cinematic_left_speed = ((2 * max_linear_speed - angular_speed * distance_between_wheels) / (2 * wheel_radio))

        return self.normalize_speed(cinematic_left_speed), self.normalize_speed(cinematic_right_speed), True


    def PID_controller(self, kp, ki, kd, delta_time, error, integral_error, fant, integral_part):
        filter = 1 / (max(abs(np.roots([kd, kp, ki]))) * 10)
        alpha_decay = mat.exp(-(delta_time / filter))
        alpha = 1 - alpha_decay
        integral_error = integral_error + error
        derivative_value = alpha_decay  * fant + alpha * error
        integral_saturation = 10

        if fant == 0:
            derivative_error = (derivative_value / delta_time)
        else:
            derivative_error = (float((derivative_value - fant) / delta_time))

        if integral_part > integral_saturation:
            integral_part = integral_saturation
        elif integral_part < -integral_saturation:
            integral_part = -integral_saturation
        else:
            integral_part = ki * integral_error * delta_time

        PID = kp * error + integral_part + derivative_error * kd
        return PID, derivative_value, integral_error, integral_part


    def Robot_Pioneer(self, pid_params, delta_time):

        (clientID, robot, left_motor, right_motor, ball) = self.connect_pioneer(19999)

        if sim.simxGetConnectionId(clientID) == -1:
            print("Failed to connect to CoppeliaSim.")
            return


        kpi, kii, kdi = pid_params

        omega_ant = 0
        is_running = True
        number_iterations = 0
        integral_error = 0
        derivative_value = 0
        integral_part = 0

        while is_running:

            s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)

            if number_iterations <= 1:
                self._stop_robot(clientID, left_motor, right_motor)

            else:
                s, ball_pos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                self.phi = orientation[2]

                error_distance = math.sqrt((ball_pos[1] - positiona[1]) ** 2 + (ball_pos[0] - positiona[0]) ** 2)

                if error_distance >= self.Min_error_distance:
                    phid = math.atan2(ball_pos[1] - positiona[1], ball_pos[0] - positiona[0])
                    controller_linear = self.v_linear * error_distance
                    lock_stop_simulation = 0

                else:
                    phid = 1.57  # 90
                    controller_linear = 0
                    lock_stop_simulation = 1


                error_phi = phid - self.phi
                pid, derivative_value, integral_error, integral_part = (self.PID_controller(kpi, kii, kdi, delta_time, error_phi, integral_error, derivative_value, integral_part))

                if pid >= 100 or pid <= -100:
                    pid = omega_ant
                else:
                    omega_ant = pid

                left_speed, right_speed, is_running = self.speed_pioneer(controller_linear, pid, lock_stop_simulation, abs(phid - self.phi))

                sim.simxSetJointTargetVelocity(clientID, left_motor, left_speed, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID, right_motor, right_speed, sim.simx_opmode_blocking)


            number_iterations += 1
            self.y_out.append(positiona[1])
            self.x_out.append(positiona[0])

        self._save_path_to_csv("Pioneer_experiment.csv")


    def _stop_robot(self, clientID, left_motor, right_motor):
        sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_blocking)


    def _save_path_to_csv(self, filename):
        if len(self.y_out) != len(self.x_out):
            raise ValueError("Mismatch in recorded X and Y positions")

        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x_out', 'y_out'])
            for x, y in zip(self.x_out, self.y_out):
                writer.writerow([x, y])
        print(f"Data saved to {filename}")


if __name__ == "__main__":
    pioneer = Pioneer()


    pid_params = [0.3413, 0.1230, 0.0049]
    deltaT = 0.05
    pioneer.Robot_Pioneer(pid_params, deltaT)

