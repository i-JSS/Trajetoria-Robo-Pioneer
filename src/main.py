import csv
import math
import math as mat
import numpy as np
from src.lib import sim

class Pioneer():

    def __init__(self):
        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max_wheels = 15
        self.v_min_wheels = -15
        self.v_linear = 15
        self.posError = []
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
            return 0, 0, 0

        distance_between_wheels = 381
        wheel_radio = 95

        cinematic_right_speed = ((2 * max_linear_speed + angular_speed * distance_between_wheels) / (2 * wheel_radio))
        cinematic_left_speed = ((2 * max_linear_speed - angular_speed * distance_between_wheels) / (2 * wheel_radio))

        return self.normalize_speed(cinematic_left_speed), self.normalize_speed(cinematic_right_speed), 1


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


    def Robot_Pioneer(self, x, deltaT, filename):

        kpi = x[0]
        kii = x[1]
        kdi = x[2]

        (clientID, robot, motorE, motorD, ball) = self.connect_pioneer(19999)

        raizes = np.roots([kdi, kpi, kii])
        acumulate_error = 0
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        omega_ant = 0
        a = 1
        angulo_anterior_phid = 0
        angulo_anterior_phi_robot = 0


        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        fant_phi = 0
        Integral_part_phi = 0

        if (sim.simxGetConnectionId(clientID) != -1):


            while (a == 1):


                if Number_Iterations <= 1:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    self.phi = 0
                    sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
                else:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1,
                                                                           sim.simx_opmode_blocking)
                    signed = 1
                    phi_robot = orientation[2]
                    self.phi = phi_robot

                    ### Calculate the distance error, the robot stop when arrive the ball ###

                    error_distance = math.sqrt((ballPos[1] - positiona[1]) ** 2 + (ballPos[0] - positiona[0]) ** 2)

                    # print(f'Angle robot ==> {angle_robot}')

                    if error_distance >= self.Min_error_distance:
                        ### Calculate the phid (see georgia tech course) ###

                        phid = math.atan2(ballPos[1] - positiona[1], ballPos[0] - positiona[0])

                        #### Proportional controller of liner velocity ####

                        controller_Linear = self.v_linear * error_distance
                        # controller_Linear = self.v_linear
                        lock_stop_simulation = 0

                    else:

                        phid = 1.57  # 90
                        controller_Linear = 0
                        lock_stop_simulation = 1

                    ### Phi error to send the PID controller

                    # phid = phid + 1.5708  # sum 90

                    # Calcula la diferencia entre el ángulo actual y el anterior
                    diferencia_phid = phid - angulo_anterior_phid
                    diferencia_phi = self.phi - angulo_anterior_phi_robot
                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phid > math.pi:
                        phid -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phid < -math.pi:
                        phid += 2 * math.pi

                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phi > math.pi:
                        self.phi -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phi < -math.pi:
                        self.phi += 2 * math.pi

                    # Actualiza el ángulo anterior
                    angulo_anterior_phid = phid
                    angulo_anterior_phi_robot = self.phi

                    print(f'phid == > {phid}, self.phi ==> {self.phi}, error ==> {phid - self.phi}')
                    error_phi = phid - self.phi

                    ### Acumulative distance error ###

                    acumulate_error = acumulate_error + abs(phid - self.phi)

                    ### Implement the PID controller ###

                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_controller(kpi, kii, kdi, deltaT,
                                                                                               error_phi, interror_phi,
                                                                                               fant_phi,
                                                                                               Integral_part_phi)

                    if omega >= 100 or omega <= -100:
                        omega = omega_ant
                    else:
                        omega_ant = omega

                    self.posError.append(error_distance)

                    ### Calculate the speed right and left based on the topology robot ###

                    vl, vd, a = self.speed_pioneer(controller_Linear, omega, lock_stop_simulation,
                                                   abs(phid - self.phi))

                    print(f'Speed lef == {vl}, Speed Right == {vd}')
                    ### Send the speed values to coppeliasim simulato ###

                    sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

                    ### update the time simulation and the simulation iteration

                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)
                if Number_Iterations >= 60:
                    a == 0
                # time.sleep(0.5)
                ### Save the robot position ###ç
                # Detener la simulación
                self.y_out.append(positiona[1])
                self.x_out.append(positiona[0])
            if len(self.y_out) != len(self.x_out):
                raise ValueError("self.y_out and self.x_out must be of the same length")

                # Open the CSV file for writing
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)

                # Write the header
                writer.writerow(['x_out', 'y_out'])

                # Write the data rows
                for x, y in zip(self.x_out, self.y_out):
                    writer.writerow([x, y])

            print(f"Data saved to {filename}")


if __name__ == "__main__":
    Rvss = 1.6
    RPioneer = 95
    FS = RPioneer / Rvss
    crb01 = Pioneer()
    filename = "Pioneer_experiment.csv"
    ### DE  Experiment best 0 0.4172###
    kpi_DE = [0.3629, 0.3609, 0.8000, 0.3746, 0.3432]
    kii_DE = [0.1891, 0.3841, 0.0479, 0.0001, 0.0001]
    kdi_DE = [0.0001, 0.0039, 0.0001, 0.0001, 0.0001]
    ### MFO  Experiment best 3 0.3736###
    kpi_MFO = [0.3902, 0.3504, 0.3201, 0.3278, 0.3413]
    kii_MFO = [0.3468, 0.2910, 0.0001, 0.0774, 0.1230]
    kdi_MFO = [0.0001, 0.0050, 0.0001, 0.0002, 0.0049]
    x = [kpi_MFO[4], kii_MFO[4], kdi_MFO[4]]
    deltaT = 0.05
    crb01.Robot_Pioneer(x, deltaT, filename)

