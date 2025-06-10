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

    def connect_Pioneer(self, port):

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

    def Speed_Pioneer(self, U, omega, lock_stop_simulation, signed, error_phi):

        L = 381  # Distance between the wheels
        R = 95  # Wheel radio

        vd = ((2 * (U) + omega * L) / (2 * R))
        vl = ((2 * (U) - omega * L) / (2 * R))
        a = 1

        Max_Speed = self.v_max_wheels
        Min_Speed = self.v_min_wheels

        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        if vl >= Max_Speed:
            vl = Max_Speed
        if vl <= Min_Speed:
            vl = Min_Speed

        if lock_stop_simulation == 1 and error_phi <= 0.08:
            a = 0
            vl = 0
            vd = 0

        return vl, vd, a

    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

        Integral_saturation = 10

        ### Find the filter e ####

        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)

        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f / deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        PID = kp * error + Integral_part + deerror * kd

        return PID, f, interror, Integral_part

    def Robot_Pioneer(self, x, deltaT, filename):

        kpi = x[0]
        kii = x[1]
        kdi = x[2]

        (clientID, robot, motorE, motorD, ball) = self.connect_Pioneer(19999)

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

                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                               error_phi, interror_phi,
                                                                                               fant_phi,
                                                                                               Integral_part_phi)

                    if omega >= 100 or omega <= -100:
                        omega = omega_ant
                    else:
                        omega_ant = omega

                    self.posError.append(error_distance)

                    ### Calculate the speed right and left based on the topology robot ###

                    vl, vd, a = self.Speed_Pioneer(controller_Linear, omega, lock_stop_simulation, signed,
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

