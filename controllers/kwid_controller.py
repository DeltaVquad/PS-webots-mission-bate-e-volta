import math
from controller import Robot, Supervisor

# Classe para um controlador PID simples
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._integral = 0
        self._previous_error = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self._integral += error * dt
        derivative = (error - self._previous_error) / dt
        self._previous_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self._integral = 0
        self._previous_error = 0

# Classe principal do controlador do Drone
class QuadcopterController(Supervisor):
    def __init__(self):
        super().__init__()
        self.TIME_STEP = int(self.getBasicTimeStep())

        # --- Parâmetros Físicos (do nosso estudo) ---
        self.GRAVITY = 9.81
        self.DRONE_MASS = 0.75  # kg
        
        # Coeficientes de empuxo e torque (estimados)
        self.K_THRUST = 2.0e-5 # N/RPM^2 (ajustado empiricamente)
        self.K_TORQUE = 1.0e-6 # Nm/RPM^2 (ajustado empiricamente)

        # Distância do motor ao centro de massa
        self.L = 0.12 # metros

        # Limite de RPM (para evitar valores irreais)
        self.MAX_RPM = 18000
        
        # --- Obter Dispositivos ---
        self.imu = self.getDevice("imu")
        self.imu.enable(self.TIME_STEP)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.TIME_STEP)
        self.camera = self.getDevice("camera")
        self.camera.enable(self.TIME_STEP)

        # Motores
        self.motor_fr = self.getDevice("motor_fr")
        self.motor_fl = self.getDevice("motor_fl")
        self.motor_rr = self.getDevice("motor_rr")
        self.motor_rl = self.getDevice("motor_rl")
        
        self.motors = [self.motor_fr, self.motor_fl, self.motor_rr, self.motor_rl]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            
        # --- Alvos (Setpoints) Iniciais ---
        self.target_altitude = 1.0  # metros
        self.target_position_x = 0.0
        self.target_position_y = 0.0
        self.target_yaw = 0.0

        # --- Controladores PID ---
        # Atitude
        self.pid_roll = PIDController(kp=6.0, ki=0.2, kd=0.3, setpoint=0.0)
        self.pid_pitch = PIDController(kp=6.0, ki=0.2, kd=0.3, setpoint=0.0)
        self.pid_yaw = PIDController(kp=4.0, ki=0.1, kd=0.35, setpoint=self.target_yaw)
        
        # Posição (cascata)
        self.pid_altitude = PIDController(kp=1.5, ki=0.2, kd=0.8, setpoint=self.target_altitude)
        self.pid_pos_x = PIDController(kp=0.8, ki=0.05, kd=0.4, setpoint=self.target_position_x)
        self.pid_pos_y = PIDController(kp=0.8, ki=0.05, kd=0.4, setpoint=self.target_position_y)
        
        print("Controlador do Quadricóptero Inicializado!")

    def run(self):
        while self.step(self.TIME_STEP) != -1:
            # --- Leitura dos Sensores ---
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            altitude = self.gps.getValues()[2]
            pos_x = self.gps.getValues()[0]
            pos_y = self.gps.getValues()[1]
            
            # --- Lógica de Controle em Cascata ---
            
            # 1. Controle de Posição (calcula os setpoints de atitude desejados)
            # O PID de posição X calcula o ângulo de PITCH necessário para se mover em X
            pitch_setpoint = self.pid_pos_x.update(pos_x, self.TIME_STEP / 1000.0)
            # O PID de posição Y calcula o ângulo de ROLL necessário para se mover em Y
            roll_setpoint = -self.pid_pos_y.update(pos_y, self.TIME_STEP / 1000.0) # Negativo por convenção

            self.pid_pitch.set_setpoint(pitch_setpoint)
            self.pid_roll.set_setpoint(roll_setpoint)

            # 2. Controle de Atitude (calcula as correções de torque)
            roll_input = self.pid_roll.update(roll, self.TIME_STEP / 1000.0)
            pitch_input = self.pid_pitch.update(pitch, self.TIME_STEP / 1000.0)
            yaw_input = self.pid_yaw.update(yaw, self.TIME_STEP / 1000.0)
            
            # 3. Controle de Altitude (calcula o empuxo base)
            vertical_input = self.pid_altitude.update(altitude, self.TIME_STEP / 1000.0)
            
            # Empuxo base para pairar (hover)
            base_thrust = self.DRONE_MASS * self.GRAVITY
            
            # Empuxo total (hover + correção de altitude)
            total_thrust = base_thrust + vertical_input

            # --- Matriz de Alocação de Empuxo (Mixagem) ---
            # Converte empuxo e torques desejados em RPMs para cada motor
            
            # Rotação: FR(-), RL(-), FL(+), RR(+) onde + é CCW e - é CW
            # u1 = Empuxo, u2 = Torque Roll, u3 = Torque Pitch, u4 = Torque Yaw
            
            # Estas equações vêm da dinâmica Newton-Euler para um quad em configuração 'X'
            rpm_fr = total_thrust - roll_input + pitch_input - yaw_input
            rpm_fl = total_thrust + roll_input + pitch_input + yaw_input
            rpm_rr = total_thrust - roll_input - pitch_input + yaw_input
            rpm_rl = total_thrust + roll_input - pitch_input - yaw_input
            
            # Precisamos converter o "esforço" calculado para RPMs reais.
            # Esta é uma simplificação. Um modelo mais complexo usaria as fórmulas de empuxo/torque.
            # A constante de conversão é ajustada empiricamente para o voo estável.
            
            CONVERSION_FACTOR = 6000 # Empírico

            motor_rpm_fr = math.sqrt(max(0, rpm_fr)) * CONVERSION_FACTOR
            motor_rpm_fl = math.sqrt(max(0, rpm_fl)) * CONVERSION_FACTOR
            motor_rpm_rr = math.sqrt(max(0, rpm_rr)) * CONVERSION_FACTOR
            motor_rpm_rl = math.sqrt(max(0, rpm_rl)) * CONVERSION_FACTOR
            
            # Limita os RPMs
            motor_rpm_fr = min(self.MAX_RPM, motor_rpm_fr)
            motor_rpm_fl = min(self.MAX_RPM, motor_rpm_fl)
            motor_rpm_rr = min(self.MAX_RPM, motor_rpm_rr)
            motor_rpm_rl = min(self.MAX_RPM, motor_rpm_rl)

            # --- Aplica a velocidade aos motores ---
            # A velocidade do motor no Webots é em rad/s
            # RPM para rad/s: (RPM / 60) * 2 * pi
            
            self.motor_fr.setVelocity((motor_rpm_fr / 60) * 2 * math.pi)
            self.motor_fl.setVelocity((motor_rpm_fl / 60) * 2 * math.pi)
            self.motor_rr.setVelocity((motor_rpm_rr / 60) * 2 * math.pi)
            self.motor_rl.setVelocity((motor_rpm_rl / 60) * 2 * math.pi)

            # Debug
            # print(f"Alt: {altitude:.2f} | Roll: {math.degrees(roll):.2f} | Pitch: {math.degrees(pitch):.2f}")


if __name__ == "__main__":
    controller = QuadcopterController()
    controller.run()