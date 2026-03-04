import time
import math
import sims.sim as sim
import signal
import sys

# CONSTANTES GLOBAIS

# Dimensões do campo (AJUSTE ESTES VALORES COM OS DA SUA CENA!)
FIELD_X_MAX = 0.75
FIELD_X_MIN = -0.75
FIELD_Y_MAX = 0.65
FIELD_Y_MIN = -0.65

# Parâmetros da Estratégia
WALL_MARGIN = 0.03 # Margem para evitar paredes preventivamente
GOAL_X = -0.75 # Posição do gol adversário
GOAL_Y = 0.0 # Posição do gol adversário
POSITIONING_DISTANCE = 0.09 # Distância para posicionamento antes do chute
KICK_ALIGNMENT_DISTANCE = 0.05 # Quão perto do "ponto-alvo" o robô precisa estar
KICK_ALIGNMENT_ANGLE = math.radians(35) # Tolerância de ângulo (em radianos) para o chute
LOST_BALL_DISTANCE = 0.15 # Se a bola estiver a mais de 15cm, perdemos

# --- NOVO: Parâmetros para detecção de robô preso e fuga ---
STUCK_VELOCITY_THRESHOLD = 0.01 # Velocidade (m/s) abaixo da qual o robô é considerado parado
STUCK_PWM_THRESHOLD = 3.0       # Velocidade mínima enviada aos motores para considerar a detecção
UNSTICK_DURATION = 1.0          # Duração da manobra de fuga em segundos

# Nomes dos Estados da Máquina de Estados
STATE_PUSHING_BALL = "pushing_ball"
STATE_POSITIONING = "positioning_for_kick"
STATE_AVOIDING_WALL = "avoiding_wall"
STATE_UNSTICKING = "unsticking" 


# FUNÇÃO DE CONEXÃO (pode ser mantida fora da classe)
def connect_CRB(port):
    """
    Função usada para se comunicar com o CoppeliaSim
    """
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID != -1:
        print("Conectado ao CoppeliaSim na porta", port)
    else:
        print("Falha ao conectar na porta", port)
        sys.exit()

    # Pega os handles (identificadores) dos objetos na cena
    returnCode, robot = sim.simxGetObjectHandle(clientID, 'robot01', sim.simx_opmode_blocking)
    returnCode, motorE = sim.simxGetObjectHandle(clientID, 'motorL01', sim.simx_opmode_blocking)
    returnCode, motorD = sim.simxGetObjectHandle(clientID, 'motorR01', sim.simx_opmode_blocking)
    returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)
    
    return clientID, robot, motorE, motorD, ball

#CLASSE PRINCIPAL DO ROBÔ
class Corobeu:
    def __init__(self, kp, ki, kd, dt, pid_max):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.pid_max = pid_max
        
        # --- Filtro para o Termo Derivativo ---
        self.filter_alpha = 0.5 
        
        # --- Variáveis de Estado do PID (precisam ser lembradas entre as chamadas) ---
        self.integral_range = 30
        self.interror = [0 for _ in range(self.integral_range)]
        self.Integral_part = 0
        self.f_ant = 0
        
        # --- Parâmetros Físicos e de Movimento do Robô ---
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 7
        
        # --- NOVO: Parâmetros específicos para o PUSH/CHUTE ---
        self.v_push = 7  # Uma velocidade de push dedicada (um pouco menor que a de approach)
        self.kp_push = 2.5 # Um ganho P "suave" apenas para o push, para evitar oscilação
        
        # --- Variáveis da Máquina de Estados ---
        self.current_state = STATE_POSITIONING
        self.robot_position = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.ball_position = {'x': 0.0, 'y': 0.0}
        
        # --- NOVO: Variáveis para detecção de robô preso ---
        self.previous_robot_position = {'x': 0.0, 'y': 0.0}
        self.last_vl = 0
        self.last_vr = 0
        self.unstick_start_time = 0
        
        # --- Temporizador para o loop de controle ---
        self.last_update_time = time.time()
        
        # --- Configura o desligamento seguro ---
        signal.signal(signal.SIGINT, self.off)
        signal.signal(signal.SIGTERM, self.off)

    def speed_control(self, U, omega) -> tuple:
        """
        Calcula a velocidade de cada roda e escala para não passar do limite máximo,
        mantendo a proporção de giro.
        """
        vr = (2 * U + omega * 7.5) / 3.0 # Roda Direita (Right)
        vl = (2 * U - omega * 7.5) / 3.0 # Roda Esquerda (Left)
        
        # Encontra o fator de escala necessário
        max_speed = max(abs(vr), abs(vl))
        if max_speed > self.v_max:
            scale_factor = self.v_max / max_speed
            vr *= scale_factor
            vl *= scale_factor
        
        if math.isnan(vr) or math.isnan(vl):
            vr, vl = 0, 0
            
        # --- NOVO: Armazena as últimas velocidades enviadas ---
        self.last_vl = vl
        self.last_vr = vr
        
        return int(vl), int(vr)

    def pid_controller(self, error, integral_counter) -> float:
        """
        Controlador PID robusto com filtro na derivada e anti-windup na integral.
        """
        Integral_saturation = 5
        raizes = math.sqrt(kd), math.sqrt(kp), math.sqrt(ki)
        Filter_e = 1 / (max(raizes) * 10)
        unomenosalfaana = math.exp(-(self.dt / Filter_e))
        alfaana = 1 - unomenosalfaana
        self.interror[integral_counter] = error
        f = unomenosalfaana * self.f_ant + alfaana * error
        deerror = (f - self.f_ant) / self.dt if self.f_ant != 0 else f / self.dt
        self.Integral_part = min(
            max(
                self.Integral_part + ki * sum(self.interror) * self.dt,
                -Integral_saturation,
            ),
            Integral_saturation,
        )
        self.f_ant = f
        pid = kp * error + self.Integral_part + deerror * kd
        # if (self.current_state == STATE_PUSHING_BALL):
        pid_saturado = max(min(pid, self.pid_max), -self.pid_max)
        print(f"P: {kp*error}, I: {self.Integral_part}, D: {deerror * kd}, PID: {pid_saturado}")
        return pid_saturado
    
    def calculate_target_point(self) -> dict:
        """
        Calcula o ponto atrás da bola, alinhado com o gol.
        Retorna um dicionário {'x': target_x, 'y': target_y}
        """
        ball_x = self.ball_position['x']
        ball_y = self.ball_position['y']

        # 1. Calcular o vetor do gol para a bola
        vec_x = ball_x - GOAL_X
        vec_y = ball_y - GOAL_Y

        # 2. Calcular a magnitude (distância) desse vetor
        magnitude = math.sqrt(vec_x**2 + vec_y**2)

        # Evitar divisão por zero se a bola estiver exatamente no gol
        if magnitude == 0:
            return {'x': ball_x, 'y': ball_y} 

        # 3. Normalizar o vetor (transformá-lo em um vetor de comprimento 1)
        norm_x = vec_x / magnitude
        norm_y = vec_y / magnitude

        # 4. Calcular o ponto-alvo: começa na bola e "anda" para trás na direção do vetor normalizado
        target_x = ball_x + norm_x * POSITIONING_DISTANCE
        target_y = ball_y + norm_y * POSITIONING_DISTANCE

        # --- NOVO (Dia 6 - Edge Case): Garantir que o ponto-alvo não esteja fora do campo
        target_x = max(min(target_x, FIELD_X_MAX - WALL_MARGIN), FIELD_X_MIN + WALL_MARGIN)
        target_y = max(min(target_y, FIELD_Y_MAX - WALL_MARGIN), FIELD_Y_MIN + WALL_MARGIN)

        return {'x': target_x, 'y': target_y}
    

    def reset_pid(self):
        """Reseta os termos de estado do controlador PID."""
        self.interror = [0 for _ in range(self.integral_range)]
        self.Integral_part = 0
        self.f_ant = 0


    def is_near_wall(self):
        """Verifica se o robô está na margem de perigo perto de uma parede."""
        x = self.robot_position['x']
        y = self.robot_position['y']
        
        if (x > FIELD_X_MAX - WALL_MARGIN or
            x < FIELD_X_MIN + WALL_MARGIN or
            y > FIELD_Y_MAX - WALL_MARGIN or
            y < FIELD_Y_MIN + WALL_MARGIN):
            return True
        return False


    def is_stuck(self):
        """Verifica se o robô está preso comparando comando de motor com movimento real."""
        # Calcula a distância percorrida desde a última verificação
        dx = self.robot_position['x'] - self.previous_robot_position['x']
        dy = self.robot_position['y'] - self.previous_robot_position['y']
        distance_moved = math.sqrt(dx**2 + dy**2)
        
        # Calcula a velocidade real
        actual_velocity = distance_moved / self.dt
        
        # Verifica se o comando enviado aos motores era alto
        commanded_speed = max(abs(self.last_vl), abs(self.last_vr))
        
        # Condição de "preso": comando alto, mas velocidade real muito baixa
        if commanded_speed > STUCK_PWM_THRESHOLD and actual_velocity < STUCK_VELOCITY_THRESHOLD:
            return True
        return False

    def perform_wall_avoidance(self):
        """
        Calcula as velocidades para o robô se afastar da parede e virar para o centro.
        Retorna (vl, vr).
        """
        robot_x = self.robot_position['x']
        robot_y = self.robot_position['y']
        robot_phi = self.robot_position['phi']

        # 1. Mira no centro do campo (0,0) para se afastar da parede
        target_angle_to_center = math.atan2(-robot_y, -robot_x)
        
        # 2. Calcula o erro de ângulo
        error_phi = self.wrap_angle(target_angle_to_center - robot_phi)
        
        # 3. Usa um controlador Proporcional simples para girar rápido
        kp_avoid = 3.0 
        omega = kp_avoid * error_phi
        
        # 4. Define uma velocidade linear para trás para se afastar
        U = -self.v_linear / 2 
        
        # Se o robô já está virado para o centro, ele para de dar ré e apenas gira
        if abs(error_phi) < math.radians(45): # 45 graus
            U = self.v_linear
            
        print(f"--- WALL AVOIDANCE ---")
        return self.speed_control(U, omega)

    # --- NOVO: Método para executar a manobra de fuga ---
    def perform_unstick_maneuver(self):
        """
        Executa uma manobra de ré e giro para destravar o robô.
        Retorna (vl, vr).
        """
        # A lógica é simples: dar ré e girar para o centro ao mesmo tempo.
        # Isso geralmente é suficiente para sair de quinas e paredes.
        robot_x = self.robot_position['x']
        robot_y = self.robot_position['y']

        # Mira no centro do campo
        target_angle_to_center = math.atan2(-robot_y, -robot_x)
        error_phi = self.wrap_angle(target_angle_to_center - self.robot_position['phi'])
        
        # Controlador P para o giro
        kp_unstick = 3.0
        omega = kp_unstick * error_phi
        
        # Velocidade de ré constante
        U = -self.v_linear
        
        print(f"--- MANOBRA DE FUGA --- Dando ré e girando para o centro.")
        
        return self.speed_control(U, omega)

    # --- FUNÇÕES UTILITÁRIAS ---

    def wrap_angle(self, angle):
        """Garante que um ângulo esteja no intervalo de -pi a pi."""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def off(self, signum=None, frame=None):
        """Função para desligar o robô de forma segura."""
        print("Desligando o robô...")
        # Adicionar aqui a lógica para parar os motores no simulador se necessário
        sys.exit(0)

    # --- LOOP PRINCIPAL (run_strategy) ---

    def run_strategy(self):
        """
        Loop principal que executa a máquina de estados do robô.
        """
        (clientID, robot, motorE, motorD, ball) = connect_CRB(19995)
        
        print("Estratégia iniciada. Pressione Ctrl+C para parar.")
        integral_counter = 0
        
        # Loop principal de estratégia
        while True:
            previous_state = self.current_state
            
            current_time = time.time()
            if (current_time - self.last_update_time) < self.dt:
                continue # Garante que o loop rode na frequência definida por dt
            self.last_update_time = current_time

            # 1. ATUALIZAR DADOS DO MUNDOintegral_counter
            s, robotPos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
            s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
            s, robotOri = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
            
            if robotPos and robotOri:
                # --- ALTERADO: Atualiza a posição anterior antes da nova ---
                self.previous_robot_position['x'] = self.robot_position['x']
                self.previous_robot_position['y'] = self.robot_position['y']

                self.robot_position['x'] = robotPos[0]
                self.robot_position['y'] = robotPos[1]
                self.robot_position['phi'] = self.wrap_angle(robotOri[2] - math.pi/2)
            
            if ballPos:
                self.ball_position['x'] = ballPos[0]
                self.ball_position['y'] = ballPos[1]

            # Prioridade 1: Sair de "preso"
            if self.current_state == STATE_UNSTICKING:
                if time.time() - self.unstick_start_time > UNSTICK_DURATION:
                    self.current_state = STATE_POSITIONING # Volta a se posicionar

            # Prioridade 2: Detectar "preso"
            if self.is_stuck() and self.current_state != STATE_UNSTICKING:
                self.current_state = STATE_UNSTICKING
                self.unstick_start_time = time.time()

            # Prioridade 3: Evitar paredes (a menos que esteja "preso")
            elif self.current_state != STATE_UNSTICKING:
                if self.is_near_wall():
                    self.current_state = STATE_AVOIDING_WALL

                # --- LÓGICA DE ATAQUE (Prioridade 4) ---
                else:
                    # Distância do robô até a BOLA (não o ponto-alvo)
                    dist_to_ball = math.sqrt((self.ball_position['x'] - self.robot_position['x'])**2 + 
                                             (self.ball_position['y'] - self.robot_position['y'])**2)

                    # Ângulo que o robô *deveria* ter (mirando no gol)
                    angle_to_goal = math.atan2(GOAL_Y - self.robot_position['y'], 
                                               GOAL_X - self.robot_position['x'])
                    error_to_goal_angle = self.wrap_angle(angle_to_goal - self.robot_position['phi'])


                    # --- LÓGICA DE TRANSIÇÃO ATUALIZADA ---

                    if self.current_state == STATE_PUSHING_BALL:
                        # CONDIÇÃO DE SAÍDA: Quando parar de chutar?
                        # 1. Se perdemos a bola (ela foi para longe)
                        # 2. Se erramos o alvo (viramos mais de ~45 graus do gol)
                        if (dist_to_ball > LOST_BALL_DISTANCE or 
                            abs(error_to_goal_angle) > math.radians(45)):

                            self.current_state = STATE_POSITIONING # Volta a se posicionar
                            self.reset_pid

                    elif self.current_state == STATE_POSITIONING:
                        # CONDIÇÃO DE ENTRADA: Quando começar a chutar?

                        # Distância do robô até o ponto-alvo atrás da bola
                        target_point = self.calculate_target_point()
                        dist_to_target = math.sqrt((target_point['x'] - self.robot_position['x'])**2 + 
                                                   (target_point['y'] - self.robot_position['y'])**2)
                        is_aligned_for_kick = (dist_to_target < KICK_ALIGNMENT_DISTANCE and 
                                               abs(error_to_goal_angle) < KICK_ALIGNMENT_ANGLE)

                        if is_aligned_for_kick:
                            self.current_state = STATE_PUSHING_BALL
                            self.reset_pid

                    else:
                         # Se saímos da parede/preso, voltamos a nos posicionar
                        self.current_state = STATE_POSITIONING
                        self.reset_pid


            # 3. EXECUTAR A LÓGICA DO ESTADO ATUAL

            if self.current_state == STATE_POSITIONING:
                print(f"ESTADO: [POSICIONANDO]")
                target_point = self.calculate_target_point() 
                phid = math.atan2(target_point['y'] - self.robot_position['y'], 
                                  target_point['x'] - self.robot_position['x'])
                error_phi = self.wrap_angle(phid - self.robot_position['phi'])

                # Usa o PID completo e a velocidade linear normal
                omega = self.pid_controller(error_phi, integral_counter)
                vl, vr = self.speed_control(self.v_linear, omega)
                integral_counter += 1
                if integral_counter >= self.integral_range:
                    integral_counter = 0

            # --- ESTADO DE CHUTE ATUALIZADO ---
            elif self.current_state == STATE_PUSHING_BALL:
                print(f"ESTADO: [CHUTANDO PARA O GOL]")

                # Mira no CENTRO DO GOL
                phid = math.atan2(GOAL_Y - self.robot_position['y'], 
                                  GOAL_X - self.robot_position['x'])
                error_phi = self.wrap_angle(phid - self.robot_position['phi'])

                # --- USA O CONTROLADOR P SIMPLES ---
                # Usa o kp_push "suave" que definimos no __init__
                omega = self.kp_push * error_phi

                # Usa a velocidade de push dedicada
                vl, vr = self.speed_control(self.v_push, omega)

                # 3. EXECUTAR A LÓGICA DO ESTADO ATUAL  

            elif self.current_state == STATE_AVOIDING_WALL:
                print(f"ESTADO: [EVITANDO PAREDE]")
                vl, vr = self.perform_wall_avoidance()
            
            # --- NOVO: Execução do estado de fuga ---
            elif self.current_state == STATE_UNSTICKING:
                print(f"ESTADO: [FUGA DE COLISÃO]")
                vl, vr = self.perform_unstick_maneuver()
            
            else: # Estado desconhecido
                vl, vr = 0, 0

            # 4. ENVIAR COMANDOS AOS MOTORES
            sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, vr, sim.simx_opmode_blocking)
            
            if self.current_state != previous_state:
                self.reset_pid()

# PONTO DE ENTRADA DO PROGRAMA
if __name__ == "__main__":
    
    # --- Parâmetros de Controle (Tuning do PID) ---
    # kp = 3.45051784 
    # ki = 0.02365731 # PSO
    # kd = 0.06288346
    
    kp = 3.09355224
    ki = 0.02755811
    kd = 0.0328926
    
    # kp = 3.45051784 
    # ki = 0.02365731 # EMPIRICO
    # kd = 0.06288346
    pid_max = 2
    
    dt = 0.05  # Tempo de ciclo do controlador (50 ms)
    
    # --- Inicialização e Execução ---
    crb01 = Corobeu(kp, ki, kd, dt, pid_max)
    crb01.run_strategy()
