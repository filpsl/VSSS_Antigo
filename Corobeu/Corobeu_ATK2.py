import socket
import time
import math
import struct
import signal
from configs import wrapper_pb2 as wr
import sys
from configs.config import IP_ARES, ID_ARES, COR_DO_TIME, GOL_ADVERSARIO

# CONSTANTES GLOBAIS

# Dimensões do campo (AJUSTE ESTES VALORES COM OS DA SUA CENA!)
FIELD_X_MAX = 0.75
FIELD_X_MIN = -0.75
FIELD_Y_MAX = 0.65
FIELD_Y_MIN = -0.65

# Parâmetros da Estratégia
WALL_MARGIN = 0.08 # Margem para evitar paredes preventivamente
GOAL_X = GOL_ADVERSARIO # Posição do gol adversário
GOAL_Y = 0.0 # Posição do gol adversário
POSITIONING_DISTANCE = 0.08 # Distância para posicionamento antes do chute
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
def init_vision_socket(VISION_IP="224.5.23.2", VISION_PORT=10015):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.setsockopt(
        socket.IPPROTO_IP,
        socket.IP_ADD_MEMBERSHIP,
        struct.pack("=4sl", socket.inet_aton(VISION_IP), socket.INADDR_ANY),
    )
    sock.bind((VISION_IP, VISION_PORT))
    sock.setblocking(False)
    return sock

#CLASSE PRINCIPAL DO ROBÔ
class Corobeu:
    def __init__(self, ROBOT_IP, ROBOT_PORT, ROBOT_ID, VISION_SOCK, kp, ki, kd, dt):
        self.robot_ip = ROBOT_IP
        self.robot_port = ROBOT_PORT
        self.robot_id = ROBOT_ID
        self.vision_sock = VISION_SOCK
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        # --- Variáveis de Estado do PID (precisam ser lembradas entre as chamadas) ---
        self.integral_range = 30
        self.interror = [0 for _ in range(self.integral_range)]
        self.Integral_part = 0
        self.f_ant = 0
        
        # --- Velocidades do robô ---   
        self.v_max = 255
        self.v_min = -255
        self.v_linear = 180
        
        # --- NOVO: Parâmetros específicos para o PUSH/CHUTE ---
        self.v_push = 255  # Uma velocidade de push dedicada (um pouco menor que a de approach)
        self.kp_push = 4 # Um ganho P "suave" apenas para o push, para evitar oscilação
        
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
        
        # --- Identificação da cor do time ---
        if COR_DO_TIME == 1:
            self._robot_attr = "robots_blue"
        elif COR_DO_TIME == 0:
            self._robot_attr = "robots_yellow"
        else:
            raise ValueError(
                f"COR_DO_TIME: {COR_DO_TIME} é inválido, altere-o no 'config_ideal.py'."
            )
        
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
        
        # ---  Enviando as velocidades ao contrário por conta da eletrônica bugada ---
        return int(vl), int(vr)
    
    
    def send_speed(self, speed_left, speed_right):
        direction_left = 1 if speed_left >= 0 else 0
        direction_right = 1 if speed_right >= 0 else 0
        combined_value = (
            (abs(speed_left) << 24)
            | (abs(speed_right) << 16)
            | (direction_left << 8)
            | direction_right
        )
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(combined_value.to_bytes(4, byteorder="little"))
        except Exception as e:
            print(f"Erro ao enviar dados: {e}")
    

    def get_position(self):
        data = None # Armazena o último pacote lido
        
        while True:
            try:
                # Tenta ler o próximo pacote do buffer
                packet_data, _ = self.vision_sock.recvfrom(1024)
                data = packet_data # Se conseguiu, guarda esse (que é o "mais novo" até agora)
            
            except (BlockingIOError, socket.error):
                # EXCEÇÃO!
                # Isso é BOM. Significa que o buffer está VAZIO.
                # O último pacote que guardamos em 'data' é o mais recente.
                break # Sai do loop 'while True'
            
            except Exception as e:
                # Algum outro erro sério
                print(f"Erro inesperado no socket de visão: {e}")
                return None, None, None, None, None

        # Se saímos do loop, ou o buffer estava vazio (data=None) 
        # ou temos o pacote mais recente (data=packet_data)

        if data is None:
            # O buffer estava vazio, não recebemos nada desta vez.
            return None, None, None, None, None

        # Se chegamos aqui, 'data' contém o pacote mais FRESCO.
        # Agora sim, fazemos o parse:
        frame = wr.SSL_WrapperPacket().FromString(data)
        robots = getattr(frame.detection, self._robot_attr)
        
        for robot in robots:
            if robot.robot_id == self.robot_id:
                return (
                    robot.x / 1000,
                    robot.y / 1000,
                    robot.orientation,
                    frame.detection.balls[0].x / 1000,
                    frame.detection.balls[0].y / 1000,
                )
        
        return None, None, None, None, None
    
    
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
        # print(f"P: {kp*error}, I: {self.Integral_part}, D: {deerror * kd}, PID: {pid}")
        return pid
    
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
            
        # print(f"--- WALL AVOIDANCE ---")
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
        
        return self.speed_control(U, omega)

    # --- FUNÇÕES UTILITÁRIAS ---

    def wrap_angle(self, angle):
        """Garante que um ângulo esteja no intervalo de -pi a pi."""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def off(self, signum=None, frame=None):
        """Função para desligar o robô de forma segura."""
        print("Desligando o robô...")
        self.send_speed(0, 0)
        sys.exit(0)

    # --- LOOP PRINCIPAL (run_strategy) ---

    def run_strategy(self):
        """
        Loop principal que executa a máquina de estados do robô.
        """
        
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
            x, y, robotOri, ball_x, ball_y = self.get_position()
            robotPos = [x, y]
            ballPos = [ball_x, ball_y]
            
            if robotPos and robotOri:
                # --- ALTERADO: Atualiza a posição anterior antes da nova ---
                self.previous_robot_position['x'] = self.robot_position['x']
                self.previous_robot_position['y'] = self.robot_position['y']

                self.robot_position['x'] = robotPos[0]
                self.robot_position['y'] = robotPos[1]
                self.robot_position['phi'] = robotOri
            
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
                try:
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
                except:
                    continue

            # 3. EXECUTAR A LÓGICA DO ESTADO ATUAL

            if self.current_state == STATE_POSITIONING:
                # print(f"ESTADO: [POSICIONANDO]")
                target_point = self.calculate_target_point() 
                phid = math.atan2(target_point['y'] - self.robot_position['y'], 
                                  target_point['x'] - self.robot_position['x'])
                error_phi = self.wrap_angle(phid - self.robot_position['phi'])
                # print(error_phi)

                # Usa o PID completo e a velocidade linear normal
                omega = self.pid_controller(error_phi, integral_counter)
                vl, vr = self.speed_control(self.v_linear, omega)
                integral_counter += 1
                if integral_counter >= self.integral_range:
                    integral_counter = 0

            # --- ESTADO DE CHUTE ATUALIZADO ---
            elif self.current_state == STATE_PUSHING_BALL:
                # print(f"ESTADO: [CHUTANDO PARA O GOL]")

                # Mira no CENTRO DO GOL
                phid = math.atan2(GOAL_Y - self.robot_position['y'], 
                                  GOAL_X - self.robot_position['x'])
                error_phi = self.wrap_angle(phid - self.robot_position['phi'])

                # --- USA O CONTROLADOR P SIMPLES ---
                # Usa o kp_push "suave" que definimos no __init__
                omega = self.pid_controller(error_phi, integral_counter)
                integral_counter += 1
                if integral_counter >= self.integral_range:
                    integral_counter = 0

                # Usa a velocidade de push dedicada
                vl, vr = self.speed_control(self.v_push, omega)

                # 3. EXECUTAR A LÓGICA DO ESTADO ATUAL  

            elif self.current_state == STATE_AVOIDING_WALL:
                # print(f"ESTADO: [EVITANDO PAREDE]")
                vl, vr = self.perform_wall_avoidance()
            
            # --- NOVO: Execução do estado de fuga ---
            elif self.current_state == STATE_UNSTICKING:
                # print(f"ESTADO: [FUGA DE COLISÃO]")
                vl, vr = self.perform_unstick_maneuver()
            
            else: # Estado desconhecido
                vl, vr = 0, 0

            # 4. ENVIAR COMANDOS AOS MOTORES
            self.send_speed(vl, vr)
            # self.send_speed(0, 0)
            
            if self.current_state != previous_state:
                self.reset_pid()

# PONTO DE ENTRADA DO PROGRAMA
if __name__ == "__main__":
    VISION_IP = "224.5.23.2"
    VISION_PORT = 10015
    ROBOT_IP = IP_ARES
    ROBOT_ID = ID_ARES
    ROBOT_PORT = 80

    kp = 7
    kd = 0.8
    ki = 0.1
    
    # kd = 0
    # ki = 0

    dt = 0.05

    # kp = 3.46
    # ki = 0.
    # kd = 2.46
    
    # kp = 3.45051784
    # ki = 0.02365731 # PSO
    # kd = 0.06288346

    vision_sock = init_vision_socket(VISION_IP, VISION_PORT)
    crb01 = Corobeu(ROBOT_IP, ROBOT_PORT, ROBOT_ID, vision_sock, kp, ki, kd, dt)
    crb01.run_strategy()
