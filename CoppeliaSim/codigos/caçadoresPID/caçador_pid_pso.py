import time
import math
import sims.sim as sim
import numpy as np

def connect_CRB(port):
        """""
        Function used to communicate with CoppeliaSim
            argument :
                Port (Integer) = used to CoppeliaSim (same CoppeliaSim)
                
            outputs : 
                clientID (Integer)  = Client number
                robot    (Integer)  = objecto robot
                MotorE   (Integer)  = Object motor left
                MotorD   (Integer)  = Object motor right
                ball     (Integer)  = Object ball on the scene
        """""

        ### Connect to coppeliaSim ###

        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Connect to", port)
        else:
            print("Can not connect to", port)

        ### Return the objects ###

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'robot01', 
                                                    sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'motorL01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'motorR01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)
               
        return clientID, robot, MotorE, MotorD, ball


def algoritmo_pso(S, N, iteracoes_max):
    
    # PARÂMETROS DO PSO
    S = S                               # Quantidade de partículas
    N = N                               # Número de dimensões
    iteracoes_max = iteracoes_max       # Iterações máximas antes do código parar
    c1 = 2.05                           # Coeficiente individual
    c2 = 2.05                           # Coeficiente social
    v_max_kp = 2                           # Velocidade máxima
    v_max_ki = 1
    v_max_kd = 0.2
    v_max = 0
    w0 = 0.9                            # Fator de inercia inicial
    w1 = 0.3                            # Fator de inercia final
    w_passo = (w1 - w0)/iteracoes_max   # Passo (diminuir o fator de inercial lentamente até o final)
    kp_max = 8
    ki_max = 3
    kd_max = 0.5
    
    # INICIALIZANDO OS PID
    coluna1 = np.random.rand(S) * 5 + 3
    coluna2 = np.random.rand(S) * ki_max
    coluna3 = np.random.rand(S) * kd_max
    x = np.hstack((coluna1.reshape(-1, 1),
                   coluna2.reshape(-1, 1),
                   coluna3.reshape(-1, 1)))

    # INICIALIZANDO O ENXAME
    v = np.random.uniform(-0.1, 0.1, (S, N))
    
    # INICIALIZANDO O ROBÔ
    crb01 = Corobeu(0.05)
    sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)
    
    # AVALIAÇÃO INICIAL
    y = x.copy()
    
    
    erros = [0.0] * S
    for i in range(S):
        erros[i] = fitnessFunc(crb01, x[i])
        sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)
        print(f"Partícula[{i}]: {x[i]} - Erro: {erros[i]}\n")
    
    ys_indice = np.argmin(erros)
    ys_posicao = y[ys_indice].copy()
    ys_fitness = erros[ys_indice]
    
    # PREPARANDO OS PLOTS
    historico = []
    
    # # ARPSO
    fase_repulsiva = False
    # iteracoes_repulsivas = 5
    
    # COMEÇANDO O ALGORITMO
    iteracao = 0                           
    while iteracao < iteracoes_max:
        
        # ## ARPSO ## 1. Define o gatilho para a fase de repulsão
        # # A cada 20 iterações (exceto a primeira), ativa a fase de repulsão.
        # if (iteracao % 40 == 0) and (iteracao > 0):
        #     fase_repulsiva = True
        
        if fase_repulsiva:
            print(f"--- Iteração {iteracao}: FASE DE REPULSÃO ATIVADA ---")

        # Loop de atualização de velocidade e posição
        j = 0
        while j < N:
            i = 0
            while i < S:
                r1 = np.random.rand()
                r2 = np.random.rand()

                ## ARPSO ## 2. Altera a fórmula da velocidade com base na fase
                if fase_repulsiva:
                    # Fórmula REPULSIVA: o termo social é subtraído (afasta do melhor global)
                    v[i, j] = w0 * v[i, j] + c1 * r1 * (y[i, j] - x[i, j]) - c2 * r2 * (ys_posicao[j] - x[i, j])
                else:
                    # Fórmula ATRATIVA: o PSO padrão
                    v[i, j] = w0 * v[i, j] + c1 * r1 * (y[i, j] - x[i, j]) + c2 * r2 * (ys_posicao[j] - x[i, j])

                # Limita a velocidade (clamping)
                if (j == 0):
                    v_max = v_max_kp
                elif (j == 1):
                    v_max = v_max_ki
                else:
                    v_max = v_max_kd
                
                v[i, j] = np.clip(v[i, j], -v_max, v_max)

                # Atualiza a posição
                x[i, j] = x[i, j] + v[i, j]
                
                # Lógica de contenção de limites (sem alterações)
                if (x[i, j] > kp_max and j == 0):
                    x[i, j] = kp_max - np.random.rand()*2
                if (x[i, j] > ki_max and j == 1):
                    x[i, j] = ki_max - np.random.rand()*0.5
                if (x[i, j] > kd_max and j == 2):
                    x[i, j] = kd_max - np.random.rand()*0.1
                    
                if (x[i, j] < 3 and j == 0):
                    x[i, j] = 3 + np.random.rand()
                if (x[i, j] < 0 and j == 1):
                    x[i, j] = 0 + np.random.rand()*0.5
                if (x[i, j] < 0 and j == 2):
                    x[i, j] = 0 + np.random.rand()*0.1
                
                i += 1
            j += 1
        
        # Loop de avaliação do fitness
        for c in range(S):
            particula = x[c]
            erro_particula = fitnessFunc(crb01, particula)
            sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)
            print(f"Partícula[{c}]: {particula} - Erro: {erro_particula} - Menor Erro: {ys_fitness} - Melhor partícula {ys_posicao}\n")
            
            if erro_particula < erros[c]:
                erros[c] = erro_particula 
                y[c] = particula
            
            if erro_particula < ys_fitness:
                # print(f"Novo melhor erro encontrado: {erro_particula}\n")
                ys_fitness = erro_particula
                ys_posicao = x[c].copy()

        if fase_repulsiva:
            iteracoes_repulsivas -= 1
            
            if(iteracoes_repulsivas == 0):
                fase_repulsiva = False
                iteracoes_repulsivas = 5
        
        iteracao += 1
        w0 = w0 + w_passo # Atualiza o fator de inércia
        
        sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)
        historico.append(ys_fitness)

    return historico, iteracoes_max, ys_posicao


def fitnessFunc(crb01, particula):
    erro = crb01.cacar_pid(particula)
    return erro


class Corobeu:
    def __init__(self, dt):

        self.dt = dt
        self.checkpoint_dt = 2

        self.integral_range = 30
        self.interror = [0 for _ in range(self.integral_range)]
        self.Integral_part = 0  
        self.f_ant = 0        
        
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 8
        self.phi = 0
        
        self.last_speed_time = time.time()
        self.last_checkpoint_time = time.time()
        
        # --- Conexão com o CoppeliaSim --- 
        self.clientID = 0;
        self.robot = 0;
        self.motorE = 0;
        self.motorD = 0;
        
        (self.clientID, self.robot, self.motorE, self.motorD, _) = connect_CRB(19995)
    

    def speed_control(self, U, omega):

        vr = (2 * U + omega * 7.5) / 3
        vl = (2 * U - omega * 7.5) / 3
        
        # Encontra o fator de escala necessário
        max_speed = max(abs(vr), abs(vl))
        if max_speed > self.v_max:
            scale_factor = self.v_max / max_speed
            vr *= scale_factor
            vl *= scale_factor
        
        if math.isnan(vr) or math.isnan(vl):
            vr, vl = 0, 0
        
        return int(vl), int(vr)


    def cacar_pid(self, PID):
        
        self.interror = [0 for _ in range(self.integral_range)]
        integral_counter = 0
        kp, ki, kd = PID[0], PID[1], PID[2]
        path = [[0.4, 0.4],
                [-0.4, 0.4],
                [-0.4, -0.4],
                [0.4, -0.4]]

        # LÓGICA 1: Inicializa o erro total para todo o percurso
        total_error = 0

        if (sim.simxGetConnectionId(self.clientID) != -1):

            sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)

            # LÓGICA 2: Loop externo para percorrer cada ponto no 'path'
            for target_point in path:
                # Define o alvo ATUAL para o loop de movimento
                path_x, path_y = target_point[0], target_point[1]

                # Reseta o acumulador de erro e o tempo para CADA novo alvo
                error_phi_sum = 0
                start_time = time.time()

                # Este loop 'while' agora é responsável por ir para UM único alvo
                while True:
                    
                    current_time = time.time()    
                    if current_time - self.last_speed_time < self.dt:
                        continue      
                                 
                    s, robotPosition = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    s, phi = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_blocking)

                    # Validação para evitar erros se a simulação não retornar posição
                    if not robotPosition:
                        continue 

                    x = robotPosition[0]
                    y = robotPosition[1]
                    phi_obs = phi[2] - math.pi/2

                    phid = math.atan2((path_y - y), (path_x - x))
                    phid = self.wrap_angle(phid)

                    phi_obs = self.wrap_angle(phi_obs)
                    error_phi = self.wrap_angle(phid - phi_obs)

                    error_phi_sum += abs(error_phi)

                    omega = self.pid_controller(kp, ki, kd, error_phi, integral_counter)

                    error_distance = math.sqrt((path_y - y)**2 + (path_x - x)**2)

                    U = self.v_linear  
                    
                    vl, vr = self.speed_control(U, omega)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, vl, sim.simx_opmode_blocking)    
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, vr, sim.simx_opmode_blocking)
                    self.last_speed_time = current_time

                    
                    integral_counter += 1
                    if integral_counter >= self.integral_range:
                        integral_counter = 0
                    
                    # LÓGICA 3: Se chegou ao alvo, quebra o loop 'while' para ir para o próximo ponto
                    if (error_distance < 0.07):
                        break
                    
                    # LÓGICA 4: Se o tempo esgotou, também quebra o loop
                    if current_time - start_time >= 5:
                        # Opcional: Penalizar o erro total se o tempo esgotou
                        error_phi_sum += 1000 # Adiciona uma grande penalidade
                        break
                    
                # LÓGICA 5: Acumula o erro do trecho recém-concluído no erro total
                total_error += error_phi_sum

        # LÓGICA 6: Retorna o erro acumulado de todo o percurso
        return total_error
            

    def pid_controller(self, kp, ki, kd, error, integral_counter):
        
        Integral_saturation = 5
        raizes = math.sqrt(kd), math.sqrt(kp), math.sqrt(ki)
        Filter_e = 1 / (max(raizes) * 10)   
        unomenosalfaana = math.exp(-(self.dt / Filter_e))
        alfaana = 1 - unomenosalfaana
        self.interror[integral_counter] = error
        f = unomenosalfaana * self.f_ant + alfaana * error
        deerror = (f - self.f_ant) / self.dt if self.f_ant != 0 else f / self.dt
        self.Integral_part = min(max(self.Integral_part + ki * sum(self.interror) * self.dt, -Integral_saturation), Integral_saturation)
        self.f_ant = f
        PID = kp * error + self.Integral_part + deerror * kd
        return PID
    
    
    def wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi
    
    
    def off(self, signum=None, frame=None ):
        
        sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)    
        sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)
        return
    

if __name__ == "__main__":

    algoritmo_pso(5, 3, 1000)