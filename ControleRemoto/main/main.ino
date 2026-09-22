#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <cmath>

#include "behavior.h"
#include "led_rgb.h"
#include "motor.h"


Motor motor1(MOTOR1_PIN_R, MOTOR1_PIN_L, PWM_MOTOR1_CHANNEL_R, PWM_MOTOR1_CHANNEL_L);
Motor motor2(MOTOR2_PIN_R, MOTOR2_PIN_L, PWM_MOTOR2_CHANNEL_R, PWM_MOTOR2_CHANNEL_L);


float v[2];
float u[2];
float mod = 0.1;
float vel_dir = 0.0;
float vel_esq = 0.0;
float vel_lin = 205;
float vel_ang = 0.0;
float vel_ang_max = 100;
float erro_motor_value = 0;
float erro_motor = 0;

// --- FUNÇÃO AUXILIAR PARA CONTROLAR OS MOTORES ---
void setMotorSpeed(Motor& motor, int speed, int motor_id) {
    // =================================================================
    // PONTO DE AJUSTE 1: DIREÇÃO DOS MOTORES
    // Se o robô anda para trás quando você comanda para frente,
    // inverta o sinal desta variável para o motor correspondente.
    // Exemplo: se o motor 1 está invertido, mude (motor_id == 1) ? -1 : 1; para (motor_id == 1) ? 1 : 1;
    // =================================================================
    int dir_frente = (motor_id == 1) ? -1 : 1;

    if (speed > 0) {
        motor.moveForward(speed, dir_frente);
    } else if (speed < 0) {
        motor.moveForward(abs(speed), -dir_frente);
    } else {
        motor.stop();
    }
}

// --- LÓGICA DE CONTROLE PRINCIPAL ---
// --- CÓDIGO COMPLETO COM FATOR DE GIRO AJUSTÁVEL ---

// ... (includes e declarações dos motores permanecem os mesmos) ...
// ... (a função setMotorSpeed permanece a mesma) ...

// void notify() {
//     // --- Etapa 1 e 2: Leitura e Zona Morta ---
//     int joyY = Ps3.data.analog.stick.ly;
//     int joyX = Ps3.data.analog.stick.lx;
//     int deadZone = 15;
//     int processedJoyY = (abs(joyY) < deadZone) ? 0 : joyY;
//     int processedJoyX = (abs(joyX) < deadZone) ? 0 : joyX;

// // =================================================================
//     // PASSO 3 e 4 AVANÇADOS: Mixagem com Curva de Sensibilidade
//     // =================================================================
//     // Mapeamento linear para FRENTE/TRÁS
//     int moveSpeed = map(processedJoyY, -128, 127, 255, -255);

//     // Mapeamento NÃO-LINEAR para ROTAÇÃO
//     // 1. Normalizamos o valor do joystick para um intervalo de -1.0 a 1.0
//     float normalizedX = processedJoyX / 127.0;
    
//     // 2. Aplicamos uma função de potência (cúbica é ótima).
//     //    Isso cria a curva: movimentos pequenos têm pouco efeito, movimentos grandes têm muito efeito.
//     float curveX = pow(normalizedX, 5);
    
//     // 3. Mapeamos o resultado da curva de volta para a velocidade de rotação.
//     //    Podemos também definir uma velocidade máxima de giro aqui (ex: 200 em vez de 255).
//     int maxTurnSpeed = 100;
//     int turnSpeed = curveX * maxTurnSpeed;

//     // A mixagem continua a mesma
//     int speedMotor1 = moveSpeed - turnSpeed;
//     int speedMotor2 = moveSpeed + turnSpeed;
//     // =================================================================

//     // --- Etapa 5: Limitar os valores ---
//     speedMotor1 = constrain(speedMotor1, -255, 255);
//     speedMotor2 = constrain(speedMotor2, -255, 255);

//     // --- Etapa 6: Enviar comandos ---
//     setMotorSpeed(motor1, speedMotor1, 1);
//     setMotorSpeed(motor2, speedMotor2, 2);

//     // --- LOG DETALHADO PARA O MONITOR SERIAL ---
//     Serial.print("Move: "); Serial.print(moveSpeed);
//     Serial.print("\t Turn: "); Serial.print(turnSpeed);
//     Serial.print("  ||  ");
//     Serial.print("==> M1: "); Serial.print(speedMotor1);
//     Serial.print("\t M2: "); Serial.println(speedMotor2);
// }


void onConnect() {
    Serial.println("Conectado.");
}

void setup() {
    Serial.begin(115200); // Aumentei a velocidade para um log mais rápido
    Dabble.begin("Robô-Coelho-Orelhudo");
}

void loop() {
    Dabble.processInput();
    Serial.print("KeyPressed: ");
    // if (GamePad.isUpPressed())
    // {
    //   Serial.print("Up");
    // }

    // if (GamePad.isDownPressed())
    // {
    //   Serial.print("Down");
    // }

    // if (GamePad.isLeftPressed())
    // {
    //   Serial.print("Left");
    // }

    // if (GamePad.isRightPressed())
    // {
    //   Serial.print("Right");
    // }

    // if (GamePad.isSquarePressed())
    // {
    //   Serial.print("Square");
    // }

    // if (GamePad.isCirclePressed())
    // {
    //   Serial.print("Circle");
    // }

    // if (GamePad.isCrossPressed())
    // {
    //   Serial.print("Cross");
    // }

    // if (GamePad.isTrianglePressed())
    // {
    //   Serial.print("Triangle");
    // }

    // if (GamePad.isStartPressed())
    // {
    //   Serial.print("Start");
    // }

    // if (GamePad.isSelectPressed())
    // {
    //   Serial.print("Select");
    // }
    // Serial.print('\t');

    // int a = GamePad.getAngle();
    // Serial.print("Angle: ");
    // Serial.print(a);
    // Serial.print('\t');
    // int b = GamePad.getRadius();
    // Serial.print("Radius: ");
    // Serial.print(b);
    // Serial.print('\t');
    float x = GamePad.getXaxisData();
    // Serial.print("x_axis: ");
    // Serial.print(x);
    // Serial.print('\t');
    float y = GamePad.getYaxisData();
    // Serial.print("y_axis: ");
    // Serial.println(y);
    // Serial.println();

    v[0] = x;
    v[1] = y;

    v[0] = v[0] / 2.34;

    mod = sqrt(pow(v[0], 2) + pow(v[1], 2));
    
    if (mod == 0){
      mod = pow(10, 10);
      erro_motor = 0;
    }

    for (int i = 0; i < 2; i++){
      u[i] = pow(v[i] / mod, 2);
    }

    if (x < 0){
      erro_motor = -erro_motor_value;
    }
    
    if(x > 0){
      u[0] = -u[0];
      erro_motor = erro_motor_value;
    }

    if (y < 0){
      u[1] = -u[1];
    }

    // Serial.print("x_axis_norm: ");
    // Serial.print(u[0]);
    // Serial.print('\t');

    // Serial.print("y_axis_norm: ");
    // Serial.println(u[1]);
    // Serial.println();

    vel_ang = vel_lin * u[0];

    if (vel_ang > vel_ang_max){
      vel_ang = vel_ang_max;
    }
    if (vel_ang < -vel_ang_max){
      vel_ang = -vel_ang_max;
    }

    if (x != 0){
      erro_motor = erro_motor_value;
    }
    
    if (y != 0){
      erro_motor = erro_motor_value;
    }

    vel_dir = vel_lin * u[1] - vel_ang;
    vel_esq = vel_lin * u[1] + vel_ang + erro_motor; 

    // Serial.print("vel_dir: ");
    // Serial.print(vel_dir);
    // Serial.print('\t');

    // Serial.print("vel_esq ");
    // Serial.println(vel_esq);
    // Serial.println();

    setMotorSpeed(motor1, -vel_dir, 1);
    setMotorSpeed(motor2, -vel_esq, 2);
}   