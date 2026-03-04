""" 
ESSE NOVO "config.py" BUSCA ENTRAR 
EM ACORDO COM A NOVA  INTERFACE A 
SER CRIADO PELO PESSOAL DE VISÃO 
COMPUTACIONAL.
"""

"CONFIGURAÇÕES GERAIS"
COR_DO_TIME = 1                             # 0 -> Amarelo; 1 -> Azul
LADO_DO_TIME = 1                            # 0 -> Esquerdo; 1 -> Direito
GOL_ADVERSARIO = None

if LADO_DO_TIME == 0:
    GOL_ADVERSARIO = 0.75
elif LADO_DO_TIME == 1:
    GOL_ADVERSARIO = -0.75
else:
    raise ValueError("LADO_DO_TIME deve ser 0 (Esquerdo) ou 1 (Direito)")

"CONFIGURAÇÕES ESPECÍFICAS" 
IP_ZEUS = "192.168.209.59"                  # Mesma vel
# IP_KRATOS = "192.168.0.103"                # + 28 VL
IP_KRATOS = "192.168.188.7"                # + 28 VL
IP_ARES = "192.168.188.30"                # + 28 VL

ID_ZEUS = 3 
ID_KRATOS = 7  
ID_ARES = 4 

FUNCAO_ZEUS = 0                             #0 -> Goleiro; 1 -> Meio-Campista; 2 -> Atacante
FUNCAO_KRATOS = 1                           #0 -> Goleiro; 1 -> Meio-Campista; 2 -> Atacante
FUNCAO_ARES = 2                             #0 -> Goleiro; 1 -> Meio-Campista; 2 -> Atacante 