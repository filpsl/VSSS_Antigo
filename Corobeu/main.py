import subprocess
import sys
import signal
import time

# --- Configuração ---
# 1. Nome do seu script de ataque
script_robo_ataque = "Corobeu/Corobeu_ATK.py"

# 2. Nome da "variante" (seu segundo robô)
#    (Você precisará criar este arquivo, veja as instruções abaixo)
script_robo_goleiro = "Corobeu/Corobeu_ATK2.py" 
# --------------------


# Lista para rastrear nossos processos filhos
child_processes = []

def signal_handler(sig, frame):
    """
    Este é o 'coração' da solução.
    Quando ESTE script (o mestre) recebe Ctrl+C, esta função é chamada.
    """
    print("\n[Mestre] Ctrl+C recebido! Enviando sinal de desligamento (SIGINT) para os robôs...")
    
    for p in child_processes:
        # Envia o sinal SIGINT (o mesmo que Ctrl+C) para o processo filho
        p.send_signal(signal.SIGINT)

    print("[Mestre] Aguardando robôs desligarem graciosamente...")
    
    # Espera que os processos filhos realmente terminem
    for p in child_processes:
        p.wait()

    print("[Mestre] Todos os robôs foram desligados. Saindo.")
    sys.exit(0)

def main():
    # Registra nosso manipulador de sinal personalizado
    signal.signal(signal.SIGINT, signal_handler)

    print(f"[Mestre] Lançando o robô de ATAQUE ({script_robo_ataque})...")
    
    # Inicia o primeiro robô
    # Usamos sys.executable para garantir que estamos usando o mesmo Python
    p_ataque = subprocess.Popen([sys.executable, script_robo_ataque])
    child_processes.append(p_ataque)

    print(f"[Mestre] Lançando o robô GOLEIRO ({script_robo_goleiro})...")
    
    # Inicia o segundo robô
    p_goleiro = subprocess.Popen([sys.executable, script_robo_goleiro])
    child_processes.append(p_goleiro)

    print("\n--- Robôs em execução ---")
    print(f"  Atacante (PID): {p_ataque.pid}")
    print(f"  Goleiro (PID):  {p_goleiro.pid}")
    print("\nPressione Ctrl+C nesta janela para desligar todos os robôs.")

    # Loop infinito para manter o script mestre vivo
    # Ele só ficará aqui, esperando pelo Ctrl+C
    while True:
        try:
            # Verifica se algum processo morreu inesperadamente
            for p in child_processes:
                if p.poll() is not None:
                    print(f"\n[Mestre] ATENÇÃO: O processo {p.pid} terminou inesperadamente.")
                    # Se um cair, derruba todos (opcional, mas mais seguro)
                    signal_handler(None, None)
            
            time.sleep(1) # Dorme para não consumir CPU
        
        except Exception:
            # Isso é para capturar outros erros, o SIGINT é tratado pelo handler
            pass

if __name__ == "__main__":
    main()