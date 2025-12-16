from controller import Robot

# Script para ler os ângulos atuais do robô (Teach Mode)
def ler_angulos():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Nomes corretos que descobrimos
    nomes_sensores = ['A sensor', 'B sensor', 'C sensor', 'D sensor', 'E sensor', 'F sensor']
    sensores = []
    
    for nome in nomes_sensores:
        s = robot.getDevice(nome)
        s.enable(timestep)
        sensores.append(s)
        
    print("--- MODO LEITURA ATIVADO ---")
    print("Mova o robô manualmente no Webots até tocar na bola.")
    print("Os valores aparecerão aqui. Copie a lista final para o seu código.")

    while robot.step(timestep) != -1:
        # Lê os valores atuais
        valores = [round(s.getValue(), 4) for s in sensores]
        
        # Imprime formatado para Python (fácil de copiar)
        print(f"q_target = np.array({valores})")

if __name__ == "__main__":
    ler_angulos()