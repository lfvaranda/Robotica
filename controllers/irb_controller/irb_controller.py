from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
from trajectory_lib import TrajectoryGenerator

# 'CUBICA', 'QUINTICA' ou 'TRAPEZOIDAL'
TIPO_TRAJETORIA = 'TRAPEZOIDAL' 

TEMPO_MOVIMENTO = 4.0
TIME_STEP = 32

def run_robot():
    robot = Robot()
    traj_gen = TrajectoryGenerator()
    
    
    nomes_motores = ['A motor', 'B motor', 'C motor', 'D motor', 'E motor', 'F motor']
    motores = []
    
    for nome in nomes_motores:
        m = robot.getDevice(nome)
        if m:
            m.setPosition(float('inf')) 
            m.setVelocity(1.5) 
            motores.append(m)
        else:
            print(f"ERRO: Não achei o motor {nome}")
            return

    
    q_home = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    q_target = np.array([-1.0, 0.2, 0.5, 0.2, 0.1, 0.2]) 

    
    for i, m in enumerate(motores):
        m.setPosition(q_home[i])
    
    
    robot.step(1000)
    
    
    hist_time = []
    hist_pos = [] 
    
    print(f"Iniciando trajetória {TIPO_TRAJETORIA} para o alvo: {q_target}")
    start_time = robot.getTime()
    
   
    while robot.step(TIME_STEP) != -1:
        now = robot.getTime()
        t = now - start_time
        
      
        if t > TEMPO_MOVIMENTO:
            break
            
       
        if TIPO_TRAJETORIA == 'CUBICA':
            q_next = traj_gen.cubic_trajectory(t, 0, TEMPO_MOVIMENTO, q_home, q_target)
        elif TIPO_TRAJETORIA == 'QUINTICA':
            q_next = traj_gen.quintic_trajectory(t, 0, TEMPO_MOVIMENTO, q_home, q_target)
        elif TIPO_TRAJETORIA == 'TRAPEZOIDAL':
            q_next = traj_gen.trapezoidal_trajectory(t, 0, TEMPO_MOVIMENTO, q_home, q_target)
        
        
        for i, motor in enumerate(motores):
            motor.setPosition(q_next[i])
            
        
        hist_time.append(t)
        hist_pos.append(q_next.copy())

    print("Movimento concluído. Gerando gráficos de desempenho...")
    plotar_graficos(hist_time, hist_pos, TIME_STEP)

def plotar_graficos(time_data, pos_data, time_step_ms):
    
    t = np.array(time_data)
    q = np.array(pos_data) 
    
    
    dt = time_step_ms / 1000.0 
    
    
    vel = np.gradient(q, dt, axis=0)
    acc = np.gradient(vel, dt, axis=0)

    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    colors = ['r', 'g', 'b', 'c', 'm', 'k']
    labels = ['Eixo 1 (Base)', 'Eixo 2', 'Eixo 3', 'Eixo 4', 'Eixo 5', 'Eixo 6']

    
    for i in range(6):
        ax1.plot(t, q[:, i], label=labels[i], color=colors[i])
    ax1.set_ylabel('Posição (rad)')
    ax1.set_title(f'Análise da Trajetória: {TIPO_TRAJETORIA}')
    ax1.grid(True)
    ax1.legend(loc='upper left', fontsize='small', bbox_to_anchor=(1, 1))

    
    for i in range(6):
        ax2.plot(t, vel[:, i], label=labels[i], color=colors[i])
    ax2.set_ylabel('Velocidade (rad/s)')
    ax2.grid(True)

    
    for i in range(6):
        ax3.plot(t, acc[:, i], label=labels[i], color=colors[i])
    ax3.set_ylabel('Aceleração (rad/s²)')
    ax3.set_xlabel('Tempo (s)')
    ax3.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_robot()