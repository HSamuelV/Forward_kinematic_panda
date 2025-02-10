import numpy as np

def franka_fk(q_degrees):
    # converte o angulo das juntas para radianos
    q = np.deg2rad(q_degrees)
    
    # Matriz DH para o Franka Research 3
    dh_params = [
        {'alpha': 0,        'a': 0,       'd': 0.333, 'theta_offset': 0},     # junta 1
        {'alpha': -np.pi/2, 'a': 0,       'd': 0,     'theta_offset': 0},     # junta 2
        {'alpha': np.pi/2,  'a': 0,       'd': 0.316, 'theta_offset': 0},     # junta 3
        {'alpha': np.pi/2,  'a': 0.0825,  'd': 0,     'theta_offset': 0},     # junta 4
        {'alpha': -np.pi/2, 'a': -0.0825, 'd': 0.384, 'theta_offset': 0},     # junta 5
        {'alpha': np.pi/2,  'a': 0,       'd': 0,     'theta_offset': 0},     # junta 6
        {'alpha': np.pi/2,  'a': 0.088,   'd': 0.107, 'theta_offset': 0},     # junta 7
    ]
    
    T = np.eye(4)
    
    for i in range(7):
        alpha = dh_params[i]['alpha']
        a = dh_params[i]['a']
        d = dh_params[i]['d']
        theta_offset = dh_params[i]['theta_offset']
        
        # angulo da junta
        theta = q[i] + theta_offset
        
        # Calcula a matriz de transformação de junta
        Ti = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
            [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
            [0, 0, 0, 1]
        ])
        
        # faz a matriz junta pra base
        T = T @ Ti
    
    # pegaa rotação de cada junta
    posicao = T[:3, 3]
    rotacao = T[:3, :3]
    
    return posicao, rotacao

# Exemplo
joint_angles = [0, 0, 0, 0, 0, 0, 0]  # posição em home
posicao, rotacao = franka_fk(joint_angles)
print("Posição do punho (meters):", posicao)
print("Matriz de rotação:\n", rotacao)
