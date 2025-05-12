import numpy as np

def get_puzzlebot_kinematic_model(r, l):
    return np.array([[r/2, r/2],
                   [r/l, -r/l]])

def get_inverse_puzzlebot_kinematic_model(r, l):
    return np.linalg.inv(get_puzzlebot_kinematic_model(r, l))

def get_linearized_puzzlebot_model_matrix(v, theta, delta):
    return np.array([[1, 0, -v*np.sin(theta)*delta],
                     [0, 1,  v*np.cos(theta)*delta],
                     [0, 0,           1           ]])

speeds_decomposer = lambda v, w, theta: np.array([v * np.cos(theta), v * np.sin(theta), w])