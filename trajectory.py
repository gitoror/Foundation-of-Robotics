import numpy as np
import matplotlib.pyplot as plt

def generate_polynomial_trajectory(p1, p2, grade=6, n_points=100):
    x = np.linspace(p1[0], p2[0], n_points)
    y = (x - p1[0]) ** grade * (p2[1] - p1[1]) / ((p2[0] - p1[0]) ** grade) + p1[1]
    return np.array(list(zip(x, y)))

def generate_radical_trajectory(p1, p2, grade=5, n_points=100):
    x = np.linspace(p1[0], p2[0], n_points)
    y = (x - p1[0]) ** (1/grade) * (p2[1] - p1[1]) / ((p2[0] - p1[0]) ** (1 / grade)) + p1[1]
    return np.array(list(zip(x, y)), dtype=np.float32)

def generate_linear_trajectory(p1, p2, n_points=100):
    x = np.linspace(p1[0], p2[0], n_points)
    y = (x - p1[0]) * (p2[1] - p1[1]) / (p2[0] - p1[0]) + p1[1]
    return np.array(list(zip(x, y)), dtype=np.float32)

def plot_trajectory(trajectory, label="trajectory"):
    # for point in trajectory:
    #     print(np.format_float_positional(point[0]), np.format_float_positional(point[1]))
    plt.plot(trajectory[:,0], trajectory[:,1], label=label)
# Example usage:
if __name__ == "__main__":
    p1 = (2, 0)
    p2 = (1, 1)
    trajectory = generate_polynomial_trajectory(p1, p2,n_points=100)
    for point in trajectory:
        print(np.format_float_positional(point[0]), np.format_float_positional(point[1]))
    plt.plot(trajectory[:,0], trajectory[:,1])
    plt.show()