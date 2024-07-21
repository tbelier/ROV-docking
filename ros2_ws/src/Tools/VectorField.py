import numpy as np
import matplotlib.pyplot as plt
from roblib import *

ymin, ymax, xmin, xmax, Ny, Nx = -10, 10, 0, 20, 20, 20
Rmax = 15
eps = 0.2
def computeV_U(Y,X):
    V = -Y
    U = -X
    
    
    for j in range(Ny):
        for i in range(Nx):
            y = Y[i,j]
            x = X[i,j]

            if np.sqrt(x**2+y**2) > Rmax :
                V[i,j] = -np.sign(y)
                U[i,j] = -np.sign(x)

            elif x != 0 and abs(np.arctan2(y,x) * 180 / np.pi) < 50:
                U[i,j] = -1
            else:
                V[i,j] = -np.sign(y)
                U[i,j] = 1

    return V, U

def compute_dy_dx(y,x):
    if np.sqrt(x**2+y**2) > Rmax :
        return -np.sign(y),-np.sign(x)
    elif x != 0 and -30 < np.arctan2(y,x) * 180 / np.pi < 30:
        return -np.sign(y), -1
    else:
        return -np.sign(y), 1

# Créer une grille 2D
y = np.linspace(ymin,ymax,Ny)
x = np.linspace(xmin,xmax,Nx)
Y, X = np.meshgrid(y,x)
y,x = 9,2
y1,x1 = 10,15
dt = 0.1
test_far = True
while  test_far :
    if (np.sqrt(x**2+y**2) < eps) and (np.sqrt(x1**2+y1**2) < eps) : test_far = False
    V,U = computeV_U(Y,X)
    dy, dx = compute_dy_dx(y, x)
    dy1, dx1 = compute_dy_dx(y1,x1)
    y, x = y + dt * dy, x + dt * dx
    y1, x1 = y1 + dt * dy1, x1 + dt * dx1

    # DRAW
    plt.cla()
    plt.quiver(Y,X,V,U, scale=50, color='gray', width=0.002)
    plt.xlabel('Y-axis')
    plt.ylabel('X-axis')
    plt.title('Champ de potentiel 2D')
    draw_tank([y,x,0], w=0.5, r=0.25,col='red')
    draw_tank([y1,x1,0], w=0.5, r=0.25, col='green')
    plt.scatter(y, x)
    plt.scatter(y1, x1)
    plt.pause(dt)

plt.show()
