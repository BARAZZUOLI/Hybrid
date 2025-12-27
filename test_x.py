import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


def f(x, t):
    return np.sin(t)

t = np.linspace(0, 50, 256)
x = odeint(f, 2, t)
plt.plot(t, x)
plt.title("solution de x'=sin(t)sin(x)")
plt.show()