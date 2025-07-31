import torch
import numpy as np
import math


def test0():
    # Create ground truth data
    x = np.linspace(-math.pi, math.pi, 2000)
    a,b,c,d = 1.0,2.0,3.0,4.0
    a0,b0,c0,d0 = a,b,c,d
    y = a + b * x + c * x ** 2 + d * x ** 3

    # add some random error
    s = np.random.normal(0, 0.3, 4)
    a += s[0]
    b += s[1]
    c += s[2]
    d += s[3]

    # Main Solver Loop
    learning_rate = 1.0e-6
    for t in range(2000):
        # Forward pass: compute predicted y
        # y = a + b x + c x^2 + d x^3
        y_pred = a + b * x + c * x ** 2 + d * x ** 3

        # Compute and print loss
        loss = np.square(y_pred - y).sum()
        if t % 100 == 99:
            print(t, loss)

        # Compute Gadient (partial derivative of cost with respect to a, b, c, d)
        grad_y_pred = 2.0 * (y_pred - y)
        grad_a = grad_y_pred.sum()
        grad_b = (grad_y_pred * x).sum()
        grad_c = (grad_y_pred * x ** 2).sum()
        grad_d = (grad_y_pred * x ** 3).sum()

        # Update weights
        a -= learning_rate * grad_a
        b -= learning_rate * grad_b
        c -= learning_rate * grad_c
        d -= learning_rate * grad_d

    print(f'Actual: y = {a0} + {b0} x + {c0} x^2 + {d0} x^3')
    print(f'Result: y = {a} + {b} x + {c} x^2 + {d} x^3')

if __name__ == "__main__":
    #pytorch_hello_world()
    test0()