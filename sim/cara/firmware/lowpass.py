"""Port of the LowPass<order> filter in Functions.ino (Curio Res). Uses HAL micros()."""
from math import sqrt


class LowPass:
    def __init__(self, order, f0, fs, adaptive, micros):
        assert order in (1, 2)
        self.order = order
        self._micros = micros
        self.omega0 = 6.28318530718 * f0
        self.dt = 1.0 / fs
        self.adapt = adaptive
        self.tn1 = -self.dt
        self.x = [0.0] * (order + 1)
        self.y = [0.0] * (order + 1)
        self.a = [0.0] * order
        self.b = [0.0] * (order + 1)
        self.setCoef()

    def setCoef(self):
        if self.adapt:
            t = self._micros() / 1.0e6
            self.dt = t - self.tn1
            self.tn1 = t
        alpha = self.omega0 * self.dt
        if self.order == 1:
            self.a[0] = -(alpha - 2.0) / (alpha + 2.0)
            self.b[0] = alpha / (alpha + 2.0)
            self.b[1] = alpha / (alpha + 2.0)
        if self.order == 2:
            alphaSq = alpha * alpha
            beta = [1, sqrt(2), 1]
            D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2]
            self.b[0] = alphaSq / D
            self.b[1] = 2 * self.b[0]
            self.b[2] = self.b[0]
            self.a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D
            self.a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D

    def filt(self, xn):
        if self.adapt:
            self.setCoef()
        self.y[0] = 0
        self.x[0] = xn
        for k in range(self.order):
            self.y[0] += self.a[k] * self.y[k + 1] + self.b[k] * self.x[k]
        self.y[0] += self.b[self.order] * self.x[self.order]
        for k in range(self.order, 0, -1):
            self.y[k] = self.y[k - 1]
            self.x[k] = self.x[k - 1]
        return self.y[0]
