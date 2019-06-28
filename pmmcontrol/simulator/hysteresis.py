import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Hysteresis():
    def __init__(self, N=10, alpha=1, beta=1):
        self.x = int(0)
        self.y = int(0)
        self.relay = np.zeros((N,N))
        self.weights = np.ones((N,N))

        self.size = N

        self.xValues = np.array([])
        self.yValues = np.array([])

    def moveT(self, x):
        if x > self.x:
            for row in range(self.size-int(x), self.size):
                self.relay[row].fill(1)
        if x < self.x:
            for col in range(int(x), self.x):
                self.relay[:,col].fill(0)

        self.x = int(x)

        #self.__printHalf(self.relay)
        #self.__printHalf(self.weights)
        y = self.__sumHalf(self.relay, self.weights)
        self.y = y

        print("Y :", y)

        self.xValues = np.append(self.xValues, self.x)
        self.yValues = np.append(self.yValues, self.y)
        self.__plot()

        return True

    def move(self, x):
        if (not isinstance(x, int)):
            raise ValueError("Parameter must be integer")

        if x > self.x:
            for i in range(self.x, x):
                self.increment('up')
        if x < self.x:
            for i in range(x, self.x):
                self.increment('down')

        self.__printHalf(self.relay)
        self.__printHalf(self.weights)

        return True

    def increment(self, direction):
        if direction == 'up':
            self.relay[self.size - self.x - 1].fill(1)
            self.x = self.x + 1
        if direction == 'down':
            self.relay[:, self.x - 1].fill(-1)
            self.x = self.x - 1

        self.y = self.__sumHalf(self.relay, self.weights)

        print("Y :", self.y)

        self.xValues = np.append(self.xValues, self.x)
        self.yValues = np.append(self.yValues, self.y)
        self.__plot()

    def __plot(self):

        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)

        ax.plot(self.xValues, self.yValues)

        plt.show()

    def __sumHalf(self, mat1, mat2):
        if not mat1.shape == mat2.shape:
            raise ValueError("Matrix 1 and 2 must be the same shape")
        if not mat1.shape[0] == mat2.shape[1]:
            raise ValueError("Matrices must be square")

        size = mat1.shape[0]
        print("Size: ", size)

        rowSum = np.zeros(size)
        for row in range(0, size):
            row1 = mat1[row][:(size-row)]
            row2 = mat2[row][:(size-row)]
            rowSum[row] = np.dot(row1, row2)

        print(rowSum)

        return np.sum(rowSum)

    def __printHalf(self, mat):
        size = mat.shape[0]

        for row in range(0, size):
            print(mat[row][:(size-row)])

        return True