import numpy as np
import matplotlib.pyplot as plt

class Hysteresis():
    def __init__(self, N=10):

        if not (N%2 == 0):
            raise ValueError("Matrix size must be even")

        #starting point
        self.x = int(0)
        self.y = int(0)

        #matrix of relays that can be flipped up (+1) or down (-1)
        self.relay = np.zeros((N,N))

        #matrix of weights to be applied to each relay
        self.weights = np.ones((N,N))

        #store size of matrix
        self.size = N

        #list of data points to be appended to
        self.xValues = np.append(np.array([]), self.x)
        self.yValues = np.append(np.array([]), self.y)

        #fill the relay matrix in starting position
        self.__relayFill()

        #fill the weights matrix to adjust behavior of hysteresis
        self.__weightFill()

    def move(self, x):
        '''
        Change x-coordinate of hysteresis function
        '''
        if (not isinstance(x, int)):
            raise ValueError("Parameter must be integer")

        if x > self.x:
            for i in range(self.x, x):
                self.increment('up')
        if x < self.x:
            for i in range(x, self.x):
                self.increment('down')

        return True

    def increment(self, direction):
        '''
        Change x-coordinate of hysteresis plot by +/- 1 depending on if direction is set as 'up' or 'down'
        '''

        if not (direction == 'up' or direction == 'down'):
            raise ValueError("\"direction\" must be set to either \"up\" or \"down\"")

        if direction == 'up':

            #only adjust matrix if x-coord is within +/- N/2
            #otherwise let x-coord increase without change of y-coord. This represents saturation
            if self.x >= -self.size/2 and self.x < self.size/2:

                #fill one row (at current coordinate) of relay matrix with "up" relays
                self.relay[self.size - (self.x + self.size//2)- 1].fill(1)

            self.x = self.x + 1

        if direction == 'down':
            # only adjust matrix if x-coord is within +/- N/2
            # otherwise let x-coord increase without change of y-coord. This represents saturation
            if self.x > -self.size/2 and self.x <= self.size/2:

                #fill one column of relay matrix with "down" relays
                self.relay[:, (self.x + self.size//2) - 1].fill(-1)

            self.x = self.x - 1

        #update y-coord using a weighted sum of relay matrix
        self.y = self.__sumHalf(self.relay, self.weights)

        #update coordinate lists
        self.xValues = np.append(self.xValues, self.x)
        self.yValues = np.append(self.yValues, self.y)

        #FOR TESTING PURPOSES
        #self.__plot()

        return True

    def __plot(self):
        '''
        Can be used to visually inspect relay weight distribution
        '''

        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)

        ax.plot(self.xValues, self.yValues)

        plt.show()

        return True

    def __weightFill(self):
        '''
        Function for filling weight matrix
        '''

        #currently distributed as a gradient that decreases away from the center diagonal
        for n in range(0, self.size):
            fill = 1-(1/(self.size - 1) * n)
            fillArr = np.ones(self.size - n) * fill
            np.fill_diagonal(self.weights[n:], fillArr)
            np.fill_diagonal(self.weights[:,n:], fillArr)

        return True

    def __relayFill(self):
        '''
        Optional function to set initial state of all relays
        '''

        for n in range(self.size//2, self.size):
            self.relay[n].fill(1)
            self.relay[:, n].fill(-1)

        return True


    def __sumHalf(self, mat1, mat2):
        '''
        Returns a scalar product of the upper-left triangle between two matrices
        '''

        if not mat1.shape == mat2.shape:
            raise ValueError("Matrix 1 and 2 must be the same shape")

        if not mat1.shape[0] == mat2.shape[1]:
            raise ValueError("Matrices must be square")

        size = mat1.shape[0]

        rowSum = np.zeros(size)

        #sum up scalar product of each row
        for row in range(0, size):
            row1 = mat1[row][:(size-row)]
            row2 = mat2[row][:(size-row)]
            rowSum[row] = np.dot(row1, row2)

        return np.sum(rowSum)

    def __printHalf(self, mat):
        '''
        Prints the upper-left half of a matrix
        '''

        size = mat.shape[0]

        for row in range(0, size):
            print(mat[row][:(size-row)])

        return True