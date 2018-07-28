import numpy as np

class Connect_array():
    def __init__(self):
        self.a = np.array([[1, 3, 5], [2, 4, 6]])
        self.b = np.array([[7, 9, 11], [8, 10, 12]])

    def Hstack(self):
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.hstack((self.a, self.b))
        print('hstack(a,b) = {0}'.format(connected_array))

    def Vstack(self):
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.vstack((self.a, self.b))
        print('vstack(a,b) = {0}'.format(connected_array))

    def Stack(self):
        print('axis number ?')
        axis_num = int(input())
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.stack((self.a, self.b), axis=axis_num)
        print('stack(a,b) = {0} (axis={1})'.format(connected_array, axis_num))

    def Concatenate(self):
        print('axis number ?')
        axis_num = int(input())
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.concatenate((self.a, self.b), axis=axis_num)
        print('Concatenate(a,b) = {0} (axis={1})'.format(connected_array, axis_num))


def main():
    connect_array = Connect_array()

    connect_array.Hstack()
    
    connect_array.Vstack()

    connect_array.Stack()

    connect_array.Concatenate()


if __name__ == '__main__':
    main()