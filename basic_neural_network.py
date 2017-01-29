import sys
import numpy as np

X = np.array([[0,0,1],
             [0,1,1],
             [1,0,1],
             [1,1,1]])
y = np.array([[0,1,0,1]]).T
def sigmoid(x):
    return 1/(1+np.exp(-x))

def slope(x):
    #f = sigmoid(x)
    return x*(1-x)

#weights = 2*np.random.random((3,1)) -1
weights_1 = np.array([[0.6,0.4,0.2,0.1]]).T
weights_0 = np.array([[0.5,0.3,0.1,0.7],
                      [0.9,0.3,0.5,0.3],
                      [0.2,0.4,0.7,0.8]])
for iter in xrange(2000):
    l0 = X # 4*3 matrix
    l1 = sigmoid(np.dot(X,weights_0)) # 4*4 matrix
    l2 = sigmoid(np.dot(l1,weights_1)) # 4*1 matrix

    error_2 = y - l2 # 4*1 matrix
    l2_delta = error_2*slope(l2) # 4*1 matrix
    error_1 = np.dot(l2_delta,weights_1.T) # 4*4 matrix, each row is one batch
    l1_delta = error_1*slope(l1) # 4*4 matrix, multiply by the confidence

    weights_0 += np.dot(X.T,l1_delta) # 3*4 matrix
    weights_1 += np.dot(l1.T,l2_delta) # 4*1 matrix

print "output after training : "
print l2
