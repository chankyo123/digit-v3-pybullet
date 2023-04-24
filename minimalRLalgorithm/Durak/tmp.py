import numpy as np
position = np.random.rand(3,3)
la_position = np.random.rand(3,3)
print(position)
print(la_position)
stack = np.stack((position,la_position),axis=-1)
print(stack)
print(stack.shape)