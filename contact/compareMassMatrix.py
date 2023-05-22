import numpy as np

# Load the text files
matrix1 = np.loadtxt('MassMatrix-bullet.txt')
matrix2 = np.loadtxt('MassMatrix-mujoco.txt')

# Compute the element-wise difference
diff_matrix = np.abs(matrix1 - matrix2)

# Specify the threshold value
threshold = 0.02  # Adjust this value as per your requirement

# Check if any element-wise difference is greater than the threshold
are_different = np.any(diff_matrix > threshold)

# Print the result
if are_different:
    print("Element-wise differences are greater than the threshold.")
else:
    print("Element-wise differences are not greater than the threshold.")
    
print(diff_matrix > threshold)
np.savetxt('compareMassMatrix.txt', diff_matrix > threshold, fmt='%.0f')

