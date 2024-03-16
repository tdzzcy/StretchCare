import numpy as np

# Read in three NumPy files
file1 = 'mat1.npy'
file2 = 'mat2.npy'
file3 = 'mat3.npy'
file4 = 'mat4.npy'
file5 = 'mat5.npy'

array1 = np.load(file1)
array2 = np.load(file2)
array3 = np.load(file3)
array4 = np.load(file4)
array5 = np.load(file5)
# Compute the average array
average_array = (array1 + array2 + array3+array4+array5) / 5

# Save the average array to a new NumPy file
average_file = '/home/hello-robot/cse481/team5/CSE481C/lab6/average.npy'
np.save(average_file, average_array)
