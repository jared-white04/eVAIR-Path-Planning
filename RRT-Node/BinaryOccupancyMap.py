import numpy as np

def LoadBinaryMapFromFile(filename):
    file = filename + '.npy'
    binaryMap = np.load(file)
    return binaryMap

def Get2DBinaryMapSlice(filename, z_index):
    binaryMap3D = LoadBinaryMapFromFile(filename)
    if z_index < 0 or z_index >= binaryMap3D.shape[2]:
        raise ValueError("z_index is out of bounds for the 3D binary map.")
    binaryMap2D = binaryMap3D[:, :, z_index]
    return binaryMap2D
    
def main(args=None):
    binaryMap = LoadBinaryMapFromFile('voxel_grid')
    print(binaryMap)

if __name__ == '__main__':
    main()