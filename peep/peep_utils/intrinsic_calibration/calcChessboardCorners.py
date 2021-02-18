import numpy as np

def calcChessboardCorners(target_shape, square_size, pattern_type):
    if(len(target_shape) != 2):
        print ("Invalid target shape")
        return [], []
    
    corners = np.zeros([target_shape[0]*target_shape[1], 3], dtype=np.float32)
    idx = 0
    if(pattern_type == 'chessboard' or pattern_type == 'circles'):
        center = [target_shape[0]*square_size/2., target_shape[1]*square_size/2., 0]
        for i in range(0,target_shape[1]):
            for j in range(0, target_shape[0]):
                corners[idx, 0] = j * square_size
                corners[idx, 1] = i * square_size
                idx += 1
        return corners, center
    elif(pattern_type == 'acircles'):
        center = [target_shape[1]*square_size/2., (target_shape[0]+.5)*square_size/2., 0]
        for i in range(0,target_shape[1]):
            for j in range(0, target_shape[0]):
                corners[idx, 0] = (2*j + i % 2) * square_size
                corners[idx, 1] = i * square_size
                idx += 1
        return corners, center
    else:
        print ('Invalid Pattern Type [chessboard, circles or acircles]: {}'.format(pattern_type))
        return [], []
