from __future__ import print_function

import numpy as np
import cv2
import colorsys
import math


ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

pcd_header = '''# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH %(width_num)d
HEIGHT %(height_num)d
VIEWPOINT 0 0 0 1 0 0 0
POINTS %(num_pts)d
DATA ascii
'''
def write_camera_info_yaml (camera_info, camera_name,file_name = None):
    if file_name is None:
        file_name = camera_name+'.yaml'
    camera_dic = {}
    camera_dic["image_height"] = camera_info.height
    camera_dic["image_width"] = camera_info.width
    camera_dic["camera_name"] = camera_name
    camera_dic["camera_matrix"]  = {"rows":3, "cols":3,"data":np.ravel(camera_info.camera_matrix).tolist()}
    dist_coeff_list = np.ravel(camera_info.distortion_coefficients).tolist()
    if (len (dist_coeff_list) == 5):
        camera_dic["distortion_model"] = "plumb_bob"
    else:
        camera_dic["distortion_model"] = "rational_polynomial"
    camera_dic["distortion_coefficients"] =  {"rows":1, "cols":len(dist_coeff_list),"data":dist_coeff_list}
    camera_dic["rectification_matrix"]  =  {"rows":3, "cols":3,"data":np.ravel(camera_info.rectification_matrix).tolist()}
    camera_dic["projection_matrix"] =  {"rows":3, "cols":4,"data":np.ravel(camera_info.projection_matrix).tolist()}
    
    overall_ordering = ['image_width','image_height','camera_name', 'camera_matrix','distortion_model' , 'distortion_coefficients',
                        'rectification_matrix','projection_matrix']
    ordering_mat = ['rows','cols','data']
    
    with open (file_name,'w') as w:
        for field in overall_ordering:
            val = camera_dic[field]
           
            if(type(val) == dict):
                print ('{}:'.format(field),file=w)
                print ('{}:'.format(field))
                for sub_field in ordering_mat:
                    sub_val = val[sub_field]
                    print ('  {}: {}'.format(sub_field,sub_val),file=w)
                    print ('  {}: {}'.format(sub_field,sub_val))
            else:
                print ('{}: {}'.format( field,val),file=w)
                print ('{}: {}'.format( field,val))



def write_ply(fn, verts, colors):
    #verts = verts.reshape(-1, 3)
    #colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'w') as f:
        f.write(ply_header % dict(vert_num=len(verts)))
        np.savetxt(f, verts, '%f %f %f %d %d %d')
        
def write_pcd(fn, verts, width, height):
    #verts = verts.reshape(-1, 3)
    #colors = colors.reshape(-1, 3)
    print(verts.shape)
    with open(fn, 'w') as f:
        f.write(pcd_header % dict(width_num=width,height_num=height, num_pts=width*height))
        np.savetxt(f, verts, '%f %f %f')
        
        
def calcChessboardCorners(target_shape, square_size, pattern_type):
    if(len(target_shape) != 2):
        print("Invalid target shape")
        return
    
    corners = np.zeros([target_shape[0]*target_shape[1], 3])
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
        print('Invalid Pattern Type [chessboard, circles or acircles]:', pattern_type)
        return []

def pseudocolor(val, minval, maxval):
    # convert val in range minval..maxval to the range 0..120 degrees which
    # correspond to the colors red..green in the HSV colorspace
    h = (float(val-minval) / (maxval-minval)) * 360
    # convert hsv color (h,1,1) to its rgb equivalent
    # note: the hsv_to_rgb() function expects h to be in the range 0..1 not 0..360
    r, g, b = colorsys.hsv_to_rgb(h/360, 1., 1.)
    return (255*r, 255*g, 255*b)


def comp_distortion_oulu(xd, k):
    k1 = k[0]
    k2 = k[1]
    k3 = k[4]
    p1 = k[2]
    p2 = k[3]
    
    x = xd; 				# initial guess
    
    for kk in range(20):
        r_2 = np.sum(x**2)
        k_radial =  1 + (k1 * r_2) + (k2 * r_2**2) + (k3 * r_2**3)
        delta_x = np.array([2*p1*x[0,:] * x[1,:] + p2*(r_2 + 2*x[0,:]**2),
                            p1*(r_2 + 2*x[1,:]**2) + 2*p2*x[0,:]*x[1,:]]);
        x = (xd - delta_x) / (np.ones((2,1))*k_radial)
    return x
 
    
def normalize_pixel(x_kk,cam_mat, dist_coeffs):
    fc = np.array ([cam_mat[0,0], cam_mat[1,1]])
    cc = np.array ([cam_mat[0,2], cam_mat[1,2]])
    # Subtract principal point, and divide by the focal length:
    xn = np.array([(x_kk[0,:].ravel() - cc[0]) / fc[0], (x_kk[1,:].ravel() - cc[1])/fc[1]])
    #compensate for lens distortion
    xn = comp_distortion_oulu(xn,dist_coeffs)
    return xn

def boardStats(centers, shape, target_shape):
        Xs = centers[:,:,0]
        Ys = centers[:,:,1]
        height,width = shape
        up_left    = centers[0,0]
        up_right   = centers[target_shape[0] - 1,0]
        down_right = centers[-1,0]
        down_left  = centers[-target_shape[0],0]

        p = (down_right - up_right) + (down_left - down_right)
        q = (up_right - up_left) + (down_right - up_right)
        area = abs(p[0]*q[1] - p[1]*q[0]) / 2.
        border = math.sqrt(area)
        # For X and Y, we "shrink" the image all around by approx. half the board size.
        # Otherwise large boards are penalized because you can't get much X/Y variation.
        p_x = min(1.0, max(0.0, (np.mean(Xs) - border / 2) / (width  - border)))
        p_y = min(1.0, max(0.0, (np.mean(Ys) - border / 2) / (height - border)))
        p_size = math.sqrt(area / (width * height))

        angle = lambda a, b, c: math.acos(np.dot(a-b,c-b) / (np.linalg.norm(a-b) * np.linalg.norm(c-b)))
        skew = min(1.0, 2. * abs((math.pi / 2.) - angle(up_left, up_right, down_right)))
        return p_x, p_y, p_size, skew

def isViewNovel(new_view_params, current_view_params, param_diff_threshold = 0.02):# = [0,0,0,0,0]):
    if(len(current_view_params) == 0):
        return True        
    param_diff  = np.abs(np.array(new_view_params)[[0,1,3]] - np.array(current_view_params)[:,[0,1,3]])
    param_diff_min_max = min(np.amax(param_diff,axis=1))
    return param_diff_min_max > param_diff_threshold