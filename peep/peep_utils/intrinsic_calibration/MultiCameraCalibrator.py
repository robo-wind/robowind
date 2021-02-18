#!/usr/bin/env python

import math

from calcChessboardCorners import calcChessboardCorners

import cv2

import numpy as np
import logging

class ExtrinsicType(object):
    AllToOne = 0
    Sequential = 1
    AllToAll = 2
    
    
class CameraCalibrationInfo:
    height = None
    width = None
    camera_matrix = None
    distortion_coefficients = None
    intrinsic_error = None
    rotation = None
    translation = None
    essential_matrix = None
    fundamental_matrix = None
    projection_matrix = None
    rectification_matrix = None
    valid_box = None
    primary_projection_matrix = None    
    primary_rectification_matrix = None
    primary_valid_box = None
    disparity_to_depth = None
    extrinsic_error = None

class MultiCameraCalibrator:
        
    def __init__ (self,
                  target_shape,
                  square_size,
                  target_type,
                  primary_camera_idx = 0,
                  num_cameras = 2,
                  param_diff_threshold = 0.2,
                  min_intrinsic_images = 10,
                  min_extrinsic_image_pairs = 10,
                  reprojection_error_threshold = 4,
                  calibrate_intrinsics = True,
                  use_init_intrinsic = False,
                  fixed_principle_point = False,
                  fixed_aspect_ratio = False,
                  zero_tangent_distortion = False,
                  use_rational_model = False,
                  fix_K1 = False,
                  fix_K2 = False,
                  fix_K3 = False,
                  fix_K4 = False,
                  fix_K5 = False,
                  fix_K6 = False,
                  logging_level = 'DEBUG',
                  log_filename = 'log.txt'):
        
        # Save logging info for connections
        self._log_filename = log_filename
        
        # Initialize logging
        numeric_level = getattr(logging, logging_level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % logging_level)
        self._logging_level = logging_level
        
        if(log_filename == ''):
            logging.basicConfig(level=numeric_level)
        else:
            logging.basicConfig(level=numeric_level, filename=self._log_filename)        
        
        logging.debug("Number of Cameras: %d", num_cameras)
        self._num_cameras = num_cameras
        
        self.setMinimumIntrinsicImages(min_intrinsic_images)
        self.setMinimumExtrinsicImagePairs(min_extrinsic_image_pairs)
        self.setReprojectionErrorThreshold(reprojection_error_threshold)
        self.setImageParameterDifferenceThreshold(param_diff_threshold)

        logging.debug("Primary Camera: %d", primary_camera_idx)
        if(primary_camera_idx >= 0 and primary_camera_idx < self._num_cameras):
            self.setExtrinsicType(ExtrinsicType.AllToOne, primary_camera_idx)
        elif(primary_camera_idx == -1):
            self.setExtrinsicType(ExtrinsicType.Sequential)
        elif(primary_camera_idx == -2):
            self.setExtrinsicType(ExtrinsicType.AllToAll)
        else:
            logging.error("Invalid Primary Camera: %d", primary_camera_idx)
            raise NameError("Invalid Primary Camera: ", primary_camera_idx)
        
        if(type(calibrate_intrinsics) is bool):
            self._calibrate_intrinsics = [calibrate_intrinsics]*self._num_cameras
        elif(type(calibrate_intrinsics) is list or type(calibrate_intrinsics) is tuple):
            self._calibrate_intrinsics = list(calibrate_intrinsics)
        else:
            logging.error("Invalid calibrate_intrinsics type (bool, list, tuple): %s", str(type(calibrate_intrinsics)))
            raise TypeError('Invalid calibrate_intrinsics type (bool, list, tuple): %s' % str(type(calibrate_intrinsics)))
            
        self.setCalibrationFlags(use_init_intrinsic,
                                 fixed_principle_point,
                                 fixed_aspect_ratio,
                                 zero_tangent_distortion,
                                 use_rational_model,
                                 fix_K1,
                                 fix_K2,
                                 fix_K3,
                                 fix_K4,
                                 fix_K5,
                                 fix_K6)
                
        self.setTargetParams(target_shape,
                             square_size,
                             target_type)
        
        self.resetCalibrationData()
        
    def resetCalibrationData(self):
        logging.debug("Resetting Calibration Data")
        self._target_image_set = [[] for i in range(self._num_cameras)]
        self._target_centers_set = [[] for i in range(self._num_cameras)]
        self._model_centers_set = []
        self._initial_camera_calibration_set = [[] for i in range(self._num_cameras)]
        #################################################################################
        self._target_params = [[[] for i in range(self._num_cameras)] for j in range(self._num_cameras)]
#        self._target_params = [[] for i in xrange(self._num_cameras)]

        self.camera_calibration_set = [CameraCalibrationInfo() for i in range(self._num_cameras)]
        self.intrinsic_image_count = [0]*self._num_cameras
        
    def setCalibrateIntrinsics(self, camera_index = None, calibrate_intrinsics = True):
        if(camera_index is None):
            self._calibrate_intrinsics = [calibrate_intrinsics]*self._num_cameras
        elif(camera_index >= 0 and camera_index < self._num_cameras):
            self._calibrate_intrinsics[camera_index] = calibrate_intrinsics
        else:
            logging.error("Invalid Camera Index: %d", camera_index)
            raise IndexError("Invalid Camera Index: ", camera_index)

        
    def setExtrinsicType(self, extrinsic_type, primary_camera_idx = 0):
        self.primary_camera_idx = primary_camera_idx
        if(extrinsic_type == ExtrinsicType.AllToOne):
            if(primary_camera_idx < self._num_cameras and primary_camera_idx >= 0):
                logging.debug("Extrinsic Type: All To One (%d)", primary_camera_idx)
                self.extrinsic_type = ExtrinsicType.AllToOne
            else:
                logging.error("Invalid Primary Camera: %d", primary_camera_idx)
                raise IndexError("Invalid Primary Camera: ", primary_camera_idx)
        elif(extrinsic_type == ExtrinsicType.Sequential):
            logging.debug("Extrinsic Type: Sequential")
            self.extrinsic_type = ExtrinsicType.Sequential
        elif(extrinsic_type == ExtrinsicType.AllToAll):
            logging.debug("Extrinsic Type: All To All")
            self.extrinsic_type = ExtrinsicType.AllToAll            
        else:
            logging.error("Invalid Extrinsic Type: %s", str(extrinsic_type))
            raise NameError("Invalid Extrinsic Type: ", extrinsic_type)
            
    def setReprojectionErrorThreshold(self, reprojection_error_threshold):
        logging.debug("Maximum Reprojection Error: %f", reprojection_error_threshold)
        self._reprojection_error_threshold = reprojection_error_threshold
    
    def setImageParameterDifferenceThreshold(self, param_diff_threshold):
        logging.debug("Minimum Parameter Difference: %f", param_diff_threshold)
        self._param_diff_threshold = param_diff_threshold

    def setMinimumIntrinsicImages(self, min_intrinsic_images):
        logging.debug("Minimum Intrinsic Images: %d", min_intrinsic_images)
        self._min_intrinsic_images = min_intrinsic_images
        
    def setMinimumExtrinsicImagePairs(self, min_extrinsic_image_pairs):
        logging.debug("Minimum Extrinsic Image Pairs: %d", min_extrinsic_image_pairs)
        self._min_extrinsic_image_pairs = min_extrinsic_image_pairs

    def setLoggingLevel(self, logging_level):
        numeric_level = getattr(logging, logging_level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % logging_level)
        logging.debug("Setting Logging Level to %s", logging_level)
        self._logging_level = logging_level
        logging.getLogger().setLevel(numeric_level)

    def setLogFile(self, log_filename):
        logging.debug("Setting Log File to %s", log_filename)
        self._log_filename = log_filename        
        # Initialize logging
        numeric_level = getattr(logging, self._logging_level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % self._logging_level)
        
        if(log_filename == ''):
            logging.basicConfig(level=numeric_level)
        else:
            logging.basicConfig(level=numeric_level, filename=self._log_filename)

    def setCalibrationFlags(self,
                            use_init_intrinsic,
                            fixed_principle_point,
                            fixed_aspect_ratio,
                            zero_tangent_distortion,
                            use_rational_model,
                            fix_K1,
                            fix_K2,
                            fix_K3,
                            fix_K4,
                            fix_K5,
                            fix_K6):
                                
        logging.debug("Setting Calibration Flags")
        self._intrinsic_calibration_flags = 0
        
        if(use_init_intrinsic):
            logging.debug("Using Initial Intrinsics")
            self._intrinsic_calibration_flags += cv2.CALIB_USE_INTRINSIC_GUESS
        
        if(fixed_principle_point):
            logging.debug("Using Fixed Principle Point")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_PRINCIPAL_POINT

        if(fixed_aspect_ratio):
            logging.debug("Using Fixed Aspect Ratio")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_ASPECT_RATIO

        if(zero_tangent_distortion):
            logging.debug("Using Zero Tangential Distortion")
            self._intrinsic_calibration_flags += cv2.CALIB_ZERO_TANGENT_DIST

        if(use_rational_model):
            logging.debug("Using Rational Model")
            self._intrinsic_calibration_flags += cv2.CALIB_RATIONAL_MODEL

        if(fix_K1):
            logging.debug("Using Fixed K1")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_K1
        if(fix_K2):
            logging.debug("Using Fixed K2")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_K2
        if(fix_K3):
            logging.debug("Using Fixed K3")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_K3
        if(fix_K4):
            logging.debug("Using Fixed K4")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_K4
        if(fix_K5):
            logging.debug("Using Fixed K5")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_K5
        if(fix_K6):
            logging.debug("Using Fixed K6")
            self._intrinsic_calibration_flags += cv2.CALIB_FIX_K6

    def setTargetParams(self, 
                        target_shape,
                        square_size,
                        target_type):
        logging.debug("Setting Target Params")

        if(type(target_shape) is str):
            target_shape_part = target_shape.partition('x')
            if(len(target_shape_part) != 3):
                logging.error("Invalid Target Shape: %s", target_shape)
                raise NameError("Invalid Target Shape:", target_shape)            
            try:
                target_cols = int(target_shape_part[0])
                target_rows = int(target_shape_part[2])
            except ValueError:
                logging.error("Invalid Target Shape: %s", target_shape)
                raise NameError("Invalid Target Shape:", target_shape)
        elif((type(target_shape) is tuple or type(target_shape) is list) and len(target_shape) == 2):
            target_cols = target_shape[0]
            target_rows = target_shape[1]
        else:
            logging.error("Invalid Target Shape Type: %s", target_shape)
            raise NameError("Invalid Target Shape Type:", target_shape)

        logging.debug("Target Shape: %d x %d", target_cols, target_rows)
        self._target_shape = (target_cols, target_rows)        
        logging.debug("Target Square Size: %f", square_size)
        self._target_square_size = square_size
        logging.debug("Target Type: %s", target_type)
        self._target_type = target_type
        if(self._target_type == 'acircles'):
            self._target_flags = cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING
        elif(self._target_type == 'circles' or self._target_type == 'chessboard'):
            self._target_flags = cv2.CALIB_CB_SYMMETRIC_GRID #+ cv2.CALIB_CB_CLUSTERING
        else:
            logging.error("Invalid Target Type [chessboard, circles or acircles]: %s", self._target_type)
            raise NameError("Invalid Target Type [chessboard, circles or acircles]:", self._target_type)
                    
        [self._target_points, self._target_center] = calcChessboardCorners(self._target_shape, self._target_square_size, self._target_type)
        
        
    def captureTarget (self, image_set, camera_info_set = None):
        logging.debug("Checking for Target in %d Images", len(image_set))
        centers_set = []
        for i, image in enumerate(image_set):
            if(image is None):
                centers_set += [None]
            else:
                image_array = np.asarray(image)
                if(len(image_array.shape) > 2 and image_array.shape[2] == 3):
                    image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
                cv2.imwrite("testout.png", image_array)
                print (self._target_shape)
                if(self._target_type == 'circles'  or self._target_type == 'acircles' ):
                    [ret_val, centers] = cv2.findCirclesGrid(image_array, self._target_shape, flags=self._target_flags)
                elif(self._target_type == 'chessboard'):
                    width = image_array.shape[0]
                    height = image_array.shape[1]
                    scale = math.sqrt( (height*width) / (640.*480.) )
                    if (scale > 1.0):
                        scaled_image = cv2.resize(image_array, (int(height / scale), int(width / scale)))
                    else:
                        scaled_image = image_array
                    x_scale = float(height) / scaled_image.shape[1]
                    y_scale = float(width) / scaled_image.shape[0]
                    
                    [ret_val, centers] = cv2.findChessboardCorners(scaled_image, self._target_shape, flags=self._target_flags)
                    
                    if(ret_val):
                        logging.debug("Chessboard found in %s scale Image %d", scale, i)
                        if(scale > 1.0):
                            centers[:,:,0] *= x_scale
                            centers[:,:,1] *= y_scale
                            radius = int(math.ceil(scale)*5)
                        else:
                            radius = 5
                            
                        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
                        logging.debug("Corners refined using %d pixel radius in Image %d", radius, i)
                        cv2.cornerSubPix(image_array, centers, (radius,radius), (-1,-1), criteria)
                else:
                    logging.error("Invalid Target Type: %s", self._target_type)
                    raise NameError("Invalid Target Type: ", self._target_type)
                    return
                
                if(not ret_val):
                    logging.debug("Centers not found in Image %d", i)
                    centers_set += [None]
 ########################################################################################################################################                    
#                    return [None]*self._num_cameras, [None]*self._num_cameras

                else:
                    print ("Target Found in Image %d" % i)
                    centers_set += [centers]

        params_set = []
        novel_centers_set = []
        centers_idxs = [idx for idx, val in enumerate(centers_set) if val is not None]
        for i, centers in enumerate(centers_set):
            if(centers is None):
                params_set += [None]
                novel_centers_set += [None]
            else:
                if(camera_info_set is not None and camera_info_set[i] is not None and camera_info_set[i].camera_matrix is not None):
                    camera_matrix = camera_info_set[i].camera_matrix
                else:
                    model_centers = []
                    model_centers.append(np.array(self._target_points, dtype=np.float32))
                    image_centers = []
                    image_centers.append(np.array(centers, dtype=np.float32))
                    camera_matrix = cv2.initCameraMatrix2D(model_centers, image_centers, image_array.shape)
                if(camera_info_set is not None and camera_info_set[i].distortion_coefficients is not None):
                    dist_coeffs = camera_info_set[i].distortion_coefficients
                else:
                    dist_coeffs = None        
                
                [ret_pnp, r_vec, t_vec] = cv2.solvePnP(self._target_points, centers, camera_matrix, dist_coeffs)
                if(not ret_pnp):
                    logging.debug("PNP Failed on Image %d", i)
                    params_set += [None]
                    novel_centers_set += [None]
                    continue
                [r_mat, r_jac] = cv2.Rodrigues(r_vec)
                norm_vec = -r_mat[:, 2]
                norm_vec = norm_vec/np.linalg.norm(norm_vec)
                target_col = np.mean(centers[:,:,0]) / image_array.shape[0]
                target_row = np.mean(centers[:,:,1]) / image_array.shape[1]
                target_area = self.boardStats(centers)
                target_area =  target_area / (image_array.shape[0] * image_array.shape[1])
                target_pitch = norm_vec[1]
                target_yaw = norm_vec[0]
 
                logging.debug("Target Params in Image %d: col: %f row: %f area: %f pitch: %f yaw: %f", 
                             i, target_col, target_row, target_area, target_pitch, target_yaw)

                target_params = [target_col, target_row, target_area, target_pitch, target_yaw]
                is_novel = False
                for j in centers_idxs:
                    if(self.isViewNovel(target_params, self._target_params[i][j], self._param_diff_threshold)):
                       is_novel = True
                       self._target_params[i][j] += [target_params]
                       
                if(is_novel):
                    logging.debug("Image %d added", i)                        
                    novel_centers_set += [centers]
                    params_set += [target_params]
                else:
                    logging.debug("Image %d is not novel enough", i)
                    novel_centers_set += [None]
                    params_set += [None]     
                    
        if(len(filter(lambda x: x != None, novel_centers_set)) > 0):
            if(camera_info_set is None):
                camera_info_set = [None]*len(image_set)
            for i, (image, centers, camera_info) in enumerate(zip(image_set, novel_centers_set, camera_info_set)):
                self._target_image_set[i].append(image)
                self._target_centers_set[i].append(centers)
                self._model_centers_set.append(self._target_points)
                self._initial_camera_calibration_set[i].append(camera_info)
        
        return centers_set, params_set

    def isViewNovel(self, new_view_params, current_view_params, param_diff_threshold = 0.02):# = [0,0,0,0,0]):
        if(len(current_view_params) == 0):
            return True        
        min_dist = min([np.sum(np.abs(np.matrix(new_view_params) - np.matrix(p))) for p in current_view_params])
        logging.debug("Min parameter difference: %f", min_dist)

        return min_dist > param_diff_threshold

    def boardStats(self, centers):
        up_left    = centers[0,0]
        up_right   = centers[self._target_shape[0] - 1,0]
        down_right = centers[-1,0]
        down_left  = centers[-self._target_shape[0],0]
        
        a = up_right - up_left
        b = down_right - up_right
        c = down_left - down_right
        
        p = b + c
        q = a + b
        area = abs(p[0]*q[1] - p[1]*q[0]) / 2.
        return area
        
    def calibrate(self):

        insufficient_images = False
        self.intrinsic_image_count = [len(filter(lambda x: x != None, centers)) for centers in self._target_centers_set]
        for i, x in enumerate(self.intrinsic_image_count):
            if(x < self._min_intrinsic_images):
                logging.warn("Insufficient Number of Images for Camera %d: %d", i, x)
                insufficient_images = True

        if(insufficient_images):
            return False

        self.image_size_set = zip([images[0].shape[0] for images in self._target_image_set], 
                          [images[0].shape[1] for images in self._target_image_set])
        
        self.calibrateIntrinsics()
        self.calibrateExtrinsics()

        return True

    def calibrateIntrinsics(self):

        for i, (centers, camera_info, image_size) in enumerate(zip(self._target_centers_set, self._initial_camera_calibration_set, self.image_size_set)):
            init_camera_info = self.camera_calibration_set[i]
            init_dist_coeffs = init_camera_info.distortion_coefficients
            init_camera_matrix = init_camera_info.camera_matrix
            if(init_dist_coeffs is None or init_camera_matrix is None):
                try:
                    last_camera_info = filter(lambda x: x != None,camera_info)[-1]
                    init_dist_coeffs = np.asfarray(last_camera_info.distortion_coefficients, dtype=np.float32)
                    init_camera_matrix = np.asfarray(last_camera_info.camera_matrix, dtype=np.float32)
                except IndexError:
                    pass
                
            if(self._calibrate_intrinsics[i] or init_dist_coeffs is None or init_camera_matrix is None):

                filtered_centers = filter((lambda x: x[0] != None),zip(centers,self._model_centers_set))

                if(len(filtered_centers) > self._min_intrinsic_images):
                    logging.info("Calculating Camera Intrinsics for Camera %d with %d images", i, len(centers))
                    
                    image_centers = list(zip(*filtered_centers)[0])
                    model_centers = list(zip(*filtered_centers)[1])
    
                    retval, camera_matrix, dist_coeffs, r_vecs, t_vecs = cv2.calibrateCamera(model_centers, image_centers, 
                                                                                         image_size, init_camera_matrix, init_dist_coeffs, 
                                                                                         flags = self._intrinsic_calibration_flags)
                    logging.info("Reprojection Error for Camera %d: %f", i, retval)                    
                    logging.info("Camera Matrix for Camera %d: \n %s", i, str(camera_matrix))
                    logging.info("Distortion Coefficents for Camera %d: \n %s", i, str(dist_coeffs))

                else:
                    logging.warn("Insufficient images for Intrinsic Camera %d: %d", i, len(centers))
                    retval = None
                    camera_matrix = init_camera_matrix
                    dist_coeffs = init_dist_coeffs
            else:
                retval = None
                camera_matrix = init_camera_matrix
                dist_coeffs = init_dist_coeffs
            
            camera_info = CameraCalibrationInfo()
            camera_info.camera_matrix = camera_matrix
            camera_info.distortion_coefficients = dist_coeffs
            camera_info.width = image_size[0]
            camera_info.height = image_size[1]
            camera_info.intrinsic_error = retval
            
            self.camera_calibration_set[i] = camera_info
        
    def calibrateExtrinsics(self):
        if(self.extrinsic_type == ExtrinsicType.AllToOne):
            self.calibrateExtrinsicsAllToOne()
        elif(self.extrinsic_type == ExtrinsicType.Sequential):
            self.calibrateExtrinsicsSequential()
        elif(self.extrinsic_type == ExtrinsicType.AllToAll):
            self.calibrateExtrinsicsAllToAll()
        else:
            logging.error("Invalid Extrinsic Type: %d", self.extrinsic_type)
            raise NameError("Invalid Extrinsic Type: ", self.extrinsic_type)

    def calibrateProjections(self, alpha):
        if(self.extrinsic_type == ExtrinsicType.AllToOne):
            self.calibrateProjectionAllToOne(alpha)
        elif(self.extrinsic_type == ExtrinsicType.Sequential):
            self.calibrateProjectionSequential(alpha)
        elif(self.extrinsic_type == ExtrinsicType.AllToAll):
            self.calibrateProjectionAllToAll(alpha)
        else:
            logging.error("Invalid Extrinsic Type: %d", self.extrinsic_type)
            raise NameError("Invalid Extrinsic Type: ", self.extrinsic_type)

    def calibrateExtrinsicsAllToOne(self):
        logging.debug("Calculating All to One Extrinsics to Camera %d", self.primary_camera_idx)
        
        primary_centers = self._target_centers_set[self.primary_camera_idx]
        primary_camera_matrix = self.camera_calibration_set[self.primary_camera_idx].camera_matrix
        primary_dist_coeffs = self.camera_calibration_set[self.primary_camera_idx].distortion_coefficients
       
        if(primary_camera_matrix is None or primary_dist_coeffs is None):
            logging.warn("Primary Camera %d is not calibrated", self.primary_camera_idx)
            return
       
       
        for i in range(len(self._target_centers_set)):
            if(i != self.primary_camera_idx):

                secondary_centers = self._target_centers_set[i]
                secondary_camera_matrix = self.camera_calibration_set[i].camera_matrix
                secondary_dist_coeffs = self.camera_calibration_set[i].distortion_coefficients
                secondary_image_size = (self.camera_calibration_set[i].width, self.camera_calibration_set[i].height)
                
                if(secondary_camera_matrix is None or secondary_dist_coeffs is None):
                    logging.warn("Secondary Camera %d is not calibrated", i)
                    continue

                filtered_centers = filter((lambda x: x[0] != None and x[1] != None),zip(primary_centers,secondary_centers, self._model_centers_set))
                if(len(filtered_centers) >= self._min_extrinsic_image_pairs):
                    logging.info("Calibrating Camera %d to Camera %d using %d images", i, self.primary_camera_idx, len(filtered_centers))
                
                    filtered_primary_centers = list(zip(*filtered_centers)[0])
                    filtered_secondary_centers = list(zip(*filtered_centers)[1])
                    filtered_model_centers = list(zip(*filtered_centers)[2])
                     
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
                    if(False and self._calibrate_intrinsics[i] and self._calibrate_intrinsics[self.primary_camera_idx]):
                        extrinsic_calibration_flags = cv2.CALIB_USE_INTRINSIC_GUESS | self._intrinsic_calibration_flags
                    else:
                        extrinsic_calibration_flags = cv2.CALIB_FIX_INTRINSIC
                    
                    retval, CM1, DC1, CM2, DC2, R, T, E, F = cv2.stereoCalibrate(filtered_model_centers, filtered_primary_centers, filtered_secondary_centers,
                                                                                 secondary_image_size, primary_camera_matrix, primary_dist_coeffs,
                                                                                 secondary_camera_matrix, secondary_dist_coeffs, None, None, None, None,
                                                                                 criteria, extrinsic_calibration_flags)
                                                                                 
                    if(retval):
                        logging.info("Reprojection Error for Camera %d to Camera %d: %f", i, self.primary_camera_idx, retval)
                        self.camera_calibration_set[i].extrinsic_error = retval                        
                        logging.info("Rotation for Camera %d to Camera %d: \n %s", i, self.primary_camera_idx, str(R))
                        self.camera_calibration_set[i].rotation = R
                        logging.info("Translation for Camera %d to Camera %d: \n %s", i, self.primary_camera_idx, str(T))
                        self.camera_calibration_set[i].translation = T
                        logging.info("Essential Matrix for Camera %d to Camera %d: \n %s", i, self.primary_camera_idx, str(E))
                        self.camera_calibration_set[i].essential_matrix = E
                        logging.info("Fundamental for Camera %d to Camera %d: \n %s", i, self.primary_camera_idx, str(F))
                        self.camera_calibration_set[i].fundamental_matrix = F
                    else:
                        logging.warn("Stereo Calibration failed for Camera %d to Camera %d: %s", i, self.primary_camera_idx)
                else:
                    logging.warn("Insufficient images for Camera %d to Camera %d: %d", i, self.primary_camera_idx, len(filtered_centers))
                    
    def calibrateExtrinsicsSequential(self):
        logging.debug("Calculating Sequential Extrinsics")
        
        primary_centers = self._target_centers_set[0]
        primary_camera_matrix = self.camera_calibration_set[0].camera_matrix
        primary_dist_coeffs = self.camera_calibration_set[0].distortion_coefficients
        
        for i in range(1, len(self._target_centers_set)):

            secondary_centers = self._target_centers_set[i]
            secondary_camera_matrix = self.camera_calibration_set[i].camera_matrix
            secondary_dist_coeffs = self.camera_calibration_set[i].distortion_coefficients
            secondary_image_size = (self.camera_calibration_set[i].width, self.camera_calibration_set[i].height)
            
            if(primary_camera_matrix is None or primary_dist_coeffs is None):
                logging.warn("Primary Camera %d is not calibrated", i-1)
                self.extrinsics_reprojection_error_set += [None]
            
                primary_centers = secondary_centers
                primary_camera_matrix = secondary_camera_matrix
                primary_dist_coeffs = secondary_dist_coeffs
                continue
            
            if(secondary_camera_matrix is None or secondary_dist_coeffs is None):
                logging.warn("Secondary Camera %d is not calibrated", i)
                self.extrinsics_reprojection_error_set += [None]

                primary_centers = secondary_centers
                primary_camera_matrix = secondary_camera_matrix
                primary_dist_coeffs = secondary_dist_coeffs
                continue

            filtered_centers = filter((lambda x: x[0] != None and x[1] != None),zip(primary_centers,secondary_centers, self._model_centers_set))
            if(len(filtered_centers) >= self._min_extrinsic_image_pairs):
                logging.info("Calibrating Camera %d to Camera %d using %d images", i, i-1, len(filtered_centers))

                filtered_primary_centers = list(zip(*filtered_centers)[0])
                filtered_secondary_centers = list(zip(*filtered_centers)[1])
                filtered_model_centers = list(zip(*filtered_centers)[2])
                
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
                
                if(False and self._calibrate_intrinsics[i] and self._calibrate_intrinsics[i-1]):
                    extrinsic_calibration_flags = cv2.CALIB_USE_INTRINSIC_GUESS | self._intrinsic_calibration_flags
                else:
                    extrinsic_calibration_flags = cv2.CALIB_FIX_INTRINSIC
                    
                retval, CM1, DC1, CM2, DC2, R, T, E, F = cv2.stereoCalibrate(filtered_model_centers, filtered_primary_centers, filtered_secondary_centers,
                                                                             secondary_image_size, primary_camera_matrix, primary_dist_coeffs,
                                                                             secondary_camera_matrix, secondary_dist_coeffs, None, None, None, None,
                                                                             criteria, extrinsic_calibration_flags)
            
                logging.info("Reprojection Error for Camera %d to Camera %d: %f", i, i-1, retval)
                self.camera_calibration_set[i].extrinsic_error = retval
                logging.info("Rotation for Camera %d to Camera %d: \n %s", i, i-1, str(R))
                self.camera_calibration_set[i].rotation = R
                logging.info("Translation for Camera %d to Camera %d: \n %s", i, i-1, str(T))
                self.camera_calibration_set[i].translation = T
                logging.info("Essential Matrix for Camera %d to Camera %d: \n %s", i, i-1, str(E))
                self.camera_calibration_set[i].essential_matrix = E
                logging.info("Fundamental Matrix for Camera %d to Camera %d: \n %s", i, i-1, str(F))
                self.camera_calibration_set[i].fundamental_matrix = F
            else:
                logging.warn("Insufficient images for Camera %d to Camera %d: %d", i, i-1, len(filtered_centers))
                retval = None
                            
            primary_centers = secondary_centers
            primary_camera_matrix = secondary_camera_matrix
            primary_dist_coeffs = secondary_dist_coeffs

    def calibrateExtrinsicsAllToAll(self):
        logging.debug("Calculating All to All Extrinsics")
        
        for i in range(len(self._target_centers_set)):
            
            reprojection_error_set = []
            rotation_set = []
            translation_set = []
            essential_matrix_set = []
            fundamental_matrix_set = []            
            
            primary_centers = self._target_centers_set[i]
            primary_camera_matrix = self.camera_calibration_set[i].camera_matrix
            primary_dist_coeffs = self.camera_calibration_set[i].distortion_coefficients
        
            if(primary_camera_matrix is None or primary_dist_coeffs is None):
                logging.warn("Primary Camera %d is not calibrated", i)
                self.camera_calibration_set[i].extrinsic_error = [None]*self._num_cameras
                self.camera_calibration_set[i].rotation = [None]*self._num_cameras
                self.camera_calibration_set[i].translation = [None]*self._num_cameras
                self.camera_calibration_set[i].essential_matrix = [None]*self._num_cameras
                self.camera_calibration_set[i].fundamental_matrix = [None]*self._num_cameras
                continue   
        
            for j in range(len(self._target_centers_set)):
                if(i != j):
    
                    secondary_centers = self._target_centers_set[j]
                    secondary_camera_matrix = self.camera_calibration_set[j].camera_matrix
                    secondary_dist_coeffs = self.camera_calibration_set[j].distortion_coefficients
                    secondary_image_size = (self.camera_calibration_set[i].width, self.camera_calibration_set[i].height)
                    if(secondary_camera_matrix is None or secondary_dist_coeffs is None):
                        logging.warn("Secondary Camera %d is not calibrated", j)
                        reprojection_error_set += [None]
                        rotation_set += [None]
                        translation_set += [None]
                        essential_matrix_set += [None]
                        continue
                        
                    filtered_centers = filter((lambda x: x[0] != None and x[1] != None),zip(primary_centers,secondary_centers,self._model_centers_set))
                    if(len(filtered_centers) >= self._min_extrinsic_image_pairs):
                        logging.info("Calibrating Camera %d to Camera %d using %d images", j, i, len(filtered_centers))

                        filtered_primary_centers = list(zip(*filtered_centers)[0])
                        filtered_secondary_centers = list(zip(*filtered_centers)[1])
                        filtered_model_centers = list(zip(*filtered_centers)[2])
                        
                        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        
                        if(False and self._calibrate_intrinsics[i] and self._calibrate_intrinsics[j]):
                            extrinsic_calibration_flags = cv2.CALIB_USE_INTRINSIC_GUESS | self._intrinsic_calibration_flags
                        else:
                            extrinsic_calibration_flags = cv2.CALIB_FIX_INTRINSIC
                            
                        retval, CM1, DC1, CM2, DC2, R, T, E, F = cv2.stereoCalibrate(filtered_model_centers, filtered_primary_centers, filtered_secondary_centers,
                                                                                     secondary_image_size, primary_camera_matrix, primary_dist_coeffs,
                                                                                     secondary_camera_matrix, secondary_dist_coeffs, None, None, None, None,
                                                                                     criteria, extrinsic_calibration_flags)                                                                     
                    
                        logging.info("Reprojection Error for Camera %d to Camera %d: %f", j, i, retval)
                        reprojection_error_set += [retval]
                        logging.info("Rotation for Camera %d to Camera %d: \n %s", j, i, str(R))
                        rotation_set += [R]
                        logging.info("Translation for Camera %d to Camera %d: \n %s", j, i, str(T))
                        translation_set += [T]
                        logging.info("Essential Matrix for Camera %d to Camera %d: \n %s", j, i, str(E))
                        essential_matrix_set += [E]
                        logging.info("Fundamental for Camera %d to Camera %d: \n %s", j, i, str(F))
                        fundamental_matrix_set += [F]
                            
                    else:
                        logging.warn("Insufficient images for Camera %d to Camera %d: %d", j, i, len(filtered_centers))
                        reprojection_error_set += [None]
                        rotation_set += [None]
                        translation_set += [None]
                        essential_matrix_set += [None]
                        fundamental_matrix_set += [None]
                else:
                    reprojection_error_set += [None]
                            
                    rotation_set += [None]
                    translation_set += [None]
                    essential_matrix_set += [None]
                    fundamental_matrix_set += [None]

            self.camera_calibration_set[i].extrinsic_error = [reprojection_error_set]
            self.camera_calibration_set[i].rotation = rotation_set
            self.camera_calibration_set[i].translation = translation_set
            self.camera_calibration_set[i].essential_matrix = essential_matrix_set
            self.camera_calibration_set[i].fundamental_matrix = fundamental_matrix_set
        
        
    def calibrateProjectionPair(self, primary_camera_idx, secondary_camera_idx, alpha):
        logging.info("Calculating Projection from Camera %d to Camera %d", secondary_camera_idx, primary_camera_idx)
       
        primary_camera_matrix = self.camera_calibration_set[primary_camera_idx].camera_matrix
        primary_dist_coeffs = self.camera_calibration_set[primary_camera_idx].distortion_coefficients

        if(primary_camera_matrix is None or primary_dist_coeffs is None):
            logging.warn("Primary Camera %d not calibrated", primary_camera_idx)
            return None, None, None, None, None, None, None

        secondary_camera_matrix = self.camera_calibration_set[secondary_camera_idx].camera_matrix
        secondary_dist_coeffs = self.camera_calibration_set[secondary_camera_idx].distortion_coefficients
        secondary_image_size = (self.camera_calibration_set[secondary_camera_idx].width, self.camera_calibration_set[secondary_camera_idx].height)
        if(secondary_camera_matrix is None or secondary_dist_coeffs is None or not all(secondary_image_size)):
            logging.warn("Secondary Camera %d not calibrated", secondary_camera_idx)
            return None, None, None, None, None, None, None
        
        rotation = self.camera_calibration_set[secondary_camera_idx].rotation
        if(type(rotation) is list):
            rotation = rotation[primary_camera_idx]
        translation = self.camera_calibration_set[secondary_camera_idx].translation
        if(type(translation) is list):
            translation = translation[primary_camera_idx]

        if(rotation is None or translation is None):
            logging.warn("Camera %d and %d are not stereo calibrated", primary_camera_idx, secondary_camera_idx)
            return None, None, None, None, None, None, None
        
        R_rec_primary, R_rec, P_primary, P, disp_to_depth, valid_box_primary, valid_box = cv2.stereoRectify(primary_camera_matrix,
                                                                                                            primary_dist_coeffs,
                                                                                                            secondary_camera_matrix,
                                                                                                            secondary_dist_coeffs,
                                                                                                            secondary_image_size,
                                                                                                            rotation,
                                                                                                            translation,
                                                                                                            alpha = alpha)
        logging.info("Projection Matrices for Camera %d: \n %s", secondary_camera_idx, P)
        logging.info("Projection Matrices for Camera %d: \n %s", primary_camera_idx, P_primary)
        logging.info("Rectification Matrices for Camera %d: \n %s", secondary_camera_idx, R_rec)
        logging.info("Rectification Matrices for Camera %d: \n %s", primary_camera_idx, R_rec_primary)
        logging.info("Disperity to Depth Map for Camera %d: \n %s", secondary_camera_idx, disp_to_depth)
        logging.info("Valid Image Box for Camera %d: \n %s", secondary_camera_idx, valid_box)
        logging.info("Valid Image Box for Camera %d: \n %s", self.primary_camera_idx, valid_box_primary)
        return R_rec_primary, R_rec, P_primary, P, disp_to_depth, valid_box_primary, valid_box

    def calibrateProjectionAllToOne(self, alpha):
        if(self.primary_camera_idx < 0):
            logging.warn("Invalid Primary Camera Index %d, Setting to 0", self.primary_camera_idx)
            self.primary_camera_idx = 0
        for i in range(self._num_cameras):
            if(i != self.primary_camera_idx):
                R_rec_primary, R_rec, P_primary, P, disp_to_depth, valid_box_primary, valid_box = self.calibrateProjectionPair(self.primary_camera_idx, i, alpha)
                self.camera_calibration_set[i].projection_matrix = P
                self.camera_calibration_set[i].primary_projection_matrix = P_primary
                self.camera_calibration_set[i].rectification_matrix = R_rec
                self.camera_calibration_set[i].primary_rectification_matrix = R_rec_primary
                self.camera_calibration_set[i].disparity_to_depth = disp_to_depth
                self.camera_calibration_set[i].valid_box = valid_box
                self.camera_calibration_set[i].primary_valid_box = valid_box_primary

    def calibrateProjectionSequential(self, alpha):
        for i in range(self._num_cameras):
            R_rec_primary, R_rec, P_primary, P, disp_to_depth, valid_box_primary, valid_box = self.calibrateProjectionPair(i-1, i, alpha)
            self.camera_calibration_set[i].projection_matrix = P
            self.camera_calibration_set[i].primary_projection_matrix = P_primary
            self.camera_calibration_set[i].rectification_matrix = R_rec
            self.camera_calibration_set[i].primary_rectification_matrix = R_rec_primary
            self.camera_calibration_set[i].disparity_to_depth = disp_to_depth
            self.camera_calibration_set[i].valid_box = valid_box
            self.camera_calibration_set[i].primary_valid_box = valid_box_primary

    def calibrateProjectionAllToAll(self, alpha):
        for i in range(self._num_cameras):
            self.camera_calibration_set[i].projection_matrix = [None]*self._num_cameras
            self.camera_calibration_set[i].primary_projection_matrix = [None]*self._num_cameras
            self.camera_calibration_set[i].rectification_matrix = [None]*self._num_cameras
            self.camera_calibration_set[i].primary_rectification_matrix = [None]*self._num_cameras
            self.camera_calibration_set[i].disparity_to_depth = [None]*self._num_cameras
            self.camera_calibration_set[i].valid_box = [None]*self._num_cameras
            self.camera_calibration_set[i].primary_valid_box = [None]*self._num_cameras
            
            for j in range(self._num_cameras):
                if(i != j):
                    R_rec_primary, R_rec, P_primary, P, disp_to_depth, valid_box_primary, valid_box = self.calibrateProjectionPair(i, j, alpha)
                    self.camera_calibration_set[i].projection_matrix[j] = P
                    self.camera_calibration_set[i].primary_projection_matrix[j] = P_primary
                    self.camera_calibration_set[i].rectification_matrix[j] = R_rec
                    self.camera_calibration_set[i].primary_rectification_matrix[j] = R_rec_primary
                    self.camera_calibration_set[i].disparity_to_depth[j] = disp_to_depth
                    self.camera_calibration_set[i].valid_box[j] = valid_box
                    self.camera_calibration_set[i].primary_valid_box[j] = valid_box_primary

if __name__ == '__main__':
    MultiCameraCalibrator()
    
