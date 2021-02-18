#ifndef BODY_PARTS_H_
#define BODY_PARTS_H_

#include <vector>

enum JointIndices {
    Nose, //0
    Neck, //1
    RightShoulder, //2
    RightElbow, //3
    RightWrist, //4
    LeftShoulder, //5
    LeftElbow, //6
    LeftWrist, //7
    MidHip, //8
    RightHip, //9
    RightKnee, //10
    RightAnkle, //11
    LeftHip, //12
    LeftKnee, //13
    LeftAnkle, //14
    RightEye, //15
    LeftEye,  //16
    RightEar, //17
    LeftEar,  //18
    LeftBigToe, //19
    LeftSmallToe, //20
    LeftHeel, //21
    RightBigToe, //22
    RightSmallToe, //23
    RightHeel, //24
    NUM_JOINTS //25
};

enum class JointIndicesCOCO {
    Nose, //0 //0
    Neck, //1  //1
    LeftShoulder,//2  //5
    LeftElbow, //3 //6
    LeftWrist, //4 //7
    RightShoulder,//5 //2
    RightElbow, //6 //3
    RightWrist, //7 //4
    LeftHip, //8  //12
    LeftKnee, //9  //13
    LeftAnkle, //10  //14
    RightHip, //11  //9
    RightKnee, //12  //10
    RightAnkle, //13  //11
    LeftEye, //14  //16 
    RightEye, //15 //15
    LeftEar, //16  //18
    RightEar, //17  //17  
    NUM_JOINTS //18 //25
};
                                         // {0, 1, 5, 6, 7, 2, 3, 4, -1, 11, 12, 13, 8, 9, 10, 15, 14, 17, 16, -1, -1, -1, -1, -1, -1};
static const std::vector<int> Coco2OpJoints {0, 1, 5, 6, 7, 2, 3, 4, 12, 13, 14, 9, 10, 11, 16, 15, 18, 17};
static const std::vector<int> OpJoints2Coco {0, 1, 5, 6, 7, 2, 3, 4, 1, 11, 12, 13, 8, 9, 10, 15, 14, 17, 16, -1, -1, -1, -1, -1, -1};

enum class WrnchJointIndices {
    RightAnkle, //0  //11
    RightKnee, //1  //10
    RightHip, //2 //9
    LeftHip, //3 //12
    LeftKnee, //4 //13
    LeftAnkle, //5 //14
    MidHip, //6 //8
    Thorax, //7
    Neck,  //8 //1
    Head, //9
    RightWrist,//10 //4
    RightElbow, //11 //3
    RightShoulder, //12 //2
    LeftShoulder, //13  //5
    LeftElbow, //14 //6 
    LeftWrist, //15 //7
    Nose, //16 //0
    RightEye, //17 //15
    RightEar,//18  //17
    LeftEye, //19  //16
    LeftEar, //20 //18
    RightBigToe, //21 //22
    LeftBigToe, //22 //19
    NUM_JOINTS //23 
};

static const std::vector<int> Wrnch2OpJoints {11, 10, 9, 12, 13, 14, 8, -1, 1, -1, 4, 3, 2, 5, 6, 7, 0, 15, 17, 16, 18, 22, 19};
static const std::vector<int> OpJoints2Wrnch {16, 8, 12, 11, 10, 13, 14, 15, 6, 2, 1, 0, 3, 4, 5, 17, 19, 18, 20, 22, -1, -1, 21, -1, -1};


enum HandJointIndices {
    Lunate,
    ThumbCarpalMetaCarpalJoint, //ThumbTrapezioMetaCarpalJoint,
    ThumbMetaCarpoPhalangealJoint,
    ThumbInterPhalangealJoint,
    ThumbDistalPhalanx,
    IndexMetaCarpoPhalangealJoint,
    IndexProximalInterPhalangealJoint,
    IndexDistalInterPhalangealJoint,
    IndexDistalPhalanx,
    MiddleMetaCarpoPhalangealJoint,
    MiddleProximalInterPhalangealJoint,
    MiddleDistalInterPhalangealJoint,
    MiddleDistalPhalanx,
    RingMetaCarpoPhalangealJoint,
    RingProximalInterPhalangealJoint,
    RingDistalInterPhalangealJoint,
    RingDistalPhalanx,
    PinkyMetaCarpoPhalangealJoint,
    PinkyProximalInterPhalangealJoint,
    PinkyDistalInterPhalangealJoint,
    PinkyDistalPhalanx
};
//https://arxiv.org/pdf/1704.07809.pdf
// joint points diagram in fig 4

static const size_t NUM_FACIAL_KEYPTS = 70;
static const size_t NUM_HAND_JOINTS = 21;

class Color
{
public:
    Color (const uint8_t &r, const uint8_t &g, const uint8_t  &b):
        rgb_{r,g,b} 
        {}

    uint8_t rgb_[3];
};

//From openpose/include/openpose/pose/poseParametersRender.hpp
// static const std::vector<Color> JointColors
// {
//     Color(255,     0,    85),
//     Color(    255,     0,     0), 
//     Color(    255,    85,     0), 
//     Color(255,   170,     0),
//     Color(255,   255,     0),
//     Color(170,   255,     0),
//     Color(85,   255,     0),
//     Color(0,   255,     0),
//     Color(255,   0,    0),
//     Color(0,   255,    85),
//     Color(0,   255,   170),
//     Color(0,   255,   255),
//     Color(0,   170,   255),
//     Color(0,    85,   255),
//     Color(0,     0,   255),
//     Color(255,     0,   170),
//     Color(170,     0,   255),
//     Color(255,     0,   255),
//     Color(85,     0,   255),
//     Color(0,     0,   255), 
//     Color(0,     0,   255), 
//     Color(0,     0,   255), 
//     Color(0,   255,   255), 
//     Color(0,   255,   255), 
//     Color(0,   255,   255)
// };

static const std::vector<Color> JointColors
{
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62),
    Color(232,     165,    62)
};


    
static const std::vector<unsigned int> POSE_BODY_25_PAIRS_RENDER {1,8,   1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   8,9,   9,10,  
                                                                 10,11,  8,12,  12,13,  13,14,  1,0,  0,15,  15,17,  0,16,  16,18,
                                                                 14,19,  19,20,  14,21,  11,22,  22,23,  11,24};

static const std::set<size_t> BOUNDING_BOX_JOINTS {JointIndices::Nose, 
                                      JointIndices::Neck, 
                                      JointIndices::RightShoulder,
                                      JointIndices::LeftShoulder,
                                      JointIndices::MidHip,
                                      JointIndices::RightHip,
                                      JointIndices::LeftHip,
                                      JointIndices::RightEar,
                                      JointIndices::LeftEar};




class ColorF
{
public:
    ColorF (const float &r, const float &g, const float &b):
        rgb_{r,g,b} 
        {}

    float rgb_[3];
};


//Yellow (236/255.0, 218/255.0, 95/255.0)
//Blue (57/255.0, 119/255.0, 244/255.0)
//Green (126/255.0, 187/255.0, 113/255.0)
//Orange (213/255.0, 150/255.0, 55/255.0)
//Red (197/255.0, 72/255.0, 64/255.0)
//Teal (119/255.0, 160/255.0, 217/255.0)
static const std::vector<ColorF> JointColorsF
{
    ColorF(236/255.0, 218/255.0, 95/255.0),
    ColorF(57/255.0, 119/255.0, 244/255.0), 
    ColorF(126/255.0, 187/255.0, 113/255.0), 
    ColorF(126/255.0, 187/255.0, 113/255.0),
    ColorF(213/255.0, 150/255.0, 55/255.0),
    ColorF(197/255.0, 72/255.0, 64/255.0),
    ColorF(197/255.0, 72/255.0, 64/255.0),
    ColorF(119/255.0, 160/255.0, 217/255.0),
    ColorF(57/255.0, 119/255.0, 244/255.0),
    ColorF(126/255.0, 187/255.0, 113/255.0),
    ColorF(126/255.0, 187/255.0, 113/255.0),
    ColorF(213/255.0, 150/255.0, 55/255.0),
    ColorF(197/255.0, 72/255.0, 64/255.0),
    ColorF(197/255.0, 72/255.0, 64/255.0),
    ColorF(119/255.0, 160/255.0, 217/255.0),
    ColorF(126/255.0, 187/255.0, 113/255.0),
    ColorF(197/255.0, 72/255.0, 64/255.0),
    ColorF(213/255.0, 150/255.0, 55/255.0),
    ColorF(119/255.0, 160/255.0, 217/255.0),
    ColorF(197/255.0, 72/255.0, 64/255.0),
    ColorF(119/255.0, 160/255.0, 217/255.0),
    ColorF(197/255.0, 72/255.0, 64/255.0),
    ColorF(126/255.0, 187/255.0, 113/255.0),
    ColorF(213/255.0, 150/255.0, 55/255.0),
    ColorF(126/255.0, 187/255.0, 113/255.0)
};



 static const std::vector<std::string> FaceKeypointLabels = {"jaw_0",
                                                 "jaw_1", 
                                                 "jaw_2",
                                                 "jaw_3",
                                                 "jaw_4", 
                                                 "jaw_5",
                                                 "jaw_7",
                                                 "jaw_8",
                                                 "jaw_9",
                                                 "jaw_10",
                                                 "jaw_11",
                                                 "jaw_12",
                                                 "jaw_13", 
                                                 "jaw_14", 
                                                 "jaw_15",
                                                 "right_eye_brow_0",
                                                 "right_eye_brow_1", 
                                                 "right_eye_brow_2",
                                                 "right_eye_brow_3", 
                                                 "right_eye_brow_4",
                                                 "left_eye_brow_0", 
                                                 "left_eye_brow_1",
                                                 "left_eye_brow_2", 
                                                 "left_eye_brow_3",
                                                 "left_eye_brow_4",
                                                 "nose_0",
                                                 "nose_1",
                                                 "nose_2",
                                                 "nose_3",
                                                 "nose_4",
                                                 "nose_5",
                                                 "nose_6",
                                                 "nose_7",
                                                 "nose_8",
                                                 "right_eye_0",
                                                 "right_eye_1",
                                                 "right_eye_2",
                                                 "right_eye_3",
                                                 "right_eye_4",
                                                 "right_eye_5",
                                                 "left_eye_0",
                                                 "left_eye_1",
                                                 "left_eye_2", 
                                                 "left_eye_3",
                                                 "left_eye_4",
                                                 "left_eye_5",
                                                 "outer_lip_0",
                                                 "outer_lip_1",
                                                 "outer_lip_2",
                                                 "outer_lip_3",
                                                 "outer_lip_4",
                                                 "outer_lip_5",
                                                 "outer_lip_6",
                                                 "outer_lip_7",
                                                 "outer_lip_8",
                                                 "outer_lip_9",
                                                 "outer_lip_10",
                                                 "outer_lip_11",
                                                 "outer_lip_12",
                                                 "outer_lip_13",
                                                 "inner_lip_0",
                                                 "inner_lip_1",
                                                 "inner_lip_2",
                                                 "inner_lip_3",
                                                 "inner_lip_4",
                                                 "inner_lip_5",
                                                 "inner_lip_6",
                                                 "inner_lip_7",
                                                 "right_center_eye",
                                                 "left_center_eye"};
#endif