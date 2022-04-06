#ifndef OPENPOSE_FACE_FACE_EXTRACTOR_TENSORRT_HPP
#define OPENPOSE_FACE_FACE_EXTRACTOR_TENSORRT_HPP

#include <opencv2/core/core.hpp> // cv::Mat
#include <openpose/core/common.hpp>
#include <openpose/core/enumClasses.hpp>
#include <openpose/face/faceExtractorNet.hpp>

namespace op
{
    /**
     * Face keypoint extractor class for TensorRT framework.
     */
    class OP_API FaceExtractorTensorRT : public FaceExtractorNet
    {
    public:
        /**
         * Constructor of the FaceExtractor class.
         * @param netInputSize Size at which the cropped image (where the face is located) is resized.
         * @param netOutputSize Size of the final results. At the moment, it must be equal than netOutputSize.
         */
        FaceExtractorTensorRT(const Point<int>& netInputSize, const Point<int>& netOutputSize,
                           const std::string& modelFolder, const std::string& tensorModel, const int gpuId,
                           const std::vector<HeatMapType>& heatMapTypes = {},
                           const ScaleMode heatMapScale = ScaleMode::ZeroToOne,
                           const bool enableGoogleLogging = false);

        virtual ~FaceExtractorTensorRT();

        /**
         * This function must be call before using any other function. It must also be called inside the thread in
         * which the functions are going to be used.
         */
        void netInitializationOnThread();

        /**
         * This function extracts the face keypoints for each detected face in the image.
         * @param faceRectangles location of the faces in the image. It is a length-variable std::vector, where
         * each index corresponds to a different person in the image. Internally, a op::Rectangle<float>
         * (similar to cv::Rect for floating values) with the position of that face (or 0,0,0,0 if
         * some face is missing, e.g. if a specific person has only half of the body inside the image).
         * @param cvInputData Original image in cv::Mat format and BGR format.
         * @param scaleInputToOutput Desired scale of the final keypoints. Set to 1 if the desired size is the
         * cvInputData size.
         */
        void forwardPass(const std::vector<Rectangle<float>>& faceRectangles, const cv::Mat& cvInputData);//,
                         //const double scaleInputToOutput);

    private:
        // PIMPL idiom
        // http://www.cppsamples.com/common-tasks/pimpl.html
        struct ImplFaceExtractorTensorRT;
        std::unique_ptr<ImplFaceExtractorTensorRT> upImpl;

        // PIMP requires DELETE_COPY & destructor, or extra code
        // http://oliora.github.io/2015/12/29/pimpl-and-rule-of-zero.html
        DELETE_COPY(FaceExtractorTensorRT);
    };
}

#endif // OPENPOSE_FACE_FACE_EXTRACTOR_TENSORRT_HPP
