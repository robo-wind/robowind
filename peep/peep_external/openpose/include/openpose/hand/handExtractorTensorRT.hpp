#ifndef OPENPOSE_HAND_HAND_EXTRACTOR_TENSORRT_HPP
#define OPENPOSE_HAND_HAND_EXTRACTOR_TENSORRT_HPP

#include <opencv2/core/core.hpp> // cv::Mat
#include <openpose/core/common.hpp>
#include <openpose/core/enumClasses.hpp>
#include <openpose/hand/handExtractorNet.hpp>

namespace op
{
    /**
     * Hand keypoint extractor class for TensorRT framework.
     */
    class OP_API HandExtractorTensorRT : public HandExtractorNet
    {
    public:
        /**
         * Constructor of the HandExtractorTensorRT class.
         * @param netInputSize Size at which the cropped image (where the hand is located) is resized.
         * @param netOutputSize Size of the final results. At the moment, it must be equal than netOutputSize.
         * @param modelFolder Folder where the models are located.
         * @param gpuId The GPU index (0-based) which the deep net will use.
         * @param numberScales Number of scales to run. The more scales, the slower it will be but possibly also more
         * accurate.
         * @param rangeScales The range between the smaller and bigger scale.
         */
        HandExtractorTensorRT(const Point<int>& netInputSize, const Point<int>& netOutputSize,
                           const std::string& modelFolder, const std::string& tensorModel, const int gpuId,
                           const unsigned short numberScales = 1, const float rangeScales = 0.4f,
                           const std::vector<HeatMapType>& heatMapTypes = {},
                           const ScaleMode heatMapScale = ScaleMode::ZeroToOne,
                           const bool enableGoogleLogging = false);

        /**
         * Virtual destructor of the HandExtractor class.
         * Required to allow inheritance.
         */
        virtual ~HandExtractorTensorRT();

        /**
         * This function must be call before using any other function. It must also be called inside the thread in
         * which the functions are going to be used.
         */
        void netInitializationOnThread();

        /**
         * This function extracts the hand keypoints for each detected hand in the image.
         * @param handRectangles location of the hands in the image. It is a length-variable std::vector, where
         * each index corresponds to a different person in the image. Internally the std::vector, a std::array of 2
         * elements: index 0 and 1 for left and right hand respectively. Inside each array element, a
         * op::Rectangle<float> (similar to cv::Rect for floating values) with the position of that hand (or 0,0,0,0 if
         * some hand is missing, e.g. if a specific person has only half of the body inside the image).
         * @param cvInputData Original image in cv::Mat format and BGR format.
         * @param scaleInputToOutput Desired scale of the final keypoints. Set to 1 if the desired size is the
         * cvInputData size.
         */
        void forwardPass(const std::vector<std::array<Rectangle<float>, 2>> handRectangles, const cv::Mat& cvInputData);//,
                         //const double scaleInputToOutput);

    private:
        // PIMPL idiom
        // http://www.cppsamples.com/common-tasks/pimpl.html
        struct ImplHandExtractorTensorRT;
        std::unique_ptr<ImplHandExtractorTensorRT> upImpl;

        void detectHandKeypoints(Array<float>& handCurrent, const double scaleInputToOutput, const int person,
                                 const cv::Mat& affineMatrix);

        Array<float> getHeatMapsFromLastPass() const;

        // PIMP requires DELETE_COPY & destructor, or extra code
        // http://oliora.github.io/2015/12/29/pimpl-and-rule-of-zero.html
        DELETE_COPY(HandExtractorTensorRT);
    };
}

#endif // OPENPOSE_HAND_HAND_EXTRACTOR_CAFFE_HPP
