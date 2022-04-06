#if defined USE_CAFFE && defined USE_CUDA
    #include <caffe/blob.hpp>
#endif
#include <openpose/core/netTensorRT.hpp>
#include <opencv2/opencv.hpp> // CV_WARP_INVERSE_MAP, CV_INTER_LINEAR
#include <openpose/net/maximumCaffe.hpp>
#include <openpose/net/resizeAndMergeCaffe.hpp>
#include <openpose/hand/handParameters.hpp>
#include <openpose/gpu/cuda.hpp>
#include <openpose/utilities/fastMath.hpp>
#include <openpose/utilities/keypoint.hpp>
#include <openpose/utilities/openCv.hpp>
#include <openpose/hand/handExtractorTensorRT.hpp>
#include <openpose/hand/handExtractorCaffe.hpp>

namespace op
{
    struct HandExtractorTensorRT::ImplHandExtractorTensorRT
    {
        #if defined USE_TENSORRT
            bool netInitialized;
            std::shared_ptr<NetTensorRT> spNetTensorRT;
            std::shared_ptr<ResizeAndMergeCaffe<float>> spResizeAndMergeCaffe;
            std::shared_ptr<MaximumCaffe<float>> spMaximumCaffe;
            // Init with thread
            boost::shared_ptr<caffe::Blob<float>> spTensorRTNetOutputBlob;
            std::shared_ptr<caffe::Blob<float>> spHeatMapsBlob;
            std::shared_ptr<caffe::Blob<float>> spPeaksBlob;

            ImplHandExtractorTensorRT(const std::string& modelFolder, const std::string& tensorModel, const int gpuId,
                                   const bool enableGoogleLogging) :
                netInitialized{false},
                spNetTensorRT{std::make_shared<NetTensorRT>(modelFolder + HAND_PROTOTXT + tensorModel,
                                                            modelFolder + HAND_TRAINED_MODEL,
                                                            gpuId, enableGoogleLogging)},
                spResizeAndMergeCaffe{std::make_shared<ResizeAndMergeCaffe<float>>()},
                spMaximumCaffe{std::make_shared<MaximumCaffe<float>>()}
            {
            }
        #endif
    };

    #if defined USE_CAFFE
        // See handExtractorCaffe.cpp for impl
        void cropFrame(Array<float>& handImageCrop, cv::Mat& affineMatrix, const cv::Mat& cvInputData,
                       const Rectangle<float>& handRectangle, const int netInputSide,
                       const Point<int>& netOutputSize, const bool mirrorImage);

        // See handExtractorCaffe.cpp for impl
        //void connectKeypoints(Array<float>& handCurrent, const double scaleInputToOutput, const int person,
        //                      const cv::Mat& affineMatrix, const float* handPeaks);
        void connectKeypoints(Array<float>& handCurrent, const int person,
                              const cv::Mat& affineMatrix, const float* handPeaks);

        // See handExtractorCaffe.cpp for impl
        Rectangle<float> getHandRectangle(Array<float>& handCurrent, const int person, const float increaseRatio,
                                          const int handNumberParts, const float thresholdRectangle,
                                          const Rectangle<float>& previousHandRectangle = Rectangle<float>{});

        // See handExtractorCaffe.cpp for impl
        void updateHandHeatMapsForPerson(Array<float>& heatMaps, const int person, const ScaleMode heatMapScaleMode,
                                         const float* heatMapsGpuPtr);
        
        inline void reshapeFaceExtractorCaffe(std::shared_ptr<ResizeAndMergeCaffe<float>>& resizeAndMergeCaffe,
                                              std::shared_ptr<MaximumCaffe<float>>& maximumCaffe,
                                              boost::shared_ptr<caffe::Blob<float>>& caffeNetOutputBlob,
                                              std::shared_ptr<caffe::Blob<float>>& heatMapsBlob,
                                              std::shared_ptr<caffe::Blob<float>>& peaksBlob)
        {
            try
            {
                // HeatMaps extractor blob and layer
                const bool mergeFirstDimension = true;
                resizeAndMergeCaffe->Reshape({caffeNetOutputBlob.get()}, {heatMapsBlob.get()},
                                             HAND_CCN_DECREASE_FACTOR, 1.f, mergeFirstDimension);
                // Pose extractor blob and layer
                maximumCaffe->Reshape({heatMapsBlob.get()}, {peaksBlob.get()});
                // Cuda check
                cudaCheck(__LINE__, __FUNCTION__, __FILE__);
            }
            catch (const std::exception& e)
            {
                error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            }
        }
    #endif

    HandExtractorTensorRT::HandExtractorTensorRT(const Point<int>& netInputSize, const Point<int>& netOutputSize,
                                           const std::string& modelFolder, const std::string& tensorModel, const int gpuId,
                                           const unsigned short numberScales,
                                           const float rangeScales, const std::vector<HeatMapType>& heatMapTypes,
                                           const ScaleMode heatMapScale,
                                           const bool enableGoogleLogging) :
        HandExtractorNet{netInputSize, netOutputSize, numberScales, rangeScales, heatMapTypes, heatMapScale}
        #if defined USE_TENSORRT
        , upImpl{new ImplHandExtractorTensorRT{modelFolder, tensorModel, gpuId, enableGoogleLogging}}
        #endif
    {
        try
        {
            #if !defined USE_TENSORRT
                UNUSED(netInputSize);
                UNUSED(netOutputSize);
                UNUSED(modelFolder);
                UNUSED(gpuId);
                UNUSED(numberScales);
                UNUSED(rangeScales);
                UNUSED(heatMapTypes);
                UNUSED(heatMapScale);
                error("OpenPose must be compiled with the `USE_CAFFE` & `USE_CUDA` macro definitions in order to run"
                      " this functionality.", __LINE__, __FUNCTION__, __FILE__);
            #endif
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    HandExtractorTensorRT::~HandExtractorTensorRT()
    {
    }

    void HandExtractorTensorRT::netInitializationOnThread()
    {
        try
        {
            #if defined USE_TENSORRT
                // Logging
                log("Starting initialization on thread.", Priority::Low, __LINE__, __FUNCTION__, __FILE__);
                // Initialize TensorRT net
                upImpl->spNetTensorRT->initializationOnThread();
                cudaCheck(__LINE__, __FUNCTION__, __FILE__);
                // Initialize blobs
                upImpl->spTensorRTNetOutputBlob = upImpl->spNetTensorRT->getOutputBlob();
                upImpl->spHeatMapsBlob = {std::make_shared<caffe::Blob<float>>(1,1,1,1)};
                upImpl->spPeaksBlob = {std::make_shared<caffe::Blob<float>>(1,1,1,1)};
                cudaCheck(__LINE__, __FUNCTION__, __FILE__);
                // Logging
                log("Finished initialization on thread.", Priority::Low, __LINE__, __FUNCTION__, __FILE__);
            #endif
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void HandExtractorTensorRT::forwardPass(const std::vector<std::array<Rectangle<float>, 2>> handRectangles,
                                         const cv::Mat& cvInputData)//,
                                         //const double scaleInputToOutput)
    {
        try
        {
            const double scaleInputToOutput = 1;
            #if defined USE_TENSORRT
                if (!handRectangles.empty())
                {
                    // Security checks
                    if (cvInputData.empty())
                        error("Empty cvInputData.", __LINE__, __FUNCTION__, __FILE__);

                    // Fix parameters
                    const auto netInputSide = fastMin(mNetOutputSize.x, mNetOutputSize.y);

                    // Set hand size
                    const auto numberPeople = (int)handRectangles.size();
                    mHandKeypoints[0].reset({numberPeople, (int)HAND_NUMBER_PARTS, 3}, 0);
                    mHandKeypoints[1].reset(mHandKeypoints[0].getSize(), 0);

                    // HeatMaps: define size
                    if (!mHeatMapTypes.empty())
                    {
                        mHeatMaps[0].reset({numberPeople, (int)HAND_NUMBER_PARTS, mNetOutputSize.y, mNetOutputSize.x});
                        mHeatMaps[1].reset({numberPeople, (int)HAND_NUMBER_PARTS, mNetOutputSize.y, mNetOutputSize.x});
                    }

                    // // Debugging
                    // cv::Mat cvInputDataCopied = cvInputData.clone();
                    // Extract hand keypoints for each person
                    for (auto hand = 0 ; hand < 2 ; hand++)
                    {
                        // Parameters
                        auto& handCurrent = mHandKeypoints[hand];
                        const bool mirrorImage = (hand == 0);
                        for (auto person = 0 ; person < numberPeople ; person++)
                        {
                            const auto& handRectangle = handRectangles.at(person).at(hand);
                            // Only consider faces with a minimum pixel area
                            const auto minHandSize = fastMin(handRectangle.width, handRectangle.height);
                            // // Debugging -> red rectangle
                            // if (handRectangle.width > 0)
                            //     cv::rectangle(cvInputDataCopied,
                            //                   cv::Point{intRound(handRectangle.x), intRound(handRectangle.y)},
                            //                   cv::Point{intRound(handRectangle.x + handRectangle.width),
                            //                             intRound(handRectangle.y + handRectangle.height)},
                            //                   cv::Scalar{(hand * 255.f),0.f,255.f}, 2);
                            // Get parts
                            if (minHandSize > 1 && handRectangle.area() > 10)
                            {
                                // Single-scale detection
                                if (mMultiScaleNumberAndRange.first == 1)
                                {
                                    // // Debugging -> green rectangle overwriting red one
                                    // if (handRectangle.width > 0)
                                    //     cv::rectangle(cvInputDataCopied,
                                    //                   cv::Point{intRound(handRectangle.x),
                                    //                             intRound(handRectangle.y)},
                                    //                   cv::Point{intRound(handRectangle.x + handRectangle.width),
                                    //                             intRound(handRectangle.y + handRectangle.height)},
                                    //                   cv::Scalar{(hand * 255.f),255.f,0.f}, 2);
                                    // Parameters
                                    cv::Mat affineMatrix;
                                    // Resize image to hands positions + cv::Mat -> float*
                                    cropFrame(mHandImageCrop, affineMatrix, cvInputData, handRectangle, netInputSide,
                                              mNetOutputSize, mirrorImage);
                                    // Deep net + Estimate keypoint locations
                                    detectHandKeypoints(handCurrent, scaleInputToOutput, person, affineMatrix);
                                }
                                // Multi-scale detection
                                else
                                {
                                    const auto handPtrArea = handCurrent.getSize(1) * handCurrent.getSize(2);
                                    auto* handCurrentPtr = handCurrent.getPtr() + person * handPtrArea;
                                    const auto numberScales = mMultiScaleNumberAndRange.first;
                                    const auto initScale = 1.f - mMultiScaleNumberAndRange.second / 2.f;
                                    for (auto i = 0 ; i < numberScales ; i++)
                                    {
                                        // Get current scale
                                        const auto scale = initScale
                                                         + mMultiScaleNumberAndRange.second * i / (numberScales-1.f);
                                        // Process hand
                                        Array<float> handEstimated({1, handCurrent.getSize(1),
                                                                    handCurrent.getSize(2)}, 0);
                                        const auto handRectangleScale = recenter(
                                            handRectangle,
                                            (float)(intRound(handRectangle.width * scale) / 2 * 2),
                                            (float)(intRound(handRectangle.height * scale) / 2 * 2)
                                        );
                                        // // Debugging -> blue rectangle
                                        // cv::rectangle(cvInputDataCopied,
                                        //               cv::Point{intRound(handRectangleScale.x),
                                        //                         intRound(handRectangleScale.y)},
                                        //               cv::Point{intRound(handRectangleScale.x
                                        //                                  + handRectangleScale.width),
                                        //                         intRound(handRectangleScale.y
                                        //                                  + handRectangleScale.height)},
                                        //               cv::Scalar{255,0,0}, 2);
                                        // Parameters
                                        cv::Mat affineMatrix;
                                        // Resize image to hands positions + cv::Mat -> float*
                                        cropFrame(mHandImageCrop, affineMatrix, cvInputData, handRectangleScale,
                                                  netInputSide, mNetOutputSize, mirrorImage);
                                        // Deep net + Estimate keypoint locations
                                        detectHandKeypoints(handEstimated, scaleInputToOutput, 0, affineMatrix);
                                        if (i == 0
                                            || getAverageScore(handEstimated,0) > getAverageScore(handCurrent,person))
                                            std::copy(handEstimated.getConstPtr(),
                                                      handEstimated.getConstPtr() + handPtrArea, handCurrentPtr);
                                    }
                                }
                                // HeatMaps: storing
                                if (!mHeatMapTypes.empty())
                                    updateHandHeatMapsForPerson(mHeatMaps[hand], person, mHeatMapScaleMode,
                                                                upImpl->spHeatMapsBlob->gpu_data());
                            }
                        }
                    }
                    // // Debugging
                    // cv::imshow("cvInputDataCopied", cvInputDataCopied);
                }
                else
                {
                    mHandKeypoints[0].reset();
                    mHandKeypoints[1].reset();
                }
            #else
                UNUSED(handRectangles);
                UNUSED(cvInputData);
                UNUSED(scaleInputToOutput);
            #endif
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void HandExtractorTensorRT::detectHandKeypoints(Array<float>& handCurrent, const double scaleInputToOutput,
                                                 const int person, const cv::Mat& affineMatrix)
    {
        try
        {
            #if defined USE_TENSORRT
                // 1. Deep net
                upImpl->spNetTensorRT->forwardPass(mHandImageCrop);

                // Reshape blobs
                if (!upImpl->netInitialized)
                {
                    upImpl->netInitialized = true;
                    reshapeFaceExtractorCaffe(upImpl->spResizeAndMergeCaffe, upImpl->spMaximumCaffe,
                                              upImpl->spTensorRTNetOutputBlob, upImpl->spHeatMapsBlob,
                                              upImpl->spPeaksBlob);
                }

                // 2. Resize heat maps + merge different scales
                #ifdef USE_CUDA
                    upImpl->spResizeAndMergeCaffe->Forward_gpu({upImpl->spTensorRTNetOutputBlob.get()},
                                                               {upImpl->spHeatMapsBlob.get()});
                    cudaCheck(__LINE__, __FUNCTION__, __FILE__);
                #else
                    upImpl->spResizeAndMergeCaffe->Forward_cpu({upImpl->spTensorRTNetOutputBlob.get()},
                                                               {upImpl->spHeatMapsBlob.get()});
                #endif

                // 3. Get peaks by Non-Maximum Suppression
                #ifdef USE_CUDA
                    upImpl->spMaximumCaffe->Forward_gpu({upImpl->spHeatMapsBlob.get()}, {upImpl->spPeaksBlob.get()});
                    cudaCheck(__LINE__, __FUNCTION__, __FILE__);
                #else
                    upImpl->spMaximumCaffe->Forward_cpu({upImpl->spHeatMapsBlob.get()}, {upImpl->spPeaksBlob.get()});
                #endif

                // Estimate keypoint locations
                //connectKeypoints(handCurrent, scaleInputToOutput, person, affineMatrix,
                //                 upImpl->spPeaksBlob->mutable_cpu_data());
                //original function does not scale correctly
                connectKeypoints(handCurrent, person, affineMatrix,
                                 upImpl->spPeaksBlob->mutable_cpu_data());
            #else
                UNUSED(handCurrent);
                UNUSED(scaleInputToOutput);
                UNUSED(person);
                UNUSED(affineMatrix);
            #endif
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
