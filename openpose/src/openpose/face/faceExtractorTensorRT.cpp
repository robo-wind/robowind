#if defined USE_CAFFE && defined USE_CUDA
    #include <caffe/blob.hpp>
#endif
#include <opencv2/opencv.hpp> // CV_WARP_INVERSE_MAP, CV_INTER_LINEAR
#include <openpose/net/maximumCaffe.hpp>
#include <openpose/core/netTensorRT.hpp>
#include <openpose/net/resizeAndMergeCaffe.hpp>
#include <openpose/face/faceParameters.hpp>
#include <openpose/gpu/cuda.hpp>
#include <openpose/utilities/fastMath.hpp>
#include <openpose/utilities/openCv.hpp>
#include <openpose/face/faceExtractorTensorRT.hpp>

namespace op
{
    struct FaceExtractorTensorRT::ImplFaceExtractorTensorRT
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

            ImplFaceExtractorTensorRT(const std::string& modelFolder, const std::string& tensorModel, const int gpuId, const bool enableGoogleLogging) :
                netInitialized{false},
                spNetTensorRT{std::make_shared<NetTensorRT>(modelFolder + FACE_PROTOTXT + tensorModel,
                                                            modelFolder + FACE_TRAINED_MODEL,
                                                            gpuId, enableGoogleLogging)},
                spResizeAndMergeCaffe{std::make_shared<ResizeAndMergeCaffe<float>>()},
                spMaximumCaffe{std::make_shared<MaximumCaffe<float>>()}
            {
            }
        #endif
    };

    #if defined USE_CAFFE && defined USE_CUDA
        // See faceExtractorCaffe.cpp for impl
        void updateFaceHeatMapsForPerson(Array<float>& heatMaps, const int person, const ScaleMode heatMapScaleMode,
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
                                             FACE_CCN_DECREASE_FACTOR, 1.f, mergeFirstDimension);
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

    FaceExtractorTensorRT::FaceExtractorTensorRT(const Point<int>& netInputSize, const Point<int>& netOutputSize,
                                           const std::string& modelFolder, const std::string& tensorModel, const int gpuId,
                                           const std::vector<HeatMapType>& heatMapTypes,
                                           const ScaleMode heatMapScale, const bool enableGoogleLogging) :
        FaceExtractorNet{netInputSize, netOutputSize, heatMapTypes, heatMapScale}
        #if defined USE_TENSORRT
        , upImpl{new ImplFaceExtractorTensorRT{modelFolder, tensorModel, gpuId, enableGoogleLogging}}
        #endif
    {
        try
        {
            #if !defined USE_TENSORRT
                UNUSED(netInputSize);
                UNUSED(netOutputSize);
                UNUSED(modelFolder);
                UNUSED(gpuId);
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

    FaceExtractorTensorRT::~FaceExtractorTensorRT()
    {
    }

    void FaceExtractorTensorRT::netInitializationOnThread()
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

    void FaceExtractorTensorRT::forwardPass(const std::vector<Rectangle<float>>& faceRectangles,
                                         const cv::Mat& cvInputData)//,
                                         //const double scaleInputToOutput)
    {
        try
        {
            const double scaleInputToOutput = 1;
            #if defined USE_TENSORRT
                if (!faceRectangles.empty())
                {
                    // Security checks
                    if (cvInputData.empty())
                        error("Empty cvInputData.", __LINE__, __FUNCTION__, __FILE__);

                    // Fix parameters
                    const auto netInputSide = fastMin(mNetOutputSize.x, mNetOutputSize.y);

                    // Set face size
                    const auto numberPeople = (int)faceRectangles.size();
                    mFaceKeypoints.reset({numberPeople, (int)FACE_NUMBER_PARTS, 3}, 0);

                    // HeatMaps: define size
                    if (!mHeatMapTypes.empty())
                        mHeatMaps.reset({numberPeople, (int)FACE_NUMBER_PARTS, mNetOutputSize.y, mNetOutputSize.x});

                    // // Debugging
                    // cv::Mat cvInputDataCopy = cvInputData.clone();
                    // Extract face keypoints for each person
                    for (auto person = 0 ; person < numberPeople ; person++)
                    {
                        const auto& faceRectangle = faceRectangles.at(person);
                        // Only consider faces with a minimum pixel area
                        const auto minFaceSize = fastMin(faceRectangle.width, faceRectangle.height);
                        // // Debugging -> red rectangle
                        // log(std::to_string(cvInputData.cols) + " " + std::to_string(cvInputData.rows));
                        // cv::rectangle(cvInputDataCopy,
                        //               cv::Point{(int)faceRectangle.x, (int)faceRectangle.y},
                        //               cv::Point{(int)faceRectangle.bottomRight().x,
                        //                         (int)faceRectangle.bottomRight().y},
                        //               cv::Scalar{0,0,255}, 2);
                        // Get parts
                        if (minFaceSize > 40)
                        {
                            // // Debugging -> green rectangle overwriting red one
                            // log(std::to_string(cvInputData.cols) + " " + std::to_string(cvInputData.rows));
                            // cv::rectangle(cvInputDataCopy,
                            //               cv::Point{(int)faceRectangle.x, (int)faceRectangle.y},
                            //               cv::Point{(int)faceRectangle.bottomRight().x,
                            //                         (int)faceRectangle.bottomRight().y},
                            //               cv::Scalar{0,255,0}, 2);
                            // Resize and shift image to face rectangle positions
                            const auto faceSize = fastMax(faceRectangle.width, faceRectangle.height);
                            const double scaleFace = faceSize / (double)netInputSide;
                            cv::Mat Mscaling = cv::Mat::eye(2, 3, CV_64F);
                            Mscaling.at<double>(0,0) = scaleFace;
                            Mscaling.at<double>(1,1) = scaleFace;
                            Mscaling.at<double>(0,2) = faceRectangle.x;
                            Mscaling.at<double>(1,2) = faceRectangle.y;

                            cv::Mat faceImage;
                            cv::warpAffine(cvInputData, faceImage, Mscaling,
                                           cv::Size{mNetOutputSize.x, mNetOutputSize.y},
                                           CV_INTER_LINEAR | CV_WARP_INVERSE_MAP,
                                           cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

                            // cv::Mat -> float*
                            uCharCvMatToFloatPtr(mFaceImageCrop.getPtr(), faceImage, true);

                            // // Debugging
                            // if (person < 5)
                            // cv::imshow("faceImage" + std::to_string(person), faceImage);

                            // 1. TensorRT deep network
                            upImpl->spNetTensorRT->forwardPass(mFaceImageCrop);

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
                                upImpl->spMaximumCaffe->Forward_gpu({upImpl->spHeatMapsBlob.get()},
                                                                    {upImpl->spPeaksBlob.get()});
                                cudaCheck(__LINE__, __FUNCTION__, __FILE__);
                            #else
                                upImpl->spMaximumCaffe->Forward_cpu({upImpl->spHeatMapsBlob.get()},
                                                                    {upImpl->spPeaksBlob.get()});
                            #endif

                            const auto* facePeaksPtr = upImpl->spPeaksBlob->mutable_cpu_data();
                            for (auto part = 0 ; part < mFaceKeypoints.getSize(1) ; part++)
                            {
                                const auto xyIndex = part * mFaceKeypoints.getSize(2);
                                const auto x = facePeaksPtr[xyIndex];
                                const auto y = facePeaksPtr[xyIndex + 1];
                                const auto score = facePeaksPtr[xyIndex + 2];
                                const auto baseIndex = mFaceKeypoints.getSize(2)
                                                     * (part + person * mFaceKeypoints.getSize(1));
                                mFaceKeypoints[baseIndex] = (float)(scaleInputToOutput
                                                                    * (Mscaling.at<double>(0,0) * x
                                                                       + Mscaling.at<double>(0,1) * y
                                                                       + Mscaling.at<double>(0,2)));
                                mFaceKeypoints[baseIndex+1] = (float)(scaleInputToOutput
                                                                      * (Mscaling.at<double>(1,0) * x
                                                                         + Mscaling.at<double>(1,1) * y
                                                                         + Mscaling.at<double>(1,2)));
                                mFaceKeypoints[baseIndex+2] = score;
                            }
                            // HeatMaps: storing
                            if (!mHeatMapTypes.empty())
                                updateFaceHeatMapsForPerson(mHeatMaps, person, mHeatMapScaleMode,
                                                            upImpl->spHeatMapsBlob->gpu_data());
                        }
                    }
                    // // Debugging
                    // cv::imshow("AcvInputDataCopy", cvInputDataCopy);
                }
                else
                    mFaceKeypoints.reset();
            #else
                UNUSED(faceRectangles);
                UNUSED(cvInputData);
                UNUSED(scaleInputToOutput);
            #endif
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
