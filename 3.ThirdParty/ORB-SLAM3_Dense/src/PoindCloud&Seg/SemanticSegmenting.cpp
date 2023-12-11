#include "SemanticSegmenting.h"

#define __FILENAME__ (std::strrchr(__FILE__, '/') ? std::strrchr(__FILE__, '/') + 1 : __FILE__)

#define THROW_ERROR(msg)                                          \
    std::cerr << "[" << __FILENAME__ << ":" << __LINE__ << "]\n"; \
    throw std::runtime_error(msg);


namespace TensorRT
{
SemanticSeger::SemanticSeger(const std::string& config_file, const std::string& model_path)
{
    cudaSetDevice(0);
    /*** Read local net file. ***/
    std::ifstream file_ptr(model_path, std::ios::binary);
    if (!file_ptr.good())
    {
        THROW_ERROR("Failed to load the trt net file!");
    }
    size_t size = 0;
    file_ptr.seekg(0, file_ptr.end);   // 将读指针从文件末尾开始移动0个字节
    size = file_ptr.tellg();           // 返回读指针的位置，此时读指针的位置就是文件的字节数
    file_ptr.seekg(0, file_ptr.beg);   // 将读指针从文件开头开始移动0个字节
    char* model_stream = new char[size];
    file_ptr.read(model_stream, size);
    file_ptr.close();

    Logger logger;   // 日志记录接口

    UniquePtr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(logger)};

    // 保存模型的模型结构、模型参数以及最优计算kernel配置；
    // 不能跨平台和跨TensorRT版本移植
    /*** Inference engine ***/
    mEngine = std::shared_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(model_stream, size), Deleter());

    // 储存中间值，实际进行推理的对象
    // 由engine创建，可创建多个对象，进行多推理任务
    mContext = UniquePtr<nvinfer1::IExecutionContext>(mEngine->createExecutionContext());
    delete[] model_stream;

    /*** Parse the config file ***/
    std::string input_tensor_name, output_tensor_name, output_type;
    cv::Mat_<double> float_array[2];
    cv::FileStorage fs_configs(config_file, cv::FileStorage::READ);
    if (!fs_configs.isOpened())
    {
        THROW_ERROR("Failed to open settings file at: " + config_file);
    }
    fs_configs["InputTensorName"] >> input_tensor_name;
    fs_configs["OutputTensorName"] >> output_tensor_name;
    fs_configs["Mean"] >> float_array[0];
    fs_configs["Std"] >> float_array[1];
    mMean = cv::Scalar(float_array[0].ptr()[0], float_array[0].ptr()[1], float_array[0].ptr()[2]);
    mStd = 3.0 / (float_array[1].ptr()[0] + float_array[1].ptr()[1] + float_array[1].ptr()[2]);
    fs_configs.release();
    /****************************/

    /*** Allocate GPU memory on input part of the buffer. ***/
    mInputTensorIdx = mEngine->getBindingIndex(input_tensor_name.c_str());
    mInputDims = mEngine->getBindingDimensions(mInputTensorIdx);
    size_t input_data_length = mInputDims.d[1] * mInputDims.d[2] * mInputDims.d[3];
    cudaMalloc(&(mCudaBuffer[mInputTensorIdx]), input_data_length * sizeof(float));

    /*** Allocate GPU memory on output part of the buffer. ***/
    mOutputTensorIdx = mEngine->getBindingIndex(output_tensor_name.c_str());
    mOutputDims = mEngine->getBindingDimensions(mOutputTensorIdx);
    mDimsOfOutput = mOutputDims.nbDims;   // Get the dimensions of output tensor
    mOutputDataSize = mOutputDims.d[mDimsOfOutput - 2] * mOutputDims.d[mDimsOfOutput - 1];

    // std::cout << mOutputDims.d[mDimsOfOutput - 2] << std::endl;
    // std::cout << mOutputDims.d[mDimsOfOutput - 1] << std::endl;

    cudaMalloc(&mCudaBuffer[mOutputTensorIdx], mOutputDataSize * sizeof(long));
}


SemanticSeger::~SemanticSeger()
{
    // cudaFree(mCudaBuffer[mParams.mInputTensorIdx]);
    // cudaFree(mCudaBuffer[mParams.mOutputTensorIdx]);
}

void SemanticSeger::run(const cv::Mat& src, cv::Mat& dst)
{
    if (src.empty())
    {
        THROW_ERROR("The input of tensorRT SemanticSeger is empty!");
    }
    doInference(src);
    postProcess(src, dst);
}

void SemanticSeger::doInference(const cv::Mat& input)
{
    /*** The preprocess of the input. ***/
    cv::Mat input_tensor;
    cv::dnn::blobFromImage(input, input_tensor, mStd, cv::Size(mInputDims.d[2], mInputDims.d[3]), mMean, true, false);
    // cv::dnn::blobFromImage(
    //     input, input_tensor, 1.0 / 58.8, cv::Size(mInputDims.d[2], mInputDims.d[3]), mMean, true, false);

    /*** Transfer the data from CPU memory to GPU memory. ***/
    cudaMemcpy(mCudaBuffer[mInputTensorIdx],
               input_tensor.ptr<float>(),
               input_tensor.total() * sizeof(float),
               cudaMemcpyHostToDevice);

    mContext->executeV2(mCudaBuffer);

    /*** Transfer the data from GPU memory to CPU memory. ***/
    long* output_array = new long[mOutputDataSize];
    cudaMemcpy(output_array, mCudaBuffer[mOutputTensorIdx], mOutputDataSize * sizeof(long), cudaMemcpyDeviceToHost);

    uint16_t raw_output_uint16[mOutputDataSize];
    for (size_t i = 0; i < mOutputDataSize; ++i)
    {
        raw_output_uint16[i] = static_cast<uint16_t>(output_array[i]);
    }

    mOutput =
        cv::Mat(mOutputDims.d[mDimsOfOutput - 2], mOutputDims.d[mDimsOfOutput - 1], CV_16U, raw_output_uint16).clone();
    delete[] output_array;
}

void SemanticSeger::postProcess(const cv::Mat& src, cv::Mat& dst)
{
    cv::Mat output =
        mOutput(cv::Rect(0, 0, mOutputDims.d[mDimsOfOutput - 2] / 2, mOutputDims.d[mDimsOfOutput - 1] / 2));

    cv::resize(output, output, cv::Size(src.cols, src.rows));


    cv::Mat white_image(src.rows, src.cols, CV_8UC3, cv::Scalar(200, 200, 200));

    for (int y = 0; y < output.rows; ++y)
    {
        for (int x = 0; x < output.cols; ++x)
        {
            if (output.ptr<uint16_t>(y)[x] == 9 || output.ptr<uint16_t>(y)[x] == 17 ||
                output.ptr<uint16_t>(y)[x] == 29 || output.ptr<uint16_t>(y)[x] == 66)
            {
                white_image.ptr<cv::Vec3b>(y)[x] = cv::Vec3b(0, 255, 0);
            }
            else if (output.ptr<uint16_t>(y)[x] == 21 || output.ptr<uint16_t>(y)[x] == 26 ||
                     output.ptr<uint16_t>(y)[x] == 27 || output.ptr<uint16_t>(y)[x] == 60 ||
                     output.ptr<uint16_t>(y)[x] == 109 || output.ptr<uint16_t>(y)[x] == 128)
            {
                white_image.ptr<cv::Vec3b>(y)[x] = cv::Vec3b(255, 0, 0);
            }
        }
    }

    dst = white_image.clone();
}
}   // namespace TensorRT
