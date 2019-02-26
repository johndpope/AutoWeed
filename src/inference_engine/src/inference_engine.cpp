#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unordered_map>
#include <cassert>
#include <vector>
#include <NvInfer.h>
#include <NvUffParser.h>
#include <NvUtils.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

#include "ros/ros.h"
#include <ros/console.h>
#include <ros/package.h>
#include "custom_messages/ImageInfo.h"
#include "custom_messages/TargetInfo.h"

using namespace nvuffparser;
using namespace nvinfer1;
using namespace std;


char resourceDirectory[256];
static const int INPUT_C = 3;
static const int INPUT_H = 224;
static const int INPUT_W = 224;
static const int NB_BINDINGS = 2;
static const int BATCH_SIZE = 4;
char trtFilename[32] = "resnet.trt";
char uffFilename[32] = "resnet.uff"; // UFF file generated using TensorRT's Python API
char timesFilename[32] = "times.csv";
char labelsFilename[32] = "labels.csv";
bool recieve_flags[] = { false, false, false, false };
ros::Time timestamps[4];
int cameraIDs[4];
int frameCounts[4];
vector<void*> buffers(NB_BINDINGS);
float* batch = new float[BATCH_SIZE * INPUT_C * INPUT_H * INPUT_W];

#define MAX_WORKSPACE (1 << 30)

#define CHECK(status)                             \
    do                                            \
    {                                             \
        auto ret = (status);                      \
        if (ret != 0)                             \
        {                                         \
            std::cout << "Cuda failure: " << ret; \
            abort();                              \
        }                                         \
    } while (0)


// Logger for TensorRT info/warning/errors
class Logger : public ILogger
{
public:

	Logger() : Logger(Severity::kWARNING) {}

	Logger(Severity severity) : reportableSeverity(severity) {}

	void log(Severity severity, const char* msg) override
	{
		// suppress messages with severity enum value greater than the reportable
		if (severity > reportableSeverity) return;

		switch (severity)
		{
		case Severity::kINTERNAL_ERROR: std::cerr << "INTERNAL_ERROR: "; break;
		case Severity::kERROR: std::cerr << "ERROR: "; break;
		case Severity::kWARNING: std::cerr << "WARNING: "; break;
		case Severity::kINFO: std::cerr << "INFO: "; break;
		default: std::cerr << "UNKNOWN: "; break;
		}
		std::cerr << msg << std::endl;
	}

	Severity reportableSeverity{ Severity::kWARNING };
} gLogger;


inline int64_t volume(const Dims& d)
{
	int64_t v = 1;
	for (int64_t i = 0; i < d.nbDims; i++)
		v *= d.d[i];
	return v;
}


inline unsigned int elementSize(DataType t)
{
	switch (t)
	{
	case DataType::kINT32:
		// Fallthrough, same as kFLOAT
	case DataType::kFLOAT: return 4;
	case DataType::kHALF: return 2;
	case DataType::kINT8: return 1;
	}
	assert(0);
	return 0;
}


void gatherCallback(const custom_messages::ImageInfoConstPtr& msg) {
	ros::Time timestamp = msg->header.stamp;
	int cameraID = msg->cameraID;
	frameCounts[cameraID] = msg->frameCount;

	// Flag reception of frame from specific camera id
	recieve_flags[cameraID] = true;

	// Store timestamps for later
	timestamps[cameraID] = timestamp;

	// Convert bytes to floats and store in specific location of batch
	int elementsPerImage = INPUT_C * INPUT_H * INPUT_W;
	for (int i = 0; i < elementsPerImage; i++)
		batch[elementsPerImage * cameraID + i] = float(msg->data[i]) / 255.0;

	ROS_INFO("Received pre-processed image from camera %d and added to CUDA batch", cameraID);
}


void* safeCudaMalloc(size_t memSize)
{
	void* deviceMem;
	CHECK(cudaMalloc(&deviceMem, memSize));
	if (deviceMem == nullptr)
	{
		std::cerr << "Out of memory" << std::endl;
		exit(1);
	}
	return deviceMem;
}


std::vector<std::pair<int64_t, DataType>> calculateBindingBufferSizes(const ICudaEngine& engine, int nbBindings, int batchSize)
{
	std::vector<std::pair<int64_t, DataType>> sizes;
	for (int i = 0; i < nbBindings; ++i)
	{
		Dims dims = engine.getBindingDimensions(i);
		DataType dtype = engine.getBindingDataType(i);

		int64_t eltCount = volume(dims) * batchSize;
		sizes.push_back(std::make_pair(eltCount, dtype));
	}

	return sizes;
}


void* prepareBatchForCuda(int64_t eltCount, DataType dtype)
{
	int elementCount = BATCH_SIZE * INPUT_C * INPUT_H * INPUT_W;
	size_t count = elementCount * elementSize(dtype);
	void* deviceMemory = safeCudaMalloc(count);
	CHECK(cudaMemcpy(deviceMemory, batch, count, cudaMemcpyHostToDevice));
	return deviceMemory;
}


ICudaEngine* loadModelAndCreateEngine(const char* uffFile, int maxBatchSize, IUffParser* parser)
{
	IBuilder* builder = createInferBuilder(gLogger);
	INetworkDefinition* network = builder->createNetwork();

	if (!parser->parse(uffFile, *network, nvinfer1::DataType::kFLOAT))
		ROS_ERROR("Failed to parse UFF file");

	/* we create the engine */
	builder->setMaxBatchSize(maxBatchSize);
	builder->setMaxWorkspaceSize(MAX_WORKSPACE);

	ICudaEngine* engine = builder->buildCudaEngine(*network);
	if (!engine)
		ROS_ERROR("Unable to create TRT engine");

	/* we can clean the network and the parser */
	network->destroy();
	builder->destroy();

	return engine;
}

ICudaEngine* loadEngine(const char* filename)
{
	std::stringstream trtModelStream;
	trtModelStream.seekg(0, trtModelStream.beg);
	std::ifstream cache(filename);
	trtModelStream << cache.rdbuf();
	cache.close();
	IRuntime* runtime = createInferRuntime(gLogger);
	trtModelStream.seekg(0, std::ios::end);
	const int modelSize = trtModelStream.tellg();
	trtModelStream.seekg(0, std::ios::beg);
	void* modelMem = malloc(modelSize);
	trtModelStream.read((char*)modelMem, modelSize);
	nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(modelMem, modelSize, NULL);
	free(modelMem);
	return engine;
}

void saveEngine(ICudaEngine& engine, const char* filepath)
{
	IHostMemory* trtModelStream;
	trtModelStream = engine.serialize();
	std::ofstream p(filepath);
	p.write((const char*)trtModelStream->data(), trtModelStream->size());
}


int main(int argc, char** argv)
{
	// Initialise ROS
	ros::init(argc, argv, "inference_engine");
	ros::NodeHandle nh;

	// Location of resource files
	string packagePath = ros::package::getPath("inference_engine");
	sprintf(resourceDirectory, "%s/resources/", packagePath.c_str());
	char trtFilepath[256];
	char uffFilepath[256];
	char timesFilepath[256];
	char labelsFilepath[256];
	sprintf(trtFilepath, "%s/%s", resourceDirectory, trtFilename);
	sprintf(uffFilepath, "%s/%s", resourceDirectory, uffFilename);
	sprintf(timesFilepath, "%s/%s", resourceDirectory, timesFilename);
	sprintf(labelsFilepath, "%s/%s", resourceDirectory, labelsFilename);

	// Get timestamp
	time_t rawtime;
	tm* timeinfo;
	char timestamp[80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(timestamp, 80, "%Y%m%d-%H%M%S", timeinfo);

	int maxBatchSize = 4;
	auto parser = createUffParser();

	/* Register tensorflow input */
	parser->registerInput("input_1", Dims3(INPUT_C, INPUT_H, INPUT_W), UffInputOrder::kNCHW);
	parser->registerOutput("fc9/Sigmoid");

	// If TRT file exists, load first
	ICudaEngine* engine;
	ifstream f(trtFilepath);
	if (f.good())
	{
		ROS_INFO("TRT file exists, constructing engine...");
		f.close();
		engine = loadEngine(trtFilepath);
		ROS_INFO("Loaded the TRT model and created the engine.");
	}
	else
	{
		// Else load from UFF and save TRT
		ROS_INFO("No TRT model file, constructing engine from UFF.\n");
		engine = loadModelAndCreateEngine(uffFilepath, maxBatchSize, parser);
		saveEngine(*engine, trtFilepath);
		ROS_INFO("Loaded the UFF model, created and saved the engine.\n");
	}
	if (!engine)
		ROS_ERROR("Unable to load TRT or UFF model");

	/* we need to keep the memory created by the parser */
	parser->destroy();

	// Prepare buffer to hold batch
	IExecutionContext* context = engine->createExecutionContext();
	int nbBindings = engine->getNbBindings();
	ROS_INFO("Number of bindings: %d", nbBindings);
	assert(nbBindings == NB_BINDINGS);
	auto bufferSizes = calculateBindingBufferSizes(*engine, nbBindings, BATCH_SIZE);
	int bindingIdxInput = 0;
	int bindingIdxOutput = 1;
	auto bufferSizesInput = bufferSizes[bindingIdxInput];
	auto bufferSizesOutput = bufferSizes[bindingIdxOutput];

	ros::Subscriber sub = nh.subscribe("/preprocessed_images", 4, gatherCallback);
	ros::Publisher target_publisher = nh.advertise<custom_messages::TargetInfo>("/targets", 100);

	// Create times file
	ofstream timesFile;
	timesFile.open(timesFilepath, ofstream::out);
	if (timesFile.is_open())
	{
		// Write header
		timesFile << "Batch,Inference time (ms)\n";
		timesFile.close();
	}
	// Create labels file
	ofstream labelsFile;
	labelsFile.open(labelsFilepath, ofstream::out);
	if (labelsFile.is_open())
	{
		// Write header
		labelsFile << "Frame,Camera,Label\n";
		labelsFile.close();
	}

	int batchCount = 0;

	while (ros::ok()) {

		ros::spinOnce();

		// If batch is ready? (i.e. Frame from each camera has been recieved)
		if (std::all_of(std::begin(recieve_flags), std::end(recieve_flags), [](bool i) { return i; })) {

			// Load batch onto CUDA device
			buffers[bindingIdxOutput] = safeCudaMalloc(bufferSizesOutput.first * elementSize(bufferSizesOutput.second));
			buffers[bindingIdxInput] = prepareBatchForCuda(bufferSizesInput.first, bufferSizesInput.second);

			// Process batch and report inference time
			auto t_start = std::chrono::high_resolution_clock::now();
			context->execute(BATCH_SIZE, &buffers[0]);
			auto t_end = std::chrono::high_resolution_clock::now();
			float ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
			ROS_INFO("Processed batch in %f ms", ms);

			// Append time to csv
			ofstream timesFile;
			timesFile.open(timesFilepath, ofstream::out | ofstream::app);
			if (timesFile.is_open())
			{
				// Write batch # and inference time to row
				timesFile << batchCount << "," << ms << "\n";
				timesFile.close();
			}

			// Free CUDA input buffer
			CHECK(cudaFree(buffers[bindingIdxInput]));

			// Gather output information
			int bindingIdxOutput = 1;
			int cameraCount = 4;
			int outputCount = 9;
			assert(outputCount * cameraCount == bufferSizesOutput.first);
			assert(sizeof(float) == elementSize(bufferSizesOutput.second));

			size_t memSize = cameraCount * outputCount * elementSize(bufferSizesOutput.second);
			float* outputs = new float[cameraCount * outputCount];
			CHECK(cudaMemcpy(outputs, buffers[bindingIdxOutput], memSize, cudaMemcpyDeviceToHost));

			for (int cameraID = 0; cameraID < cameraCount; cameraID++)
			{
				int maxIdx = cameraID * outputCount;
				for (int i = 0; i < outputCount; ++i)
					if (outputs[cameraID * outputCount + i] > outputs[maxIdx])
						maxIdx = i;
				float score = outputs[maxIdx];
				int label = maxIdx % outputCount;
				ROS_INFO("Image classified as label %d with %f%% confidence", label, score);

				// Present target for spraying if non-negative and > 50% confidence
				if (score > 0.5 && label != 8) {
					// Publish target
					custom_messages::TargetInfo newTarget;
					newTarget.header.stamp = timestamps[cameraID];
					newTarget.targetId = 0;
					newTarget.cameraId = cameraID + 1;
					newTarget.xPixelLocation = 240;
					newTarget.yPixelLocation = 150;
					newTarget.boxHeight = 300;
					newTarget.boxWidth = 480;
					target_publisher.publish(newTarget);
				}

				// Append label to csv
				ofstream labelsFile;
				labelsFile.open(labelsFilepath, ofstream::out | ofstream::app);
				if (labelsFile.is_open())
				{
					// Write frame, camera and predicted label to file
					labelsFile << frameCounts[cameraID] << "," << cameraID << "," << label << "\n";
					labelsFile.close();
				}
			}

			// Free CUDA output buffer
			CHECK(cudaFree(buffers[bindingIdxOutput]));

			// Reset flags
			for (int i = 0; i < sizeof(recieve_flags); i++) {
				recieve_flags[i] = false;
			}

			batchCount++;
		}
	}

	engine->destroy();
	shutdownProtobufLibrary();
}
