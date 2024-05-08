## [3. The C++ API](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#c_topics)

This chapter illustrates basic usage of the C++ API, assuming you are starting with an ONNX model. [sampleOnnxMNIST](https://github.com/NVIDIA/TensorRT/tree/main/samples/sampleOnnxMNIST) illustrates this use case in more detail.

The C++ API can be accessed through the header NvInfer.h, and is in the nvinfer1 namespace. For example, a simple application might begin with:
```cpp
#include “NvInfer.h”
using namespace nvinfer1;

```

Interface classes in the TensorRT C++ API begin with the prefix I, for example ILogger, IBuilder, and so on.

A CUDA context is automatically created the first time TensorRT makes a call to CUDA, if none exists before that point. It is generally preferable to create and configure the CUDA context yourself before the first call to TensorRT.

In order to illustrate object lifetimes, code in this chapter does not use smart pointers; however, their use is recommended with TensorRT interfaces.

### [3.1. The Build Phase](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#build-phase-c)

To create a builder, you first must instantiate the ILogger interface. This example captures all warning messages but ignores informational messages:
```cpp
class Logger : public ILogger           
{
    void log(Severity severity, const char* msg) noexcept override
    {
        // suppress info-level messages
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} logger;
```

You can then create an instance of the builder:
```cpp
IBuilder* builder = createInferBuilder(logger);
```

### [3.1.1. Creating a Network Definition](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#create_network_c)

After the builder has been created, the first step in optimizing a model is to create a network definition:
```cpp
uint32_t flag = 1U << static_cast<uint32_t>
    (NetworkDefinitionCreationFlag::kEXPLICIT_BATCH); 

INetworkDefinition* network = builder->createNetworkV2(flag);

```

The kEXPLICIT_BATCH flag is required in order to import models using the ONNX parser. Refer to the [Explicit Versus Implicit Batch](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#explicit-implicit-batch "TensorRT supports two modes for specifying a network: explicit batch and implicit batch.") section for more information.

### [3.1.2. Importing a Model Using the ONNX Parser](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#import_onnx_c)

Now, the network definition must be populated from the ONNX representation. The ONNX parser API is in the file NvOnnxParser.h, and the parser is in the nvonnxparser C++ namespace.
```cpp
#include “NvOnnxParser.h”

using namespace nvonnxparser;

You can create an ONNX parser to populate the network as follows:

IParser* parser = createParser(*network, logger);

Then, read the model file and process any errors.

parser->parseFromFile(modelFile, 
    static_cast<int32_t>(ILogger::Severity::kWARNING));
for (int32_t i = 0; i < parser.getNbErrors(); ++i)
{
std::cout << parser->getError(i)->desc() << std::endl;
}
```
An important aspect of a TensorRT network definition is that it contains pointers to model weights, which are copied into the optimized engine by the builder. Since the network was created using the parser, the parser owns the memory occupied by the weights, and so the parser object should not be deleted until after the builder has run.

### [3.1.3. Building an Engine](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#build_engine_c)

The next step is to create a build configuration specifying how TensorRT should optimize the model.
```cpp
IBuilderConfig* config = builder->createBuilderConfig();
```
This interface has many properties that you can set in order to control how TensorRT optimizes the network. One important property is the maximum workspace size. Layer implementations often require a temporary workspace, and this parameter limits the maximum size that any layer in the network can use. If insufficient workspace is provided, it is possible that TensorRT will not be able to find an implementation for a layer. By default the workspace is set to the total global memory size of the given device; restrict it when necessary, for example, when multiple engines are to be built on a single device.
```cpp
config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1U << 20);
```

Once the configuration has been specified, the engine can be built.
```cpp
IHostMemory* serializedModel = builder->buildSerializedNetwork(*network, *config);
```
Since the serialized engine contains the necessary copies of the weights, the parser, network definition, builder configuration and builder are no longer necessary and may be safely deleted:
```cpp
delete parser;
delete network;
delete config;
delete builder;
```
The engine can then be saved to disk, and the buffer into which it was serialized can be deleted.
```cpp
delete serializedModel
```

Note: Serialized engines are not portable across platforms or TensorRT versions. Engines are specific to the exact GPU model that they were built on (in addition to the platform and the TensorRT version).

Since building engines is intended as an offline process, it can take significant time. Refer to the [Optimizing Builder Performance](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#opt-builder-perf "For each layer, the TensorRT builder profiles all the available tactics to search for the fastest inference engine plan. The builder time can be long if the model has a large number of layers or complicated topology. The following sections provide options to reduce builder time.") section for how to make the builder run faster.

### [3.2. Deserializing a Plan](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#perform_inference_c)

Assuming you have previously serialized an optimized model and want to perform inference, you must create an instance of the Runtime interface. Like the builder, the runtime requires an instance of the logger:
```cpp
IRuntime* runtime = createInferRuntime(logger);
```
After you have read the model into a buffer, you can deserialize it to obtain an engine:
```cpp
ICudaEngine* engine = 
  runtime->deserializeCudaEngine(modelData, modelSize);
```
### [3.3. Performing Inference](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#perform-inference)

The engine holds the optimized model, but to perform inference we must manage additional state for intermediate activations. This is done using the ExecutionContext interface:
```cpp
IExecutionContext *context = engine->createExecutionContext();
```

An engine can have multiple execution contexts, allowing one set of weights to be used for multiple overlapping inference tasks. (A current exception to this is when using dynamic shapes, when each optimization profile can only have one execution context, unless the preview feature, kPROFILE_SHARING_0806, is specified.)

To perform inference, you must pass TensorRT buffers for input and output, which TensorRT requires you to specify with calls to setTensorAddress, which takes the name of the tensor and the address of the buffer. You can query the engine using the names you provided for input and output tensors to find the right positions in the array:
```cpp
context->setTensorAddress(INPUT_NAME, inputBuffer);
context->setTensorAddress(OUTPUT_NAME, outputBuffer);
```


You can then call TensorRT’s method enqueueV3 to start inference using a CUDA stream:
```cpp
context->enqueueV3(stream);
```

A network will be executed asynchronously or not depending on the structure and features of the network. A non-exhaustive list of features that can cause synchronous behavior are data dependent shapes, DLA usage, loops, and plugins that are synchronous, for example. It is common to enqueue data transfers with cudaMemcpyAsync() before and after the kernels to move data from the GPU if it is not already there.

To determine when the kernels (and possibly cudaMemcpyAsync()) are complete, use standard CUDA synchronization mechanisms such as events or waiting on the stream.