import os
from ultralytics import YOLO
import tensorrt as trt

MODEL_PATH = "../../models/yolo26_large_ds09_v2.pt"

# https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/python-api-docs.html
def convert_tensorrt(model_name):
    # The Build Phase
    # The build phase uses the builder to optimize a model and produce an engine. To create a builder:
    # Create a logger. The Python bindings include a simple logger implementation that logs all messages 
    # preceding a certain severity to stdout:
    logger = trt.Logger(trt.Logger.WARNING)
    
    # Create the builder:
    builder = trt.Builder(logger)
    
    # Creating a Network Definition
    # After the builder has been created, the first step in optimizing a model is to create a network definition:
    # Specify the network creation options using a combination of flags OR’d together (or 0 for none).
    # Note that all networks are strongly typed in TensorRT 11, so you need not set the STRONGLY_TYPED 
    # flag (a warning will be emitted if you do). For more information, refer to the Strongly Typed Networks section.
    # Create the network:
    flag = 0
    network = builder.create_network(flag)
    
    # Importing a Model Using the ONNX Parser
    # Now, the network definition must be populated from the ONNX representation.
    # Create an ONNX parser to populate the network.
    parser = trt.OnnxParser(network, logger)
    
    # Read the model file and process any errors.
    success = parser.parse_from_file(MODEL_PATH)
    for idx in range(parser.num_errors):
        print(parser.get_error(idx))
    if not success:
        pass # Error handling code here
    
    # IF NOT WORKING -> SEE IMPORTING A MODEL USING THE ONNX PARSER W/ CUSTOM WEIGHTS
    
    # Building an Engine
    # The next step is to create a build configuration specifying how TensorRT should optimize the model:
    # Create a builder configuration object. This interface has many properties that you can set 
    # to control how TensorRT optimizes the network.
    config = builder.create_builder_config()
    
    # Set the maximum workspace size. Layer implementations often require a temporary workspace, and 
    # this parameter limits the maximum size that any layer in the network can use. If insufficient 
    # workspace is provided, TensorRT might not be able to find an implementation for a layer. By default, 
    # the workspace is set to the total global memory size of the given device; restrict it when necessary, 
    # such as when multiple engines are to be built on a single device.
    # 1 GiB
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)
    
    # Build the serialized engine:
    serialized_engine = builder.build_serialized_network(network, config)
    
    # Save the serialized engine to disk for future use:
    with open(f"{model_name}.engine", "wb") as f:
        f.write(serialized_engine)

# Convert to onnx first. The trtexec is going to make the optimization to tensorrt format,
# adequing it to Jetson most optimal compilation
def conv_onnx_onnx(model_name):
    model_path = f"../../models/{model_name}"
    model = YOLO(model_path, 'detect', True)
    path_onnx = model.export(format='onnx', half=True) # half=True is not assured to be correct
    return path_onnx

def main():
    model_paths = os.listdir("../models")
    print(model_paths)
    for model_name in model_paths: 
        path_engine = conv_onnx_onnx(model_name)
        print(model_name, "found and corrrectly converted to tensorrt model")

if __name__ == "__main__":
    main()
