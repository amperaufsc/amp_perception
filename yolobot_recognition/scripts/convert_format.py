import os
from ultralytics import YOLO

model_path = "../../models/yolo26_large_ds09_v2.pt"

# Convert to onnx first. The trtexec is going to make the optimization to tensorrt format,
# adequing it to Jetson most optimal compilation
def conv_onnx_onnx(model_path):
    model = YOLO(model_path, 'detect', True)
    model_path = f"../../models/{model}"
    path_onnx = model.export(format='onnx', half=True)
    return path_onnx

def main():
    model_paths = os.listdir("../models")
    print(model_paths)
    for model_path in model_paths: 
        path_engine = conv_onnx_onnx(model_path)
        print(model_path, "found and corrrectly converted to tensorrt model")

if __name__ == "__main__":
    main()
