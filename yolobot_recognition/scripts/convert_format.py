import os
from ultralytics import YOLO

model_path = "../../models/yolo26_large_ds09_v2.pt"
def conv_onnx_onnx(model_path):
    model = YOLO(model_path, 'detect', True)
    model_path = f"../../models/{model}"
    path_onnx = model.export(format='onnx')
    return path_onnx

def main():
    models_paths = os.listdir("../../models/")
    for model_path in models_paths: 
        path_engine = conv_onnx_onnx(model_path)

if __name__ == "__main__":
    main()
