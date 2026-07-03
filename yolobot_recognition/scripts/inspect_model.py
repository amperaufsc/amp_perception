import torch

ckpt = torch.load("../models/besta.pt", map_location="cpu", weights_only=False)

print(type(ckpt))
if isinstance(ckpt, dict):
    print(ckpt.keys())

sd = ckpt["model"].state_dict() if hasattr(ckpt.get("model", ckpt), "state_dict") else ckpt

for name, tensor in sd.items():
    print(name, tuple(tensor.shape))

print(ckpt.get("epoch"))
print(ckpt.get("train_args"))     # hyperparameters used
print(ckpt.get("model").yaml)     # architecture config, if present
print(ckpt.get("model").names)    # class names, if present