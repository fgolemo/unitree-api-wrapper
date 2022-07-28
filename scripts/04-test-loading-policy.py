import torch
import os


### IF YOU GET AN ERROR LIKE
# RuntimeError: PytorchStreamReader failed locating file constants.pkl: file not found
### THEN YOU ARE USING A POLICY THAT ISN'T JIT-COMPILED.
### FIRST RUN scripts/02-play.py WITH THE `EXPORT_POLICY` FLAG TURNED ON

dev = torch.device("cpu")  #
model = torch.jit.load("../policies/go1_flat---22May28_17-36-59_.pt").to(dev)
input_ = torch.zeros((32, 48), dtype=torch.float32, device=dev)
output = model(input_)
