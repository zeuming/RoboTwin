### Installation
---

#### **Dependencies**

Python versions:

* Python 3.8, 3.10

Operating systems:

* Linux: Ubuntu 18.04+, Centos 7+


Hardware:

* Rendering: NVIDIA or AMD GPU

* Ray tracing: NVIDIA RTX GPU or AMD equivalent

* Ray-tracing Denoising: NVIDIA GPU

* GPU Simulation: NVIDIA GPU

Software:

* Ray tracing: NVIDIA Driver >= 470
* Denoising (OIDN): NVIDIA Driver >= 520


First, prepare a conda environment.
```bash
conda create -n RoboTwin python=3.8
```

```
pip install torch==2.3.1 sapien==3.0.0b1 scipy==1.10.1 mplib gymnasium==0.29.1 trimesh==4.4.3 open3d==0.18.1 imageio==2.34.2 pydantic
```
2. Install pytorch3d
```
cd third_party/pytorch3d_simplified && pip install -e . && cd ..
```

3. Install DP3 (Optional)
3.1 Install torch
```
# if using cuda>=12.1
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
# else, 
# just install the torch version that matches your cuda version
```
3.2 Install dp3
```
cd 3D-Diffusion-Policy && pip install -e . && cd ..
```
3.3 Install some necessary package
```
pip install zarr==2.12.0 wandb ipdb gpustat dm_control omegaconf hydra-core==1.2.0 dill==0.3.5.1 einops==0.4.1 diffusers==0.11.1 numba==0.56.4 moviepy imageio av matplotlib termcolor
```


