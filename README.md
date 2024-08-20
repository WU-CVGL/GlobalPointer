<h1 align=center font-weight:150> <strong><i>GlobalPointer</i></strong>: <strong><i>Large-Scale Plane Adjustment
with Bi-Convex Relaxation</i></strong></h1>

<a href="https://arxiv.org/abs/2407.13537"><img src="https://img.shields.io/badge/arXiv-2407.13537-b31b1b.svg"></a>
<a href="https://bangyan101.github.io/GlobalPointer/"><img src="https://img.shields.io/badge/Project-Page-green.svg"/></a>

This as an official implementation of our ECCV 2024 paper 
[**GlobalPointer**: Large-Scale Plane Adjustment with Bi-Convex Relaxation]([https://lingzhezhao.github.io/BAD-Gaussians/](https://bangyan101.github.io/GlobalPointer/)).


## Requirement
1. Matlab r2023a
2. [Yalmip](https://yalmip.github.io/)
3. [Mosek](https://www.mosek.com/)

## Replace TODO codes
find our main matlab script in **main/sythetic_main.mlx** and replace the following codes 

```
% ---------------------- TODO ----------------------
% please replace with your own YALMIP and Mosek solver path
addpath(genpath("C:\Users\10627\Desktop\YALMIP-master"))
addpath(genpath("C:\Program Files\Mosek\10.1\toolbox\r2017a"))

% please replace with your project root path
root_path = "C:\Users\10627\Desktop\GlobalPointer\";
addpath(genpath(root_path))
% ---------------------- TODO END ----------------------
```

## Select experiment 
we provide three full experiment setups
- increasing point cloud noise
- increasing pose initialization noise
- increasing number of poses and planes
```
% ---------------------- Experiment Selection Setup ----------------------
% please select your experiment setup
param.increasing_point_noise = false;
param.increasing_pose_noise = false;
param.increasing_scale = true;
% ---------------------- Experiment Selection Setup END ----------------------
```


## Example results



