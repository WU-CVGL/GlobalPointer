<p align="center">
  <h1 align="center"> <ins>GlobalPointer</ins> ⚡️<br>Large-Scale Plane Adjustment with Bi-Convex Relaxation</h1>
</p>

<a href="https://arxiv.org/abs/2407.13537"><img src="https://img.shields.io/badge/arXiv-2407.13537-b31b1b.svg"></a>
<a href="https://bangyan101.github.io/GlobalPointer/"><img src="https://img.shields.io/badge/Project-Page-green.svg"/></a>

This is an official implementation of our ECCV 2024 paper 
[**GlobalPointer**: Large-Scale Plane Adjustment with Bi-Convex Relaxation](https://bangyan101.github.io/GlobalPointer/).


## Requirement
1. Matlab r2023a
2. Yalmip [Version R20230622](https://yalmip.github.io/R20230622)
3. MOSEK [Version 10.1.11](https://www.mosek.com/downloads/10.1.11/)


## Replace TODO codes
1. Find our main matlab script in **main/sythetic_main.mlx** and replace the following codes.

2. Replace `PATH_TO_YALMIP` and `PATH_TO_MOSEK` with your own YALMIP and MOSEK solver path respectively.

3. Replace `PATH_TO_PROJECT` with your own project root path.
```
% ---------------------- TODO ----------------------
addpath(genpath("PATH_TO_YALMIP\YALMIP-master"))
addpath(genpath("PATH_TO_MOSEK\Mosek\10.1\toolbox\r2017a"))
root_path = "PATH_TO_PROJECT\GlobalPointer\";
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


## Citation

If you find our work useful in your research, please consider citing:

```
@inproceedings{Liao2024GlobalPointer,
    author 	= { Bangyan Liao and Zhenjun Zhao and Lu Chen and Haoang Li and Daniel Cremers and Peidong Liu },
    title 	= { GlobalPointer: Large-Scale Plane Adjustment with Bi-Convex Relaxation},
    booktitle = {European Conference on Computer Vision (ECCV)},
    year 	= 2024,
    keywords = {Plane Adjustment, Semidefinite Programming (SDP), Convex Relaxation}
}
```