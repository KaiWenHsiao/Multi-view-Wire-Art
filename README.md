# Multi-view Wire Art
This is the C++ implementation of the paper [Multi-view Wire Art (SIGGRAPH Asia 2018)](https://cgv.cs.nthu.edu.tw/download/file?guid=3d4d4ee9-cdec-11e8-9b71-0011328fa92e) 

## Introduction
Wire art is the creation of 3D sculptural art using wire strands. As the 2D projection of a 3D wire sculpture forms line drawing patterns, it is possible to craft multi-view wire sculpture art — a static sculpture with multiple interpretations when perceived at different viewpoints. In this project, we present a computational framework for automatic creation of multi-view 3D wire sculpture. Our system takes two or three user-specified line drawings and the associated viewpoints as inputs and output 3D multi-view wire sculptures. For more detail please refer to our [Project Page](https://cgv.cs.nthu.edu.tw/projects/recreational_graphics/MVWA).

## Results
Our method handles inputs with varying complexity. More results and dynamic exhibition can be found in our [Online Gallery](https://cgv.cs.nthu.edu.tw/MVWA_onlinegallery/).
<img style="width: 50%;" src="https://cgv.cs.nthu.edu.tw/download?guid=801220cf-c313-11e8-9b71-0011328fa92e">

## Prerequisite
- [OpenGL Mathematics (GLM)](https://glm.g-truc.net/0.9.9/index.html)
- [OpenCV](https://opencv.org/)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Citation 
If you use this code for your research, please cite this paper:
```
@article{Hsiao:2018:MVWA:,
 author = {Hsiao, Kai-Wen and Huang, Jia-Bin and Chu, Hung-Kuo},
 title = {Multi-view Wire Art},
 journal = {ACM Trans. Graph.},
 volume = {37},
 number = {6},
 year = {2018},
 pages = {242:1--242:11},
 articleno = {242},
 numpages = {11}
} 
```

## Related Projects
[Shadow Art](https://graphics.stanford.edu/~niloy/research/shadowArt/shadowArt_sigA_09.html) : A computational tools for the creation of shadow art.<br>
[Image-based Reconstruction of Wire Art](http://geometry.cs.ucl.ac.uk/projects/2017/wire-art-reconstruction/) : A image-based method that reconstructs a set of continuous 3D wires.

