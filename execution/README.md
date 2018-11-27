# Executable
## Input line Drawings
- For the input images, we use only rasterized images and .JPG is the best.
- Please be careful of that our system wil calculate the bounding box of the line-drawing pattern and use it to extract line-drawing. Please make sure the ratio of width/height is lower than 1.2, or it would be applied non-uniform deformation.
<img width=30% src="https://cgv.cs.nthu.edu.tw/download?guid=756c408a-eeec-11e8-9b71-0011328fa92e">

## Script File
- Please use script file (script.txt) to specify input images, the format is as followed
```
The number of input images
Full path of input image 0
Full path of input image 1
...
Full path of input image n
1
XXX
1
Full path of specified output file
```

##  Getting Started
- Please use these instructions to create the multi-view wire art.
- The 3d model will be placed in Results\objModel.
```
> MVWA
> Please input your requirement: 0
> SkeletonExtraction_64
> MVWA
> Please input your requirement: 1
```
