# vision_measure

# 说明

这是一个基于`C++`和`OpenCV`实现的基于CCT编解码的图像拼接和摄影测量程序，主要针对弱纹理特征的物体进行机器视觉测量。程序目前主要包含两个功能模块：CCT码检测与识别模块，基于特征点匹配的多图像拼接模块。

针对图像中出现的CCT码，程序可以自动检测并识别出对应的ID号（即解码），具体效果如下图所示。

![cct_extraction_1](https://github.com/JC-Yu/vision_measure/tree/main/resources/demo/cct_extraction_1.jpg)

基于解码结果，可以得到特征点信息。在多张图像中基于特征点匹配，即可完成图像的拼接任务。具体匹配效果如下图所示。

![image_matching_2](https://github.com/JC-Yu/vision_measure/tree/main/resources/demo/image_matching_2.jpg)

利用匹配点信息求解图像之间的单应矩阵，即可对图像进行变换，从而实现拼接，下图为2张图片的拼接效果展示。

![imageStiched](https://github.com/JC-Yu/vision_measure/tree/main/resources/demo/imageStiched.jpg)

# 运行说明

## 依赖项

* `OpenCV 4.2.0`
* `CMake 3.0`及以上版本

## 在 Ubuntu 20.04 系统上运行

1. 修改配置文件

修改工程中的 `config.yaml` 文件，具体需要修改存放图片和文件夹的路径，可根据注释信息进行修改。

![config](https://github.com/JC-Yu/vision_measure/tree/main/resources/demo/config.png)

2. 编译

```bash
# 进入到工程目录
# 创建build目录
mkdir build
cd build
# 利用CMake构建
cmake ..
# 编译
make
# 运行测试程序
./test
```

## 三种运行模式

在`config.yaml`文件中可以修改`InputFormat`参数来选择不同的运行模式。

1. **视频流输入模式**。通过配置`InputFormat`为`"Video"`，可以启用该模式。该模式可以针对摄像头采集的实时图像进行CCT编码的检测和识别，并进行显示。通过配置`Camera`参数可以选择不同的摄像头类型，选用`"Internal_Cam"`表示使用笔记本电脑自带的摄像头，而选用`"Web_Cam"`则表示使用外接摄像头。

2. **单帧图像识别模式**。通过配置`InputFormat`为`"Image"`，可以启用该模式。该模式可以针对给定的一张图片进行CCT编码的检测和识别，并进行显示。启用该模式时，需要同时在`ImagePath`参数中给定图片的存放路径。

3. **多帧图像拼接模式**。通过配置`InputFormat`为`"MultiView"`，可以启用该模式。该模式可以对多张含有CCT编码的图片进行拼接，得到拼接结果。启用该模式时，需要同时在`ImagesDir`参数中给定图片文件夹的存放路径，同时图片需要按照"1, 2, 3, ..."的方式进行命名。在`ImageFormat`参数中需要指出图片的格式，在`ImageNums`参数中指出待拼接图片的数量。

## 参考资料

[1] 关于CCT编解码的python程序实现：https://github.com/poxiao2/CCTDecode

[2] 《数字近景工业摄影测量中编码标志点识别与检测技术的研究_苏新勇》

[3] 基于SIFT的图像拼接算法实现：https://www.cnblogs.com/skyfsm/p/7411961.html

[4] 基于线性最小二乘法求解仿射变换矩阵：https://www.cnblogs.com/bingdaocaihong/p/7003581.html