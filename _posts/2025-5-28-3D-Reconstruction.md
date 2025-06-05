---
title: "文化遺産の保存と共有のための３次元復元（英語版）- 共同研究"
date: 2025-05-28
categories: projects
---

Some would say that we are currently living through the information age, a universal turning point for civilization which has the potential to rival the agricultural and industrial revolution in terms of societal impact. One important aspect of the information age is that it has enabled the sharing of information with far less work and with far more accessibility than in the past. The invention of the internet and its advancement as a global exchange for information has undoubtedly raised the bar for the collective intelligence of mankind. Increasing the amount of information that can be stored and transferred has been a field of constant research in the past decades, but there comes a point where the medium of information exchange must be updated to allow for further growth. Recent advancements in fields such as VR (Virtual Reality) have begun to foreshadow the next stage of the information revolution, beginning with a new medium to share information with a level of immersion that can hardly be imagined today. 

To support an advancement of the information medium, a simple and accessible method to convert seamlessly between objects in the real world and those in the virtual world becomes of dire demand. 3D reconstruction is a field of computer vision that involves the “reconstruction” of 3D objects from 2D pictures. Examples of uses for 3D reconstruction include, generating 3D models of museum artifacts to share with the world, or increasing the reliability and efficiency of self-driving cars by generating models of the environment with greater accuracy.

研究機関：2024年4月ー2024年7月

# 2. Camera Calibration
## 2.1. Motivation 

Camera calibration is an important step in the 3D-reconstruction model and the primary focus of this research report. The role of camera calibration is to parameterize the internal properties and position/orientation of a given camera from the images that it takes. Additionally, camera lenses are necessarily flawed in their construction, and have varying degrees of optical distortion that can be modeled and accounted for through camera calibration. This research report details an implementation of the OpenCV (Open Source Computer Vision Library) camera calibration model, which involves matching 3D object points to their corresponding 2D image points, and optimizing several parameters based on the reprojection error.
 
## 2.2. Principles of Camera Calibration

### 2.2.1. Pin-hole Camera Model

The pin-hole camera model is a theoretical camera model that enables a simple geometrical interpretation of the transformation between a world coordinate and its corresponding pixel coordinate. The pin-hole camera model consists of a sensor, a box, and a pinhole, and they are arranged as shown in figure 1. The sensor is represented by the plane between the camera and the object, and the pinhole is represented by the hole where the orange rays intersect. 

![figure1](/assets/2025-5-28-3D-Reconstruction/figure1.png "figure1"){: .img-large-centered }
<div style="text-align: center;"> Figure 1: A graphic representation of the pin-hole model [1] </div>  
<br>

### 2.2.2. Homogeneous Coordinates
The homogeneous coordinate system is a coordinate system that is used primarily in computer vision and computer graphics, for its ability to express affine transformation and projective transformations with relative ease. This is because homogeneous coordinates can express points at infinity with finite coordinates [2]. In other words, they represent a ray through space with a single n+1 dimensional vector.
One would turn a n-dimensional vector into a homogeneous vector by appending a 1 as follows:

$$
\left[\begin{matrix}X\\Y\\Z\\\end{matrix}\right]\longrightarrow\left[\begin{matrix}X\\Y\\Z\\1\\\end{matrix}\right]
$$

Similarly, one would turn a (n+1)-dimensional homogeneous vector into a n-dimensional Cartesian vector by dividing all elements by the last element and removing the last element as follows:

$$
\left[\begin{matrix}X\\Y\\Z\\\end{matrix}\right]\longrightarrow\left[\begin{matrix}\frac{X}{Z}\\\frac{Y}{Z}\\\end{matrix}\right]
$$

### 2.2.3. Extrinsic Matrix

The extrinsic matrix of a picture represents the affine transformation of the 3D world coordinate system to the 3D camera coordinate system. The name extrinsic matrix implies that the values contained within the matrix are extrinsic to the camera and represents how the camera is situated in space.

Any camera’s pose, i.e. its position and orientation, can be represented by 3-dimensional rotation and translation. The transformation of a 3D homogeneous world coordinate $\left(X_w,Y_w,Z_w,\ 1\right)$ to its 3D camera coordinate $\left(X_c,Y_c,Z_c\right)$ is given by,

$$
\left[\begin{matrix}X_c\\Y_c\\Z_c\\\end{matrix}\right]=\left[\begin{matrix}r_{11}&r_{12}&r_{13}&t_x\\r_{21}&r_{22}&r_{23}&t_y\\r_{31}&r_{32}&r_{33}&t_z\\\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\\\end{matrix}\right].
$$

The extrinsic matrix is thus a joint rotation and transformation matrix $\left[R\middle| t\right]$ given by equation (1), where $R$ is the rotational matrix and $t$ is the translation matrix representing the change in basis from the world coordinate to the camera coordinate.

$$
\begin{equation}
\left[R\middle| t\right]=\left[\begin{matrix}r_{11}&r_{12}&r_{13}&t_x\\r_{21}&r_{22}&r_{23}&t_y\\r_{31}&r_{32}&r_{33}&t_z\\\end{matrix}\right]
\end{equation}
$$


### 2.2.4. Intrinsic Matrix

The Intrinsic matrix of a given picture represents the transformation of the 3D camera coordinate system to the 2D pixel coordinate system. The name “intrinsic matrix” implies that the values within this matrix (intrinsic parameters) are characteristics of the camera that took the photo, and not dependent on external factors such as the location or orientation of the camera in the world coordinate system. 

Figure 2 depicts the relationship between a 3D world coordinate $\left(X_w,Y_w,Z_w\right)$ and its corresponding 2D pixel coordinate $\left(u,v\right)$ in the pin-hole camera model. 

![figure2](/assets/2025-5-28-3D-Reconstruction/figure2.png "figure2"){: .img-large-centered }
<div style="text-align: center;"> Figure 2: the relationship between a 3D world coordinate and a 2D pixel coordinate [3] </div>  
<br>

When the principle point of the image sensor (i.e. the center of the image sensor) is given by $\left(c_x,c_y\right)$, and the 3D world coordinate is defined by the camera coordinate system, the 2D pixel coordinate $\left(u,v\right)$ of a given 3D camera coordinate $\left(X_c,Y_c,Z_c\right)$ is described by

$$
\begin{bmatrix} u \\ v \end{bmatrix}
= 
\begin{bmatrix} f_x \frac{X_c}{Z_c} + c_x  \\  f_y \frac{Y_c}{Z_c} + c_y \end{bmatrix}
$$

represented in practice as a matrix product of the form 

$$
\left[\begin{matrix}u\\v\\1\\\end{matrix}\right]=\left[\begin{matrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\\\end{matrix}\right]\left[\begin{matrix}X_c\\Y_c\\Z_c\\\end{matrix}\right],
$$

where $\left[\begin{matrix}u&v&1\end{matrix}\right]^\top$ is a homogeneous representation of $\left[\begin{matrix}u&v\end{matrix}\right]^\top$. 

The intrinsic matrix $A$ is thus defined by matrix (2), where $f_x$ and $f_y$ represent the camera’s focal length in the x-direction and y-direction respectively, and $c_x$ and $c_y$ represent the camera’s principle point in x-coordinates and y-coordinates respectively.

$$
\begin{equation}
A=\left[\begin{matrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\\\end{matrix}\right]
\end{equation}
$$

### 2.2.5. Optical Distortion

In camera calibration, it is typical to consider two types of optical distortion: radial and tangential distortions. Radial and tangential distortions are primarily caused by manufacturing inconsistencies that alter the shape of the lens. The effects of radial and tangential distortions are mitigated in the calibration process by altering the intrinsic matrix equation into equation (3).

$$
\begin{equation}
\left[\begin{matrix}u\\v\\\end{matrix}\right]=\left[\begin{matrix}f_xx^{\prime\prime}+c_x\\f_yy^{\prime\prime}+c_y\\\end{matrix}\right]
\end{equation}
$$

With

$$
\left[\begin{matrix}x^{\prime\prime}\\y^{\prime\prime}\\\end{matrix}\right] = \left[\begin{matrix}x^\prime\frac{1+k_1r^2+k_2r^4+k_3r^6}{1+k_4r^2+k_5r^4+k_6r^6}+2p_1x^\prime y^\prime+p_2\left(r^2+2x^{\prime2}\right)+s_1r^2+s_2r^4\\y^\prime\frac{1+k_1r^2+k_2r^4+k_3r^6}{1+k_4r^2+k_5r^4+k_6r^6}+p_1\left(r^2+2y^{\prime2}\right)+2p_2x^\prime y^\prime+s_3r^2+s_4r^4\\\end{matrix}\right],
$$  

$$
\begin{align*}
r^2 &= x^{\prime2}+y^{\prime2}, \\
\left[x^\prime,y^\prime\right] &= \left[\frac{X_c}{Z_c},\frac{Y_c}{Z_c}\right].
\end{align*}
$$

Where, $k_1, k_2, k_3, k_4, k_5,$ and $k_6$ are the radial distortion coefficients; $p_1$ and $p_2$ are tangential distortions coefficients; and $s_1, s_2, s_3,$ and $s_4$ are the thin prism distortion coefficients.

### 2.2.6. Camera Matrix

The final camera matrix is a matrix multiplication of the intrinsic and extrinsic matrices, and represented in equation (4), where s is a scale factor that adjusts for the unknown scale factor of the projective transformation.

$$
\begin{equation}
s\left[\begin{matrix}u\\v\\1\\\end{matrix}\right]=\left[\begin{matrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\\\end{matrix}\right]\left[\begin{matrix}r_{11}&r_{12}&r_{13}&t_x\\r_{21}&r_{22}&r_{23}&t_y\\r_{31}&r_{32}&r_{33}&t_z\\\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\1\\\end{matrix}\right]
\end{equation}
$$

Equation (4) can also be represented in the form, 

$$
sp=A\left[R\middle| t\right]P_w,
$$

where $p$ is the pixel coordinate and $P_w$ is the world coordinate both represented in homogeneous coordinates.

## 2.3. Camera Calibration from a Checkerboard Pattern

The standard method for camera calibration, and the method outlined in OpenCV’s documentation, involves taking pictures of a well-defined geometric pattern such as a chessboard and finding a transformation to map the 2-dimensional image points (pixel coordinates of the corners) to their known 3-dimensional object points (world coordinates of the corners).

The checkerboard pattern used in this method is shown in figure 3, which has 7 by 10 inner corners. The pattern was obtained as a pdf file from Mark Hedley Jones’s Calibration Checkerboard Collection [4] and printed onto standard A4 sized printing paper. 20 images were then taken from various angles with the Canon EOS R100 mounted with the Canon Lens EF 28 mm and the EF-EOS mount adapter, at a fixed focal length.

![figure3](/assets/2025-5-28-3D-Reconstruction/figure3.svg "figure3"){: .img-large-centered }
<div style="text-align: center;"> Figure 3: 8 by 11 checkerboard pattern with 7 by 10 inner corners [4]  

black and white when printed on paper </div>  
<br>

Program 1 is a python script that was written with the OpenCV documentation [5] as reference, and takes in a directory of images in order to derive their image points and object points.

<div style="text-align: center;"> Program 1: “findCorners” function definition </div> 

```python
def findCorners(directory, dimensions, show):
    # assign object points
    objp = np.zeros((dimensions[0]*dimensions[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:dimensions[0], 0:dimensions[1]].T.reshape(-1, 2)

    objpoints = []  # 3d objectpoints
    imgpoints = []  # 2d image points
    names = []      # names of the image file

    # get /*.JPG from specified directory
    images = glob.glob(directory + '/*.JPG')  # Specify the path to your images

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, dimensions, None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
            names.append(fname)

        # display corners if specified
        if show == True:
            img = cv2.drawChessboardCorners(img, dimensions, corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()  
    return names, objpoints, imgpoints
```

The function in program 1, searches through a given directory for files with the “.JPG” extension and runs them through an OpenCV function called “findChessboardCorners”. This function effectively maps the known 3D coordinates of the chessboard corners to the 2D coordinate that they occupy in the given image. If corners are found, their pixel coordinate and world coordinates as well as the name of the image to which they belong, are stored in an array and are returned. Additionally, this information is passed to the “drawChessboardCorners” function where the detected chessboard corners are indicated on the image by dots, and their connections by lines. Figure 4 shows an example of a sample image (left) and its corresponding corner-indicated image (right).

![figure4](/assets/2025-5-28-3D-Reconstruction/figure4.png "figure4"){: .img-large-centered }
<div style="text-align: center;"> Figure 4: A sample image (left) and its corresponding corner-indicated image (right) </div>  
<br>

The next step is to utilize the derived object points and image points to calculate the camera matrix. Program 2 is a python script that utilizes the OpenCV “calibrateCamera” function to take object and image points as inputs and produces several matrix arrays which are combined to make the camera matrix. The “savePoint” and “saveCalib” functions are used to save the image points, object points, and resulting camera matrices as NumPy files. Their definitions are shown in program 3.

<div style="text-align: center;"> Program 2: camera calibration from image points and object points </div> 

```python
# Number of internal corners in the chessboard
DIMENSIONS = (7, 10)

# Current Directory 
DIRECTORY = 'batch1'

# get object points and image points
names, objpoints, imgpoints = findCorners(DIRECTORY, DIMENSIONS, 1)

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, RESOLUTION, None, None)

# save points and calibration
savePoint(DIRECTORY, names, objpoints, imgpoints)
saveCalib(DIRECTORY, names, ret, mtx, dist, rvecs, tvecs)
```

<div style="text-align: center;"> Program 3: “saveCalib” and “savePoint” function definitions </div> 

```python
def saveCalib(directory, names, ret, mtx, dist, rvecs, tvecs):
    with open(directory + '/calib.npy', 'wb') as f:
        np.save(f, names)
        np.save(f, ret)
        np.save(f, mtx)
        np.save(f, dist)
        np.save(f, rvecs)
        np.save(f, tvecs)

def savePoint(directory, names, objpoints, imgpoints):
    with open(directory + '/point.npy', 'wb') as f:
        np.save(f, names)
        np.save(f, objpoints)
        np.save(f, imgpoints)
```

An example of a calibration result is shown in figure 5. The x focal length and y focal length were calculated by converting the relevant intrinsic matrix component with the image sensor’s dimensions and pixel resolution, effectively converting the focal length given in pixel units to be given in mm units.
 
![figure5](/assets/2025-5-28-3D-Reconstruction/figure5.png "figure5"){: .img-large-centered }
<div style="text-align: center;"> Figure 5: the resulting intrinsic and extrinsic matrices of a camera calibration </div>  
<br>

## 2.4. Visualizing Distortion with Reprojection Error

To estimate the accuracy of the calibration, the reprojection error is calculated. In fact, the OpenCV “calibrateCamera” function optimizes intrinsic, extrinsic, and distortion parameters by minimizing the reprojection error. By disabling the distortion parameter optimization in the “calibrateCamera” function and plotting the reprojection error, the effects of lens distortion can be visualized. 

Reprojection error is calculated by taking the mean pixel distance between the 3D object points of the chessboard corners and the hypothetical projection of those corners to the image plane using the newly calculated camera matrix. By comparing where the corners are projected and where the corners should be, the accuracy of the transformation can be inferred. To obtain a visual representation of this process, program 4, based on the OpenCV documentation [5], draws exaggerated lines on the image to give a relative representation of reprojection error. 

<div style="text-align: center;"> Program 4: “reproject” function definition </div> 

```python
def reproject(images, objpoints, imgpoints, rvecs, tvecs, mtx, dist, show):
    mean_error = 0
    line_scaler = 50

    for i in range(len(objpoints)):
         imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
         error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
         mean_error += error

         if show == True:
            # Draw original and reprojected points
            reprojected_img = cv2.imread(images[i])
            for pt_orig, pt_proj in zip(imgpoints[i].reshape(-1, 2), imgpoints2.reshape(-1, 2)):
                pt_len = [ pt_orig[0] + (pt_proj[0]-pt_orig[0])*line_scaler, 
pt_orig[1] + (pt_proj[1]-pt_orig[1])*line_scaler ]
                cv2.circle(reprojected_img, (int(pt_orig[0]), int(pt_orig[1])), 20, (255, 0, 0), -1)
                cv2.line(reprojected_img, (int(pt_orig[0]), int(pt_orig[1])), 
(int(pt_len[0]), int(pt_len[1])), (255, 0, 0), 10)

            cv2.imshow('Reprojected', reprojected_img)
            cv2.waitKey(0)

    cv2.destroyAllWindows()
    return mean_error/len(objpoints)
```

Looping through each of the images, the OpenCV function “reprojectPoints” is used to project object points to the image plane, and the “norm” function is used to calculate the absolute distance between the image points and the projected points. Circles are drawn around the image points, and lines are drawn to the projected points, with their lengths scaled by 50 pixels to make them easily visible. 

The two images shown in figure 6, are examples of images that have been calibrated with distortion correction disabled. The image on the left is an example of a good reprojection, and a particularly good example of positive radial distortion, a type of lens aberration where the image seems to bulge towards the lens and points are projected farther from the center than they should be. The lines in the left image indicate that the projected points were generally closer to the center than the actual image points, which would fall in line with the properties of positive radial distortion. Additionally, radial distortion has a tendency for the reprojection errors to become larger the farther they are from the center of the image, which can generally be seen in both images. 

<div class="image-row-two">
  <div class="image-column-two">
    <img src="{{ '/assets/2025-5-28-3D-Reconstruction/figure6left.png' | relative_url }}" alt="">
  </div>
  <div class="image-column-two">
    <img src="{{ '/assets/2025-5-28-3D-Reconstruction/figure6right.png' | relative_url }}" alt="">
  </div>
</div>

<div style="text-align: center;"> Figure 6: Reprojection error with distortion correction disabled </div>  
<br>

Enabling the “calibrateCamera” distortion correction, reprojecting the images again gives the images shown in figure 7. It can be noted that the reprojection errors have been significantly reduced.

<div class="image-row-two">
  <div class="image-column-two">
    <img src="{{ '/assets/2025-5-28-3D-Reconstruction/figure7left.png' | relative_url }}" alt="">
  </div>
  <div class="image-column-two">
    <img src="{{ '/assets/2025-5-28-3D-Reconstruction/figure7right.png' | relative_url }}" alt="">
  </div>
</div>

<div style="text-align: center;"> Figure 7: Reprojection error with distortion correction enabled </div>  
<br>

## 2.5. Plotting Camera Pose from Calibration Parameters

To ensure that there are no obvious issues with the extrinsic parameters, it is useful to derive the camera pose from the extrinsic parameters and plot them in a 3D coordinate system. The derivation of this transformation begins with a reconfirmation of the relationship between the world coordinate system and the camera coordinate system in the current model. The extrinsic matrix is a joint rotation-translation matrix which is broken down as follows,

$$
\left[\begin{matrix}X_c\\Y_c\\Z_c\\\end{matrix}\right]=\left[\begin{matrix}r_{11}&r_{12}&r_{13}\\r_{21}&r_{22}&r_{23}\\r_{31}&r_{32}&r_{33}\\\end{matrix}\right]\left[\begin{matrix}X_w\\Y_w\\Z_w\\\end{matrix}\right]+\left[\begin{matrix}t_x\\t_y\\t_z\\\end{matrix}\right].
$$

For simplicity, the rotation matrix is represented as R and the translation vector is represented as t. The world coordinate is the unknown, so it is rewritten as P_W, and the camera coordinate is rewritten as the camera’s position in the camera coordinate system, the origin, and thus the zero vector 0. Therefore, the new expression is written as

$$
\mathbf{0}=RP_W+t,
$$

Which is simplified to

$$
P_W=-R^{-1}t.
$$

Here, the fact that for a rotation matrix $R$, its inverse $R^{-1}$ and its transpose $R^\top$ are the same matrix is used to further simplify the above equation into expression (5).

$$
\begin{equation} P_W=-R^\top t \end{equation}
$$

With expression (5), the camera pose for a given extrinsic matrix can be calculated trivially and plotted with an extension such as matplotlib. Program 5 shows the implementation of this method in python.

<div style="text-align: center;"> Program 5: python file to plot camera pose on a 3D matplotlib graph </div> 

```python
import numpy as np
import cv2
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d

from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

from definitions import *

BOARD = np.array([9, 12])
names, ret, mtx, dist, rvecs, tvecs = loadCalib('batch1')

# instantiate figure
fig = plt.figure()

# instantiate axis
ax = plt.axes(projection='3d')
ax.set_xlim([-40, 40])
ax.set_ylim([-40, 40])
ax.set_zlim([0  , 40])

# Draw the checkerboard on the floor
p = Rectangle((0, 0), BOARD[0], BOARD[1])
ax.add_patch(p)
art3d.pathpatch_2d_to_3d(p, z=0, zdir="z")

for i in range(len(rvecs)):
    # rodrigues rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvecs[i])
    # find the camera center
    C = -np.dot(R.T, tvecs[i])
    # print the basis vectors of the camera coordinate system
    ax.quiver(C[0], C[1], -C[2], R[0, 0], R[0, 1], -R[0, 2], color="red")
    ax.quiver(C[0], C[1], -C[2], R[1, 0], R[1, 1], -R[1, 2], color="green")
    ax.quiver(C[0], C[1], -C[2], R[2, 0], R[2, 1], -R[2, 2], color="blue")
    
plt.show()
```

The resulting camera pose graph can be seen in figure 8, where the basis vector in blue represents the z-axis of the camera coordinate system, and thus the direction of the camera in 3D space. Additionally, the blue square at the bottom of the graph represents the chessboard pattern in 3D space.
 
 
![figure8](/assets/2025-5-28-3D-Reconstruction/figure8.png "figure8"){: .img-large-centered }
<div style="text-align: center;"> Figure 8: camera pose derived from the 14 extrinsic parameters of batch 1 </div>  
<br>

It can be observed that the camera poses are all pointing towards the chessboard pattern, and further examination reveals that their positions and orientations seem to match the expected camera poses of the individual images. The first batch of sample images has been highlighted in detail within this report, but other samples such as the 3 samples shown in figure 9, were also taken to determine the quantitative effects of altering the focal lengths on the camera’s intrinsic matrix. As the focal lengths increase from left to right in figure 9, it was necessary to take the images farther from the checkerboard pattern in order to retain focus, which seems to be depicted with great accuracy in the resulting camera poses.

<div class="image-row-three">
  <div class="image-column-three">
    <img src="{{ '/assets/2025-5-28-3D-Reconstruction/figure9left.png' | relative_url }}" alt="">
  </div>
  <div class="image-column-three">
    <img src="{{ '/assets/2025-5-28-3D-Reconstruction/figure9mid.png' | relative_url }}" alt="">
  </div>
  <div class="image-column-three">
    <img src="{{ '/assets/2025-5-28-3D-Reconstruction/figure9right.png' | relative_url }}" alt="">
  </div>
</div>

<div style="text-align: center;"> Figure 9: camera pose derived from extrinsic parameters with focal length 0.5, 0.75, 1.0 from left to right </div>  
<br>

# 3. 3D Reconstruction from Video
## 3.1. Motivation

The need to convert seamlessly between the real and virtual world is not just a concern for professionals, and can be expected to play an important role in the everyday lives of ordinary people. However, a challenge arises in the difficulty of collecting sample data, as current 3D reconstruction techniques require specific and expensive equipment that would be unfeasible to implement at the consumer level. Further advancement of 3D reconstruction methods must also entail an increase in their accessibility and ease of use.

One implication of technological advancements in recent years is that, although the ordinary person may not have a high-end digital camera, the ordinary person is increasingly likely to have a smartphone. Additionally, videos can be seen as a discrete array of images. Thus, a new method to conduct 3D reconstruction by extracting the non-blurry frames from a video of an object taken with a smartphone, is considered in this report. Considering that videos generally have around 30 to 60 frames per second, the difference between adjacent frames is typically insignificant in the context of SFM (Structure from Motion). Additionally, videos use auto focus to maintain focus on a subject, but their control systems are typically based off of feedback systems and have latencies and result in an uneven distribution of focus throughout the frames of a video. A consequence of this is that most frames in a video with motion are blurry. Although this is not a concern when played back with real time speed, a blurry image for 3D reconstruction implies that the resulting point cloud contains uncertainty. Depending on its severity, the algorithm may decide that the point is too uncertain to even plot. Although 3D reconstruction is not conducted in this report, a method to extract the non-blurry frames from an image using Laplacian blur detection is explored.

## 3.2. Sampling Images from a Video

There are two main methods for detecting if a given image is blurry. The first method involves comparing the variance of images that have been converted to gray scale and ran through a Laplacian filter. Details of this method are given in section 2.2.1. The second method involves comparing the magnitude spectrums of images that have undergone a Fourier transform, and determining the ratio of high to low-frequency components of the image. This method is not discussed or implemented in this research.

### 3.2.1. Laplacian Variance Method

The Laplacian filter is a convolution filter that can be described as a discrete second-order derivative of an input image. The Laplacian filter is typically used as a method to quantitatively describe regions of rapid intensity change in an image, making it an ideal fit for edge-detection applications. Because the primary distinction between blurry and non-blurry images are the sharpness of their edges, the Laplacian filter can also be used to describe the relative blurriness of an image. In particular, the variance of an image that has been passed through the Laplacian filter can be used to compare the level of blur between similar images. It is important to consider however, that the Laplacian variance is only meaningful when taken with respect to images of similar composition, as this method is, for instance, incapable of distinguishing between sharp but monotone images and unsharp but colorful images.

The Laplacian filter method described in the OpenCV documentation [6] uses a 3 by 3 convolution kernel of the form

$$
\left[\begin{matrix}0&1&0\\1&-4&1\\0&1&0\\\end{matrix}\right].
$$

The implementation is given in program 6. The “GetVar” function takes an image as an input, converts the image to gray scale and returns Laplacian variance by using the OpenCV “cv2.Laplacian()” function and calling the NumPy “var()” method on the resulting NumPy array. The “GetLaplacians” function takes the path to a video and an output folder as inputs and calculates the Laplacian variance of each frame in a video, storing the resulting values in a NumPy file.

<div style="text-align: center;"> Program 6: “GetVar” and “GetLaplacian” function definitions </div> 

```python
def GetVar(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    return laplacian_var

def GetLaplacians(video_path, output_folder):
    laplacians = []
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video {video_path}")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        laplacians.append(GetVar(frame))
    cap.release()
    with open(output_folder, 'wb') as f:
        np.save(f, laplacians)
```

### 3.2.2. Filtering Laplacian Variance for Optimal Image Selection

Program 6 produces a NumPy array that contains the Laplacian variance values of each frame in a video. The next step is to find a method to optimally select images based on these values. As mentioned previously, the Laplacian variance values themselves have no inherent meaning, and only have meaning relative to the values of adjacent frames. Thus, simply selecting the top N highest Laplacian variance values from a video will not necessarily produce good results. Additionally, the images that are selected in this process are ultimately used in SFM, where adjacent frames should be relatively near each other in order to effectively find correspondences between them. It is thus, also important to consider the interval of the selected frames for optimal results. The three main filters proposed in this research are the Decay Filter, the Moving Maximum Filter, and the Local Maxima Filter.

### 3.2.3. Decay Filter

Program 7 shows the implementation of the decay filter. The decay filter takes a threshold and a decay constant as inputs. The search begins by setting the moving threshold to the input threshold. Looping through all elements of the array, while the current value does not meet the threshold, the threshold is decreased by the decay constant, and the next value is checked. When a value that meets the threshold is found, the moving threshold is set back to the initial input threshold, and the process is repeated.

<div style="text-align: center;"> Program 7: “DecayFilter” function definition </div> 

```python
def DecayFilter(laplacians, blur_threshold=100.0, decay_per_cycle=0.5):
    array = []
    threshold = blur_threshold
    for i in range(len(laplacians)):
        if (threshold<laplacians[i]): 
            array.append([i, laplacians[i]])
        threshold = blur_threshold if threshold<laplacians[i] else threshold - decay_per_cycle
    return np.array(array)
```

A benefit of the decay filter is that the intervals of selected frames are relatively easy to control. With fine tuning and ideal conditions, the decay filter can be described as a filter that prioritizes tight control on the interval between frames, and compromises for a pseudo interval maximum. The biggest flaw with the decay filter is that it compromises with an interval maximum, meaning it does not necessarily consider the local context of the value, and merely selects them based on their height. It is also not a full interval maximum filter because it selects frames based on an arbitrary decay cycle, meaning it does not necessarily select the highest value in a given interval.

The decay filter with threshold 100 and decay constant 0.5, applied to a 3 second demonstration video is shown in figure 10. The horizontal axis is the frame, and the vertical axis is the corresponding Laplacian variance. It is to be noted that an initial threshold of 100 is likely too high for this sample, but the equally large decay constant makes for a good demonstration of the decay filter’s interval consistency. 

![figure10](/assets/2025-5-28-3D-Reconstruction/figure10.svg "figure10"){: .img-large-centered }
<div style="text-align: center;"> Figure 10: Laplacian variance by frame overlayed with decay filter selection in red </div>  
<br>

### 3.2.4. Moving Maximum Filter

Program 8 shows the implementation of the moving maximum filter. The moving maximum filter takes a block size as input, and moves this block throughout the array, always selecting the largest value in the block. Like the moving average, the moving maximum filter soothes a large discontinuous array into a relatively continuous array. In the case of the moving maximum filter, the individual Laplacian variance values are smoothed into a function that represents the local ceiling values.

<div style="text-align: center;"> Program 8: “MovingMaximumFilter” function definition </div> 

```python
def MovingMaximumFilter(laplacians, blockSize = 10):
    array = []
    prev = -1
    for i in range(0, len(laplacians)-blockSize):
        splice = laplacians[i:i + blockSize]
        max = np.max(splice)
        arg = i + np.argmax(splice)
        if (prev != arg): 
            array.append([arg, max])
            prev = arg
    return np.array(array)
```

Program 8 loops through the values of the array and finds the maximum value in a splice of length “blockSize” starting from the current index. If this value has not yet been selected, it is appended to an array which is retuned at the end of the function. Thus, for large block sizes such as 10, it is normal for the same point to be selected multiple times. The benefit of the moving maximum filter is that it is a full interval maximum filter, meaning that it always selects the highest value in a given interval. However, to reiterate, the highest value does not necessarily indicate the sharpest frame, and therefore it is necessary to pick a block size that strikes a fine balance between the locality necessary for making the variance values meaningful, and the generality that is necessary to create a meaningful distinction from the local maxima filter that will be covered next. In one sentence, the moving maximum filter carries over the decay filter’s constant interval property, but also implements a full interval maximum filter unlike the decay filter.

The same 3 second demonstration video was filtered through a moving maximum filter with the block size set to 10, and the results are shown in figure 11. It can be observed that the moving maximum filter generally selects ideal values, but there are areas where needless selections occur, and others where perfectly good candidates are ignored because of their higher neighbors. 

![figure11](/assets/2025-5-28-3D-Reconstruction/figure11.svg "figure11"){: .img-large-centered }
<div style="text-align: center;"> Figure 11: Laplacian variance by frame overlayed with moving maximum filter selections in red </div>  
<br> 

### 3.2.5. Local Maxima Filter

Program 9 shows the implementation of the local maxima filter. If the two adjacent array elements are both less than the current array element, the value is selected. 

<div style="text-align: center;"> Program 9: “LocalMaximaFilter” function definition </div> 

```python
def LocalMaximaFilter(laplacians):
    array =  []
    for i in range(1, len(laplacians)-1):
        if (laplacians[i-1]<laplacians[i] and laplacians[i]>laplacians[i+1]):
            array.append([i, laplacians[i]])
    return np.array(array)
```

The local maxima filter is likely to be the best method for this application because it follows the principles of auto focus in smartphone videos. Video auto focus in many smartphone cameras rely on passive feedback to constantly adjust the focus based on the previous frame, and thus include an inherent latency. In most videos, motion of an object or motion of the camera induces a pattern of, relatively in focus, relatively out of focus, relatively in focus, and so on… An example of this effect is shown in figure 12, where three adjacent frames and their Laplacian variances are displayed. 

![figure12](/assets/2025-5-28-3D-Reconstruction/figure12.png "figure12"){: .img-large-centered }
<div style="text-align: center;"> Figure 12: Auto focus for video has the tendency to alternate between focused and non-focused frames </div>  
<br> 

The benefit of the local maxima filter is its ability to select all the points in relative focus. However, selecting all points relative to their neighbors can simultaneously be hazardous as very blurry photos that are locally non-blurry can easily be selected on accident.
The 3 second demonstration video was filtered through the local maxima filter and the results are shown in figure 13. It can be observed that several points with insignificant local maxima have also been selected.

![figure13](/assets/2025-5-28-3D-Reconstruction/figure13.svg "figure13"){: .img-large-centered }
<div style="text-align: center;"> Figure 13: Laplacian variance by frame overlayed with local maxima filter selections in red </div>  
<br> 

## 3.3. Comparing the Results

This section shows the results of a comparison between frames that were selected by the local maxima filter and those that were not selected. Figure 14 shows 4 frames that were selected from a longer 45 second video, and Figure 15 shows 4 frames that were not selected from the same video. Samples were taken randomly from the same frame range.

![figure14](/assets/2025-5-28-3D-Reconstruction/figure14.png "figure14"){: .img-large-centered }
<div style="text-align: center;"> Figure 14: frames selected by the local maxima filter  </div>  
<br> 
 
![figure15](/assets/2025-5-28-3D-Reconstruction/figure15.png "figure15"){: .img-large-centered }
<div style="text-align: center;"> Figure 15: frames not selected by the local maxima filter </div>  
<br> 

It can be noted with some close inspection that the frames that were not selected by the local maxima filter are generally all blurry, while those that were selected by the filter can occasionally be blurry. It can be concluded from these results that the local maxima filter is relatively not selective, resulting in a tendency for false positives. On the other hand, the highly non-selective nature of the filter ensures that almost all of the non-blurry frames are indeed selected. This is a good result, as applying a second filter to further refine the results and discard the false positives becomes a possibility.
 
# 4. Conclusion

In conclusion, as the amount and the quality of the information we share with the world continues to change and grow, the ways in which we convey them, the information medium, is also expected to change and grow. A direct consequence of this is that the demand for a means to convert between the real world and the virtual world increases rapidly. One way to potentially meet this demand is to introduce new technologies that increase the accessibility of 3D reconstruction models. This report suggested a new approach to select relatively good frames from a smartphone video, and detailed an implementation of the OpenCV camera calibration model which is a critical component of the 3D reconstruction pipeline.

# 5. References

[1]. MathWorks, “カメラキャリブレーションとは”, https://jp.mathworks.com/help/vision/ug/camera-calibration.html, Accessed: 7/2/24

[2]. Stanford University, “Homogeneous Coordinates”, https://ai.stanford.edu/~birch/projective/node4.html, Accessed: 6/21/24

[3]. OpenCV, “Camera Calibration and 3D Reconstruction”, https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html, Accessed: 6/21/24

[4]. Mark Hadley Jones, “Calibration Checkerboard Collection”, https://markhedleyjones.com/projects/calibration-checkerboard-collection, referenced 5/10/2024 

[5]. OpenCV, “Camera Calibration”, https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html, Accessed: 6/21/24

[6]. OpenCV, “Image Filtering”, https://docs.opencv.org/4.x/d4/d86/group__imgproc__filter.html#gad78703e4c8fe703d479c1860d76429e6, Accessed: 8/2/24