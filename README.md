# Tsai Calibration

[Samuel Bailey](http://bailey.geek.nz) <[sam@bailey.geek.nz](mailto:sam@bailey.geek.nz)>


An implementation of [Tsai's](https://en.wikipedia.org/wiki/Camera\_resectioning#Tsai.27s\_Algorithm) [camera calibration](https://en.wikipedia.org/wiki/Camera_resectioning) technique.

Camera calibration is the process of estimating the parameters of a camera model order to create a model that approximates a physical camera.
In this implementation we use a modified [pinhole camera model](https://en.wikipedia.org/wiki/Pinhole_camera_model) with corrections applied to account for [radial lens distortion](https://en.wikipedia.org/wiki/Distortion_(optics)#Radial_distortion).
Photographs of an object with known geometry (a calibration object) are used to estimate the parameters using optimization techniques.

The most popular method for low-cost camera calibration is Zhang's technique\[2\] - which uses several photographs to calibrate the camera. This can be problematic in some applications (for example, when battery life is limited).
In contrast, Tsai's technique\[1\] uses a single photograph. A downside to this approach is that the points used in Tsai calibration cannot lie on the same plane, so the calibration object must be more complex than the simple checkerboards typically used in Zhang calibration.

To estimate the parameters of the model, a 2-stage process is used. First, the parameters of a linear pinhole camera model are estimated using a linear optimisation process (least-squares or SVD).
In the second stage, the model is extended to estimate the nonlinear lens distortion parameters. Parameters for the resulting model can then be found using a nonlinear optimisation process (gradient descent).
The resulting algorithm is much faster than the traditional methods used in photogrammetry that simultaneously optimize all the parameters using nonlinear methods.


## Requires
 * [Python 2.x](https://www.python.org/)
   * [SciPy](https://www.scipy.org/) (Math library)
   * [Matplotlib](https://matplotlib.org/) (2D/3D plotting library)


## Command Line
~~~~
python main.py > output.txt
~~~~


## References
 1. R. Tsai, "A versatile camera calibration technique for high-accuracy 3D machine vision metrology using off-the-shelf TV cameras and lenses," in IEEE Journal on Robotics and Automation, vol. 3, no. 4, pp. 323-344, August 1987.
     web: [ieee.org](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087109&isnumber=23638) [sci-hub.io](http://sci-hub.io/10.1109/JRA.1987.1087109) doi: [10.1109/JRA.1987.1087109](https://doi.org/10.1109/JRA.1987.1087109)
 2. Z. Zhang, "A Flexible New Technique for Camera Calibration," in IEEE Transactionson Pattern Analysis and Machine Intelligence, Vol. 22, No. 11, pp. 1330-1334, 2000.
     web: [microsoft.com](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/11/zhan99.pdf) [sci-hub.io](http://sci-hub.io/10.1109/34.888718) doi: [10.1109/34.888718](https://doi.org/10.1109/34.888718)
 3. W. Li, T. Gee, H. Friedrich, and P. Delmas, “A practical comparison between Zhang’s and Tsai’s calibration approaches,” in Proceedings of the 29th International Conference on Image and Vision Computing New Zealand, ser. IVCNZ ’14, Hamilton, New Zealand: ACM, pp. 166–171, 2014. ISBN: 978-1-4503-3184-5.
     web: [acm.org](http://doi.acm.org/10.1145/2683405.2683443) [sci-hub.io](http://sci-hub.io/10.1145/2683405.2683443) doi: [10.1145/2683405.2683443](https://doi.org/10.1145/2683405.2683443)


## See Also
 * Manuel E. L. Fernandez, Lucas Teixeira  and Marcelo Gattass, [ANSI C Implementation of Classical Camera Calibration Algorithms: Tsai and Zhang](http://webserver2.tecgraf.puc-rio.br/~mgattass/calibration/)
 * For calibration without any special objects in the scene, see [Camera auto-calibration](https://en.wikipedia.org/wiki/Camera_auto-calibration)