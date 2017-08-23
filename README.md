# Tsai Calibration

[Samuel Bailey](http://bailey.geek.nz) <[sam@bailey.geek.nz](mailto:sam@bailey.geek.nz)>


An implementation of [Tsai's](https://en.wikipedia.org/wiki/Camera\_resectioning#Tsai.27s\_Algorithm) [camera calibration](https://en.wikipedia.org/wiki/Camera_resectioning) technique.

Camera calibration is the process of estimating the parameters of a camera model order to create a model that approximates a physical camera.
In this case we use a modified [pinhole camera model](https://en.wikipedia.org/wiki/Pinhole_camera_model) with corrections applied to account for [radial lens distortion](https://en.wikipedia.org/wiki/Distortion_(optics)#Radial_distortion).

Tsai's technique uses a 2-stage process, where the parameters of a pinhole camera model are estimated using a linear optimisation process (least-squares or SVD).
Using a process known as [bundle adjustment](https://en.wikipedia.org/wiki/Bundle_adjustment) the model is further refined to take into account lens distortion using nonlinear least-squares optimisation (gradient descent).


## Requires
 * Python 2.x
   * SciPy (and NumPy)
   * Matplotlib


## Command Line:
python main.py > output.txt
