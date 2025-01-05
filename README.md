# cloth-simulation
Final Project of CS171 in ShanghaiTech

## Log 

### xjx-dev

- Upload Eigen 3.4.0 from [https://gitlab.com/libeigen/eigen/-/releases/3.4.0](https://gitlab.com/libeigen/eigen/-/releases/3.4.0).

- Update cmakelists to support `eigen` and `openmp`.

- Finish constructor of `RectClothSimulator`.

- Finish `fast_step`. However, I have got some problems:
  1. In the local step, how to fix `x` and solve for `d`?
  2. When calculate `externalForceVector`, we have got to assign a negative sign, why is that?

- Fix a bug in scratch.

- (newest) To add the collision with a sphere.