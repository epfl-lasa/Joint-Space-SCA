# Visualisation
![alt text](https://github.com/epfl-lasa/Joint-Space-SCA/blob/main/Dataset-Creation/fcl-sampling/sampling_vis.gif)
# Dependencies
1) Eigen3 - `sudo apt install libeigen3-dev`
2) CCD - https://github.com/danfis/libccd
3) FCL - https://github.com/flexible-collision-library/fcl
4) SDFast - which is a commercial software, and therefore can not be put into this repo. Please contact Michael Koptev after acquiring the license for SDFast sources.

# Building
`mkdir build && cd build && cmake .. && make -j8`

# Usage
As we have 10 submodels to describe self-collsions for humanoid robot iCub, we need to generate 10 datasets.
The first parameter specifies submodel number, and the second - desired size of the dataset
For example, to generate 100000 datapoints for specific model (let's say 1 - describing both arms), one should launch `./main 1 100000`. Code is not parallelized, but it can be achieved by launching multiple instances in two terminals, `./main 1 100000` + `./main 1 100000` would generate two separate datasets of 100k size, that can later be merged into single dataset on the processing stage.

# Precomputed datasets
Achieved by running `./main i 100000` ten times for i=1..10. In total 100 runs (10x100,000 for i = 1..10).
https://drive.google.com/file/d/1B-_hR59ftv93V3_rvtORBuVH7i15BJxZ/view?usp=sharing
