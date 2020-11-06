## Creating a dataset of collided/uncollided iCub postures requires two steps:
1) Sample data from SDFAST iCub model via FCL collision detection library (folder `fcl-sampling`)
2) Converting resulting binary data files into `.txt` formatted for libSVM (folder `data-processing`)

Instructions for each step are provided in `README.md` files in corresponding folders.

Sampling the data takes up to 8 hours, so we offer pre-sampled datasets: https://drive.google.com/file/d/1aQ7nWlCL2JtHCax6pXXfNg707a1uIRa3/view?usp=sharing
