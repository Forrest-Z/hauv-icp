# README #
This repository is for testing the three ICP variants for the hauv-slam project, as well as subsequent detection of degenerate transformations by calculating eigen values of the ATA matrix from the nonlinear optimization of the ICP solution. This is used to check for bad loop-closures and improve them by using odometry information for degenerate directions and thus creating a "partial" factor in the factor graph SLAM framework (solved using iSAM).

### What is this repository for? 

* Implementing 2D-ICP, point-to-plane ICP, and point-to-plane ICP with an added odometry prior
 
### How do I get set up? ###
* To download datasets, run the files in "scripts":

```
./download_dataset_shiphull.sh
./download_dataset_cube.sh
```

* To build:

```
mkdir build
cd build
cmake ..
make 
```

* Run example:

```
./main -o ../data/dataset_shiphull/pair_2-3  
```
* -o	: 	point-to-plane ICP with odometry prior
* -p	: 	point-to-plane ICP 
* -t  	: 	2D ICP

### Who do I talk to? ###

* Tushar Kusnur
* kusnur.tushar@gmail.com
