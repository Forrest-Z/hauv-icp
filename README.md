# README #
This repository is for testing the three ICP variants for the hauv-slam project.

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