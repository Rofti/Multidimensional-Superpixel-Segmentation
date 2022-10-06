# Multidimensional Superpixel Segmentation
## Extensions of well known superpixel algorithms for mineral thin sections

This repository contains the implementations of the superpixel segmentation algorithms ERS [2], SLIC [3,4], ETPS [5] and ERGC [6,7] as well as their extensions for multidimensional 
input images. The adaptations were originally designed to find superpixel segmenentation for cross polarized images of mineral thin-sections taken by a petrographic microscope.
Instead of a traditional single image input we are given a plain light image as well as multiple cross polarized images with varying angles. 
This repository aims to provide a tool useful in annotating large amounts of data to create a usable data set for more advanced algorithms and deep learning approaches.

The multi-dimensional version of SLIC was is introduced in [17] [[doi](https://doi.org/10.1016/j.cageo.2022.105232)] and a detailed explanation as well as a comparison to other superpixel segmentation algorithms can be found there.
The extensions of ERS, ETPS and ERGC follow the same idea but were only part of a prior master thesis, therefore, apart from the respective papers of the base algorithms, no further information is publicly available.
It is possible to provide the master thesis upon request.

**Please cite the following work if you use the provided implementations:**

    [17] J. Yu, F. Wellmann, S.Virgo, M. von Domarus, M. Jiang, J. Schmatz, B. Leibe.
        Superpixel segmentations for thin sections: Evaluation of methods to enable the generation of machine learning training data sets.
        Computers & Geosciences, 2022.


#### Special Mention
A special thanks and mention goes to David Stutz. 
This repository is using tools and extending his Superpixel Benchmark [1], a large-scale comparison of state-of-the-art superpixel algorithms. 
Much of the code consist of modification to his framework or the respective algorithms.
The paper can be found on [ArXiv](https://arxiv.org/abs/1612.01601) and the source code on [GitHub](https://github.com/davidstutz/superpixel-benchmark).
For any more information on the benchmark please visit his [Project Page](http://davidstutz.de/projects/superpixel-benchmark/).

    [1] D. Stutz, A. Hermans, B. Leibe.
        Superpixels: An Evaluation of the State-of-the-Art.
        Computer Vision and Image Understanding, 2018.


## Building

Building the components is done using [CMake](https://cmake.org/) and all of them are based
on the `lib_eval` component which holds the benchmark as well as tools.

### Prerequisites

A C++ compiler supporting C++11 is assumed to be available.
It was tested with gcc >= 4.8.4. Building has been tested with Ubuntu 20.04.

All algorithms depend on the tools in `lib_eval`. Requirements are:

* [CMake](https://cmake.org/), 
* [OpenCV](http://www.boost.org/)
* [Boost](https://github.com/google/glog)
* [GLog](https://github.com/google/glog).

Additionally, the algorithms built by default depend on:

* [PNG](http://www.libpng.org/pub/png/libpng.html)
* [PNG++](http://www.nongnu.org/pngpp/)

Note that the required CMake modules, e.g. for finding GLog, can
be found in `cmake` in case these need to be adapted. 

CMake, OpenCV and Boost can be installed as follows:

    $ sudo apt-get install build-essential cmake libboost-dev-all libopencv-dev

OpenCV can alternatively be installed following [these instructions](http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation).
Currently, OpenCV 2.4.11 is supported, in general 2.4.x should work fine. For upgrading
to OpenCV 3.0 and OpenCV 3.1, it might be necessary to update constants (e.g. as used for
color conversion). Some implementations are known to work with OpenCV 3 and OpenCV 3.1.

GLog 0.3.3 should be installed manually. GLog should be downloaded or cloned from [google/glog](https://github.com/google/glog).
If the repository is cloned, make sure to checkout version 0.3.3. Then:

    $ cd glog-0.3.3/
    $ ./configure
    $ make
    $ sudo make install

For further instructions see the issue tracker at [google/glog](https://github.com/google/glog).

Note that GLog can alternatively be installed using

    $ sudo apt-get install libgoogle-glog-dev

However, `cmake/FindGlog.cmake` needs to be adapted and some parts might not working
with newer versions.

For installing PNG and PNG++:

    sudo apt-get install libpng-dev # should already be installed for OpenCV
    sudo apt-get install libpng++-dev

As reference, these are the library versions as installed on Ubuntu 14.04 (checked using `dpkg -l`)
where OpenCV and GLog where installed manually:

    ||/ Name                Version        Architecture  
    +++-===================-==============-==============
    ii  gcc                 4:9.3.0-1ubuntu2 amd64
    ii  cmake               3.16.3-1ubuntu1  amd64
    ii  libboost-dev        1.71.0.6ubuntu6  amd64
    ii  libpng12-dev        1.2.50-1ubuntu1  amd64
    ii  libpng++-dev        0.2.10-1         all

## Building Options

After verifying that the requirements are met:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ cmake -LAH

This will list all available CMake options. These options include:

* `-DBUILD_ERS`: build ERS (On)
* `-DBUILD_ETPS`: build ETPS (On)
* `-DBUILD_SLIC`: build SLIC (On)
* `-DBUILD_ERGC`: build ERGC (Off)

#### Prerequisites for ERGC

[CImg](http://cimg.eu/) and [LAPACK](http://www.netlib.org/lapack/) is required:

    sudo apt-get install cimg-dev cimg-doc cimg-examples

For reference, the following version was installed on Ubuntu 20.04:

    ||/ Name                Version        Architecture
    +++-===================-==============-==============
    ii  cimg-dev            2.4.5+dfsg-1    all


## Executables

### Algorithms in C++

All command line tools for algorithms in C++ have the following options in common:

    $ ../bin/ers_cli --help
    Allowed options:
      -h [ --help ]                   produce help message
      -i [ --input ] arg              folder containing the images to process
      # Algorithm specific options ...
      -d [ --dimensions ] arg         number of subimages, 1 for traditional superpixel algorithm ,  > 1 for the multi-dimensional version
      -o [ --csv ] arg                save segmentation as CSV file
      -v [ --vis ] arg                visualize contours
      -x [ --prefix ] arg             output file prefix
      -w [ --wordy ]                  verbose/wordy/debug

`--input` is additionally a positional option. The algorithm specific options can
be displayed using the `--help` options. For details on the specific options, the reader
is referred to the corresponding publication(s).

The `--csv` option will output the superpixel segmentations as `.csv` files in
the provided directory, which is created if it does not exist. The naming follows
the naming of the images found in the directory specified by `--input`. Similarly,
`--vis` outputs visualizations in the provided directory, which is also created
if it does not exist.

`--prefix` can be used to specify a prefix, then the output files (CSV files and
visualizations) are prefixed with the given string. `--wordy` will cause the
tool to provide more detailed output while running (i.e. be verbose).

`--dimensions` is used to signal the number of standard 2D images form one multi dimensional input.
A value of 1 would execute the traditional superpixel algorithm, while a higher value would execute the multi-dimensional extension of the algorithm.
In our work with mineral thin section the dimensionality was 11, one unpolarized image and 10 cross polarized images with varying degrees of polarization.

Examples:

    $ build
    $ cmake ..
    $ make
    # ERGC (built by default)
    $ ../bin/ergc_cli --input ../examples/ --superpixels 1200 --color-space 1 --perturb-seeds 0 --compacity 0 --dimensions 1 -o ../output/ergc -w
    # ETPS (built by default)
    ../bin/etps_cli --input ../examples/ --superpixels 1200 --regularization-weight 0.01 --length-weight 0.1 --size-weight 1 --iterations 25 --dimensions 1 -o ../output/etps -w

### Utilities in C++

As part of the superpixel benchmark of [1] that we build upon, several tools for evaluation are provided. All of them
are prefixed by `eval_` and only depend on `lib_eval`.

#### eval_parameter_optimization

`eval_parameter_optimization` demonstrates the parameter optimization procedure used in [1]. 

For parameter optimization, the command line tools for all algorithms provide the parameters `-i` and `-o` for input and output.
Having a close look at `eval_parameter_optimization_cli/main.cpp` shows that 
the parameters for the different algorithms are hard-coded. 

    $ ../bin/eval_parameter_optimization_cli --help
    Allowed options:
       --img-directory arg                   image directory
       --gt-directory arg                    ground truth directory
       --base-directory arg                  base directory
       --algorithm arg                       algorithm to optimize: ers, 
       --dimensions arg          	      image dimensions, 1 corresponds to traditional algorithm with one image ,  > 1 to the multi-dim version
       --depth-directory arg                 depth directory
       --intrinsics-directory arg            intrinsics directory
       --not-fair                            do not use fair parameters
       --help                                produce help message

#### eval_summary_cli

`eval_summary_cli` bundles all evaluation metrics. 
Given a directory containing superpixel segmentations as `.csv` files 
and directories with the corresponding images (as `.png`, `.jpg`, `.jpeg` or `.tiff`)
and ground truth segmentations (also as `.csv` files), summarizes the performance of
the superpixel segmentations. The provided options are:

    $ ../bin/eval_summary_cli --help
    Allowed options:
      --sp-directory arg    superpixel segmentation directory
      --img-directory arg   image directory
      --gt-directory arg    ground truth directory
      --append-file arg     append file
      --vis                 visualize results
      --help                produce help message



## Algorithms

An overview of the original algorithms that were extended to multidimensional input images can be found below. 
More details can be found in the respective paper or in the original README found in the respective library directory.
Also check the corresponding web pages for author and license information. 
The corresponding references are given below the table.

Algorithm    | Library       | Executable    | Implementation | Reference | Link
-------------|---------------|---------------|----------------|-----------|-----
ERS          | `lib_ers`     | `ers_cli`     | C++            | [2]       | [Web](http://mingyuliu.net/)
SLIC         | `lib_slic`    | `slic_cli`    | C++            | [3,4]     | [Web](http://ivrl.epfl.ch/research/superpixels)
ETPS         | `lib_etps`    | `etps_cli`    | C++            | [5]       | [Web](https://bitbucket.org/mboben/spixel)
ERGC         | `lib_ergc`    | `ergc_cli`    | C++            | [6,7]     | [Web](https://sites.google.com/site/pierrebuyssens/code/ergc-superpixels)
        
    [2] M. Y. Lui, O. Tuzel, S. Ramalingam, R. Chellappa.
         Entropy rate superpixel segmentation.
         IEEE Conference on Computer Vision and Pattern Recognition, 2011, pp. 2097–2104.
    [3] R. Achanta, A. Shaji, K. Smith, A. Lucchi, P. Fua, S. Susstrunk.
         SLIC superpixels.
         Tech. rep., Ecole Polytechnique Federale de Lausanne (2010).
    [4] R. Achanta, A. Shaji, K. Smith, A. Lucchi, P. Fua, S. Susstrunk.
         SLIC superpixels compared to state-of-the-art superpixel methods.
         IEEE Transactions on Pattern Analysis and Machine Intelligence 34 (11) (2012) 2274–2281.
    [5] J. Yao, M. Boben, S. Fidler, R. Urtasun.
         Real-time coarse-to-fine topologically preserving segmentation.
         IEEE Conference on Computer Vision and Pattern Recognition, 2015, pp. 2947–2955.
    [6 P. Buyssens, I. Gardin, S. Ruan.
         Eikonal based region growing for superpixels generation: Application to semi-supervised real time organ segmentation in CT images.
         Innovation and Research in BioMedical Engineering 35 (1) (2014) 20–26.
    [7] P. Buyssens, M. Toutain, A. Elmoataz, O. Lézoray.
         Eikonal-based vertices growing and iterative seeding for efficient graph-based segmentation.
         International Conference on Image Processing, 2014, pp. 4368–4372
         

## Evaluation Metrics

This project includes serveral metrics from different references focussing on different
aspects of superpixel segmentations. The implementations of these metrics were originally provided alongside the paper [1] and were only marginally modified where necessary.
Detailed equations can be found there or in their [Doxygen Documentation](https://davidstutz.github.io/superpixel-benchmark/). 


### Boundary Recall

Boundary Recall is introduced in [8] and quantifies the fraction of boundary pixels correctly captured by
a superpixel segmentation. Higher Boundary Recall describes better adherence to image boundaries.

    [8] D. Martin, C. Fowlkes, J. Malik.
        Learning to detect natural image boundaries using local brightness, color, and texture cues.
        IEEE Transactions on Pattern Analysis and Machine Intelligence 26 (5) (2004) 530–549.

### Undersegmentation Error

Undersegmentation Error was first introduced in [9] and quantifies the leakage of
superpixels across ground truth segments. However, the original formulation penalizes
large superpixels (see [9, 10]) covering multiple ground truth segments which may be misleading.
Therefore, the formulation of [11] is used. 

    [9] A. Levinshtein, A. Stere, K. N. Kutulakos, D. J. Fleet, S. J. Dickinson, K. Siddiqi.
        TurboPixels: Fast superpixels using geometric flows.
        IEEE Transactions on Pattern Analysis and Machine Intelligence 31 (12) (2009) 2290–2297.
    [10] R. Achanta, A. Shaji, K. Smith, A. Lucchi, P. Fua, S. Susstrunk.
        SLIC superpixels compared to state-of-the-art superpixel methods.
        IEEE Transactions on Pattern Analysis and Machine Intelligence 34 (11) (2012) 2274–2281.
    [11] P. Neubert, P. Protzel. 
        Superpixel benchmark and comparison.
        Forum Bildverarbeitung, 2012.

### Achievable Segmentation Accuracy

Achievable Segmentation Accuracy [12] quantifies the segmentation performance achievable
when using superpixels instead of pixels. To this end, each superpixel is assigned to the
ground truth segment with highest overlap. The number of pixels correctly classified
this way is used to compute Achievable Segmentation Accuracy.

    [12] M. Y. Lui, O. Tuzel, S. Ramalingam, R. Chellappa.
        Entropy rate superpixel segmentation.
        IEEE Conference on Computer Vision and Pattern Recognition, 2011, pp. 2097–2104.

### Explained Variation

Explained Variation [13] computes the variance within each superpixels, weights it by
the size of the superpixel and normalizes the sum by the total image variance. This
way, Explained Variation quantifies the fraction of variance within the image that
is captured (i.e. "explained") by the superpixel segmentation.

    [13] A. P. Moore, S. J. D. Prince, J. Warrell, U. Mohammed, G. Jones.
        Superpixel lattices.
        IEEE Conference on Computer Vision and Pattern Recognition, 2008, pp. 1–8.

### Compactness

Compactness [14] compares the area of each superpixel with the area of a circle with the
same perimeter.

    [14] A. Schick, M. Fischer, R. Stiefelhagen.
        Measuring and evaluating the compactness of superpixels.
        International Conference on Pattern Recognition, 2012, pp. 930–934.

Note: The implementation in `lib_eval/evaluation.h` uses a simple estimate of the
perimeter of superpixels. Therefore, results may not be comparable to those in [14].

## Intra-Cluster Variation

Intra-Cluster Variation [15] computes the average standard deviation of the superpixels.
However, this is not done in relation to the overall image variation.

    [15] W. Benesova, M. Kottman.
        Fast superpixel segmentation using morphological processing.
        Conference on Machine Vision and Machine Learning, 2014.

### Mean Distance to Edge

Mean Distance to Edge [15] averages the distance of each boundary pixel in the ground
truth segmentation to the nearest boundary pixel in the superpixel segmentation.
Assuming the distances to encode the binary relationship of true positives or false
negatives, Mean Distance to Edge resembles Boundary Recall.

    [15] W. Benesova, M. Kottman.
        Fast superpixel segmentation using morphological processing.
        Conference on Machine Vision and Machine Learning, 2014.

### Contour Density

Contour Density [16] quantifies the fraction of pixels that are boundary pixels
in the superpixel segmentation

    [16] V. Machairas, M. Faessel, D. Cardenas-Pena, T. Chabardes, T. Walter, E. Decenciere.
        Waterpixels.
        Transactions on Image Processing 24 (11) (2015) 3707–3716.
