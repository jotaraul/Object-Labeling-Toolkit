# Object-Labeling-Toolkit

Toolkit for the labeling of objects within sequences of RGB-D observations.

Copyright (C) 2015-2016 Jose Raul Ruiz Sarmiento

University of Malaga (jotaraul@uma.es)

http://mapir.isa.uma.es/jotaraul

If you use this software, please read the LICENSE file. If the utilization is for academic purposes, please cite the work:

>  @INPROCEEDINGS{Ruiz-Sarmiento-ECMR-2015,

>     author = {Ruiz-Sarmiento, J. R. and Galindo, Cipriano and Gonz{\'{a}}lez-Jim{\'{e}}nez, Javier},

>     title = {OLT: A Toolkit for Object Labeling Applied to Robotic RGB-D Datasets},

>     booktitle = {European Conference on Mobile Robots},

>     year = {2015},

>     location = {Lincoln, UK}

>  }


Toolkit structure
--------
This project is structured as follows:
* _This directory_: Rules to build the toolkit using CMake, and the license file (GNU General Public License v3).
* _src_: Source files to be compiled, which build the following toolkit applications (listed in the same order as they should be run):
    * *Process_rawlog*: Sets the extrinsic and intrinsic parameters of the sensors used within the dataset.
    * *Mapping*: Localizes the poses/locations from which the observations within the datset were taken in order to create a map of the scene.
    * *Visualize_reconstruction*: Visually shows a 3D reconstruction of the collected data, and stores it as a _scene_.
    * *Label_scene*: Permits us to effortlessly label a reconstructed scene. Example of scene being labeled:
    
        ![Scene being labeled with the OLT tool](http://mapir.isa.uma.es/jotaraul/Resources/example_scene.png "Scene being labeled with the OLT tool")

    * *Label_rawlog*: Propagates the annotated labels in a scene to each observation within the dataset.
    * *Dataset_statistics*: Shows information of the dataset, e.g. a summary of the objects appearing on it, number of times that they appear, number of pixels they occupy, etc.
    * *Benchmark*: Compare two labeled rawlogs and yields some performance results.
    * *Create_video & Segmentation*: Experimental applications under development.
* _Examples of configuration files_: A directoy containing some examples of configuration files to be used with the different toolkit applications.

Prerequisites
--------

Mandatory dependencies:

* MRPT (www.mrpt.org)
* PCL (http://www.pointclouds.org/)

Compiling
--------

mkdir build && cd build && cmake .. && make
