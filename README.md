# Object-Labeling-Toolkit

Toolkit for the labeling of objects within sequences of RGB-D observations.

Copyright (C) 2015 Jose Raul Ruiz Sarmiento

University of Malaga (jotaraul@uma.es)

http://mapir.isa.uma.es/jotaraul

Toolkit structure
--------
This project is structured as follows:
* _This directory_: Rules to build the toolkit using CMake, and the license file (GNU General Public License v3).
* _src_: Source files to be compiled, which build the following toolkit applications (listed in the same order as they should be run):
    * *Process_rawlog*: Sets the extrinsic and intrinsic parameters of the sensors used within the dataset.
    * *Localization*: Localizes the poses/locations from which the observations within the datset were taken.
    * *Sequential_visualization*: Visually shows a 3D reconstruction of the collected data, and stores it as a _scene_.
    * *Label_scene*: Permits us to effortlessly label a reconstructed scene.
    * *Label_rawlog*: Propagates the annotated labels in a scene to each observation within the dataset.
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
