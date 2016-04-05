/*---------------------------------------------------------------------------*
 |                         Object Labeling Toolkit                           |
 |            A set of software components for the management and            |
 |                      labeling of RGB-D datasets                           |
 |                                                                           |
 |            Copyright (C) 2015-2016 Jose Raul Ruiz Sarmiento               |
 |                 University of Malaga <jotaraul@uma.es>                    |
 |             MAPIR Group: <http://http://mapir.isa.uma.es/>                |
 |                                                                           |
 |   This program is free software: you can redistribute it and/or modify    |
 |   it under the terms of the GNU General Public License as published by    |
 |   the Free Software Foundation, either version 3 of the License, or       |
 |   (at your option) any later version.                                     |
 |                                                                           |
 |   This program is distributed in the hope that it will be useful,         |
 |   but WITHOUT ANY WARRANTY; without even the implied warranty of          |
 |   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            |
 |   GNU General Public License for more details.                            |
 |   <http://www.gnu.org/licenses/>                                          |
 |                                                                           |
 *---------------------------------------------------------------------------*/

#include "CEditor.hpp"
#include <cmath>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CText3D.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/utils.h>
#include <mrpt/poses/CPose3D.h>

#ifdef USING_OPENCV
#include <opencv2/opencv.hpp>
#endif


using namespace OLT;
using namespace std;

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::poses;


int CSaveAsPlainText::process()
{
    if ( m_iRawlog.is_open() )
        return processRawlog();
    else
        return processScene();
}

int CSaveAsPlainText::processRawlog()
{
    cout << "  [INFO] Saving as human readable " << endl;
    string outputFile = ".";
    string outputObsDir = "observations";

    if (m_optionsS.count("output_file"))
        outputFile = m_optionsS["output_file"];


    if (m_optionsS.count("output_obs_dir"))
        outputObsDir = m_optionsS["output_obs_dir"];

    cout << "  [INFO] Creating directory " << outputObsDir << endl;
    mrpt::system::createDirectory(outputObsDir);

    //
    // Process rawlog
    //

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;

    cout << "  [INFO] Opening sequence file " << outputFile << endl;

    cout << "    Process: ";
    cout.flush();

    ofstream sequenceFile(outputFile.c_str());

    if (!sequenceFile.is_open())
    {
        cerr << "  [ERROR] Can't open file " << outputFile << endl;
        return 0;
    }

    sequenceFile << "# This file contains information about the (sequence of) observations in the directory with the same name. \n"
                    "# Its format is: \n"
                    "# [Observation_id] [sensor_label] [sensor_pose_x] [sensor_pose_y] [sensor_pose_z] [sensor_pose_yaw] [sensor_pose_pitch] [sensor_pose_roll] [time-stamp] \n"
                    "# Units for the sensor pose are meters and radians. The tiem-stamp holds the the number of 100-nanosecond intervals since January 1, 1601 (UTC). \n";

    while ( CRawlog::getActionObservationPairOrObservation(m_iRawlog,action,observations,obs,obsIndex) )
    {
        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        // Get info about the obs to be stored in the sequence file

        TPose3D pose;
        obs->getSensorPose(pose);
        vector<double> v_pose;
        pose.getAsVector(v_pose);

        // Store obs info in the sequence file

        sequenceFile << obsIndex << " " << obs->sensorLabel << " ";
        sequenceFile << v_pose[0] << " " << v_pose[1] << " " << v_pose[2] << " "
                     << v_pose[3] << " " << v_pose[4] << " " << v_pose[5] << " ";
        sequenceFile << obs->timestamp << endl;

        string fileName;

        if (IS_CLASS(obs, CObservation2DRangeScan))
        {

            CObservation2DRangeScanPtr obs2D = CObservation2DRangeScanPtr(obs);

            ofstream  file;

            fileName = format("%s%lu_scan.txt",outputObsDir.c_str(),obsIndex);
            file.open(fileName.c_str());


            file << "# This file contains a 2D laser scan observation. \n"
                    "# Its format is: \n"
                    "# [aperture] [max_range] [number_of_scans] \n"
                    "# [vector_of_scans] \n"
                    "# [vector_of_valid_scans] \n"
                    "# The aperture of the sensor is in radians, the maximum range \n"
                    "# and measurements are in meters. '1' means a valid scan, '0' otherwise.\n";


            file << obs2D->aperture << endl;
            file << obs2D->maxRange << endl;
            file << obs2D->scan.size() << endl;

            for ( size_t i = 0; i < obs2D->scan.size(); i++ )
                file << obs2D->scan[i] << " ";

            file << endl;

            for ( size_t i = 0; i < obs2D->validRange.size(); i++ )
                ( !obs2D->validRange[i] ) ? file << "0 " : file << "1 ";

            file << endl;

            file.close();
        }
        else if (IS_CLASS(obs, CObservation3DRangeScan))
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            ofstream  file;

            if ( m_optionsD["generate_point_clouds"] )
            {
                if ( !obs3D->hasPoints3D )
                    obs3D->project3DPointsFromDepthImage();

                fileName = format("%s%lu_points.txt",outputObsDir.c_str(),obsIndex);
                file.open(fileName.c_str());

                for (size_t i = 0; i < obs3D->points3D_x.size(); i++ )
                {
                    file << trunc(1000 * obs3D->points3D_x[i]) / 1000  <<
                            " " << trunc(1000 * obs3D->points3D_y[i]) / 1000  <<
                            " " << trunc(1000 * obs3D->points3D_z[i]) / 1000  << endl;
                }

                file.close();
            }

            if ( obs3D->hasPixelLabels() )
            {
                int N_cols, N_rows;
                obs3D->pixelLabels->getSize(N_rows,N_cols);

                file << N_rows << endl;
                file << N_cols << endl;

                fileName = format("%s%lu_labels.txt",outputObsDir.c_str(),obsIndex);
                file.open(fileName.c_str());

                file << "#This file contains a labelled matrix/mask, associated with the RGB-D images with the same id. \n"
                        "#Its format is: \n"
                        "#[number_of_labels/categories] \n"
                        "#[label_id_0] [label_string_0] \n"
                        "# ... \n"
                        "#[label_id_n] [label_string_n] \n"
                        "#[number_of_rows_of_the_matrix] \n"
                        "#[number_of_cols_of_the_matrix] \n"
                        "#[labelled_matrix/mask] \n"
                        "#The elements/pixels of the matrix must be interpreted as string of bites, \n"
                        "#so a bit with value 1 means that the element belongs to the label \n"
                        "#which label_id matchs the bit position. For example, if an element has \n"
                        "#the value 1010, this means that is belongs to the labels with id 1 and 3. \n"
                        "#In this way an element can belong to more than one label. \n";


                std::map<uint32_t,std::string>::iterator it;

                file << obs3D->pixelLabels->pixelLabelNames.size() << endl;

                for ( it = obs3D->pixelLabels->pixelLabelNames.begin();
                      it != obs3D->pixelLabels->pixelLabelNames.end(); it++ )
                {
                    file <<  it->first << " " << it->second << endl;
                }



                for ( int row = 0; row < N_rows; row++ )
                {
                    for ( int col = 0; col < N_cols; col++ )
                    {
                        uint64_t labels;
                        obs3D->pixelLabels->getLabels(row,col,labels);
                        file << static_cast<int>(labels) << " ";
                    }

                    file << endl;
                }

                file.close();
            }


            if ( obs3D->hasRangeImage )
            {

                int N_rows = obs3D->rangeImage.rows();
                int N_cols = obs3D->rangeImage.cols();

#ifdef USING_OPENCV
                {
                    uint16_t max = numeric_limits<uint16_t>::max();

                    double maxValue = 10;
                    double factor = static_cast<double>(max)/maxValue;

                    cv::Mat img(N_rows,N_cols, CV_16UC1);

                    for ( int row = 0; row < N_rows; row++ )
                        for ( int col = 0; col < N_cols; col++ )
                        {
                            double value = obs3D->rangeImage(row,col);
                            uint16_t finalValue = static_cast<uint16_t>(value*factor);

                            img.at<uint16_t>(row,col) = finalValue;
                        }

                    fileName = format("%s%lu_depth.png",outputObsDir.c_str(),obsIndex);
                    vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                    compression_params.push_back(0);

                    cv::imwrite(fileName,img,compression_params);

                    // Load to check that it's ok
                    //cv::Mat image = cv::imread(fileName,CV_LOAD_IMAGE_UNCHANGED);
                }
#else
                {


                    fileName = format("%s%lu_depth.txt",outputObsDir.c_str(),obsIndex);
                    file.open(fileName.c_str());

                    file << "#This file contains the depth image associated with the RGB image with the same id. \n"
                            "#Its format is: \n"
                            "#[number_of_rows] \n"
                            "#[number_of_cols] \n"
                            "#[matrix_of_depths] \n"
                            "#Units are meters \n";

                    for ( int row = 0; row < N_rows; row++ )
                    {
                        for ( int col = 0; col < N_cols; col++ )
                        {
                            file << trunc(1000 * obs3D->rangeImage(row,col) )/ 1000 << " ";
                        }

                        file << endl;
                    }

                    file.close();
                }
#endif

                //obs3D->rangeImage.saveToTextFile(format("./external/%d_depth.txt",obsIndex),mrpt::math::MATRIX_FORMAT_FIXED);//_convertToExternalStorage(format("./external/%d_depth.txt",obsIndex),"./");
                if ( obs3D->hasIntensityImage )
                {
                    fileName = format("%s%lu_intensity.png",outputObsDir.c_str(),obsIndex);
                    obs3D->intensityImage.saveToFile(fileName,100);
                }

            }
        }

    }

    sequenceFile.close();

    cout << endl << "  [INFO] Done!" << endl << endl;

    return 1;

}

int CSaveAsPlainText::processScene()
{
    string outputFile = ".";

    if (m_optionsS.count("output_file"))
        outputFile = m_optionsS["output_file"];

    ofstream sceneFile(outputFile.c_str());

    cout << "  [INFO] Saving as human readable in " << outputFile << endl;

    vector<string> v_labels;
    vector<vector<TPoint3D> > v_corners;
    vector<CPose3D> v_poses;

    // Load previously inserted boxes
    bool keepLoading = true;
    size_t boxes_inserted = 0;

    while ( keepLoading )
    {
        CText3DPtr text = m_scene.getByClass<CText3D>(boxes_inserted);

        if ( !text.null() )
            v_labels.push_back(text->getString());
        else
            v_labels.push_back("none");

        CBoxPtr box = m_scene.getByClass<CBox>(boxes_inserted);

        if ( box.null() )
            keepLoading = false;
        else
        {
            CPose3D pose = box->getPose();

            TPoint3D c1,c2;
            box->getBoxCorners(c1,c2);

            TPoint3D C111 ( pose + static_cast<TPose3D>(TPoint3D(c1.x,c1.y,c1.z)) );
            TPoint3D C222 ( pose + static_cast<TPose3D>(TPoint3D(c2.x,c2.y,c2.z)) );

            vector<TPoint3D> v;
            v.push_back(C111);
            v.push_back(C222);

            v_corners.push_back(v);

            v_poses.push_back(pose);
        }

        boxes_inserted++;
    }

    size_t N_corners = v_corners.size();

    // Is it a labelled scene?

    if ( N_corners > 0 )
    {
        sceneFile << "#This file contains a labelled 3D point cloud of a reconstructed scene. \n"
                     "#Its format is: \n"
                     "#[number_of_bounding_boxes/labels] \n"
                     "#[bounding_box_1_label] \n"
                     "#[bb1_pose_x] [bb1_pose_y] [bb1_pose_z] [bb1_pose_yaw] [bb1_pose_pitch] [bb1_pose_roll] \n"
                     "#[bb1_corner1_x] [bb1_corner1_y] [bb1_corner1_z] \n"
                     "#[bb1_corner2_x] [bb1_corner2_y] [bb1_corner2_z] \n"
                     "# ..."
                     "#[bbn_pose_x] [bbn_pose_y] [bbn_pose_z] [bbn_pose_yaw] [bbn_pose_pitch] [bbn_pose_roll] \n"
                     "#[bbn_corner1_x] [bbn_corner1_y] [bbn_corner1_z] \n"
                     "#[bbn_corner2_x] [bbn_corner2_y] [bbn_corner2_z] \n"
                     "#[number_of_points] \n"
                     "#[point1_x] [point1_y] [point1_z] [point1_color_R] [point1_color_G] [point1_color_B]\n"
                     "# ..."
                     "#[pointn_x] [pointn_y] [pointn_z] [pointn_color_R] [pointn_color_G] [pointn_color_B]\n";

        sceneFile << N_corners << endl;

        for ( size_t i = 0; i < N_corners; i++ )
        {
            sceneFile << v_labels[i] << endl;
            sceneFile << v_poses[i].x() << " " << v_poses[i].y() << " " << v_poses[i].z() << " "
                      << v_poses[i].yaw() << " " << v_poses[i].pitch() << " " << v_poses[i].roll() << endl;
            sceneFile << v_corners[i][0].x << " " << v_corners[i][0].y << " " << v_corners[i][0].z << endl;
            sceneFile << v_corners[i][1].x << " " << v_corners[i][1].y << " " << v_corners[i][1].z << endl;
        }

    }
    else
    {
        sceneFile << "#This file contains a 3D point cloud of a reconstructed scene. \n"
                     "#Its format is: \n"
                     "#[number_of_points] \n"
                     "#[point1_x] [point1_y] [point1_z] [point1_color_R] [point1_color_G] [point1_color_B]\n"
                     "# ... \n"
                     "#[pointn_x] [pointn_y] [pointn_z] [pointn_color_R] [pointn_color_G] [pointn_color_B]\n";
    }

    CPointCloudColouredPtr gl_points = m_scene.getByClass<CPointCloudColoured>(0);
    size_t N_points = gl_points->size();
    sceneFile << N_points << endl;

    for ( size_t i = 0; i < N_points; i++ )
    {
        CPointCloudColoured::TPointColour point = gl_points->getPoint(i);

        sceneFile << point.x << " " << point.y << " " << point.z << " "
                  << point.R << " " << point.G << " " << point.B << endl;
    }

    cout << "  [INFO] Done! " << endl << endl;

    return 1;
}

