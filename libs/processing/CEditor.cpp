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

#include <mrpt/utils.h>
#include <opencv2/core/core.hpp>

using namespace OLT;
using namespace std;

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::utils;


int CSaveAsPlainText::process()
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
                    "# and measurements are in meters. \n";


            file << obs2D->aperture << endl;
            file << obs2D->maxRange << endl;
            file << obs2D->scan.size() << endl;

            for ( size_t i = 0; i < obs2D->scan.size(); i++ )
                file << obs2D->scan[i] << " ";

            file << endl;

            for ( size_t i = 0; i < obs2D->validRange.size(); i++ )
                ( !obs2D->validRange[i] ) ? file << "0 " : file << "1 ";

            file << endl;

            //            std::vector<float>   scan; //!< The range values of the scan, in meters. Must have same length than \a validRange
            //            std::vector<char>    validRange;  //!< It's false (=0) on no reflected rays, referenced to elements in \a scan

            file.close();
        }
        else if (IS_CLASS(obs, CObservation3DRangeScan))
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            ofstream  file;

            if ( m_optionsD["generate_point_clouds"] && obs3D->hasPoints3D )
            {
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
                if ( 1 )
                {
                    cv::Mat img(2,2, CV_8UC3);
                }
                else
                {
                    fileName = format("%s%lu_labels.txt",outputObsDir.c_str(),obsIndex);
                    file.open(fileName.c_str());

                    file << "#This file contains a labelled matrix/mask, associated with RGB-D images with the same id. \n"
                            "#Its format is: \n"
                            "#[number_of_labels/categories] \n"
                            "#[label_id_0] [label_string_0] \n"
                            " ... \n"
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

                    int N_cols, N_rows;
                    obs3D->pixelLabels->getSize(N_rows,N_cols);

                    file << N_rows << endl;
                    file << N_cols << endl;

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
            }

            if ( obs3D->hasRangeImage )
            {
                CImage imgDepth;
                imgDepth.setFromMatrix(obs3D->rangeImage);

                fileName = format("%s%lu_depth.png",outputObsDir.c_str(),obsIndex);
                imgDepth.saveToFile(fileName);

                fileName = format("%s%lu_depth.txt",outputObsDir.c_str(),obsIndex);
                file.open(fileName.c_str());

                file << "#This file contains the depth image associated with the RGB image with the same id. \n"
                        "#Its format is: \n"
                        "#[number_of_rows] \n"
                        "#[number_of_cols] \n"
                        "#[matrix_of_depths] \n"
                        "#Units are meters \n";

                int N_rows = obs3D->rangeImage.rows();
                int N_cols = obs3D->rangeImage.cols();

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
            //obs3D->rangeImage.saveToTextFile(format("./external/%d_depth.txt",obsIndex),mrpt::math::MATRIX_FORMAT_FIXED);//_convertToExternalStorage(format("./external/%d_depth.txt",obsIndex),"./");
            if ( obs3D->hasIntensityImage )
            {
                fileName = format("%s%lu_intensity.png",outputObsDir.c_str(),obsIndex);
                obs3D->intensityImage.saveToFile(fileName,100);
            }

        }
    }

    sequenceFile.close();

    cout << endl << "  [INFO] Done!" << endl << endl;

}

