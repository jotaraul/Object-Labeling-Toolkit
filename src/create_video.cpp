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

/** This application permits to create a video file showing the RBG or depth
  * images of RGB-D observations into a rawlog. If the batch mode is on, then
  * the video will contain all the images in all the rawlogs of the dataset.
  */

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system.h>

#include <mrpt/vision/CVideoFileWriter.h>

#include <iostream>
#include <fstream>

#include <numeric> // std::accumulate

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::math;

using namespace std;


vector<bool> v_sensorsToUse;
bool         useDepthImg = false;


//-----------------------------------------------------------
//                     getImages
//-----------------------------------------------------------

void getImages( CVideoFileWriter &vid, string &i_rawlogFileName,
                size_t imgWidth, size_t width, size_t height, size_t N_channels,
                string textToAdd, size_t N_sensors )
{
    if ( !mrpt::system::fileExists(i_rawlogFileName) )
        return;

    cout << "    Working with " << i_rawlogFileName;
    cout.flush();

    CImage lastImg;
    vector<CObservation3DRangeScanPtr> v_obs(4);
    vector<bool>   v_imagesLoaded(4,false);

    // Insert a few frames to show the
    CImage preImage(width,height,N_channels);
    preImage.textOut(10,20,textToAdd,TColor(255,0,255));

    for ( size_t i=0; i < 10; i++ )
        vid << preImage;

    CFileGZInputStream i_rawlog(i_rawlogFileName);

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,observations,obs,obsIndex) )
    {
        if ( IS_CLASS(obs, CObservation3DRangeScan) )
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            size_t inserted;

            if ( obs3D->sensorLabel == "RGBD_1" )
            {
                v_obs[0] = obs3D;
                v_imagesLoaded[0] = true;
                inserted = 0;
            }
            else if ( obs3D->sensorLabel == "RGBD_2" )
            {
                v_obs[1] = obs3D;
                v_imagesLoaded[1] = true;
                inserted = 1;
            }
            else if ( obs3D->sensorLabel == "RGBD_3" )
            {
                v_obs[2] = obs3D;
                v_imagesLoaded[2] = true;
                inserted = 2;
            }
            else if ( obs3D->sensorLabel == "RGBD_4" )
            {
                v_obs[3] = obs3D;
                v_imagesLoaded[3] = true;
                inserted = 3;
            }

            double sum = v_imagesLoaded[0] + v_imagesLoaded[1]
                    + v_imagesLoaded[2] + v_imagesLoaded[3];

            for ( size_t i = 0; i < 4; i++ )
            {
                if (( i == inserted ) || (v_obs[i].null()))
                    continue;

                if ( timeDifference( v_obs[i]->timestamp, v_obs[inserted]->timestamp )
                     > 0.5 )

                    v_imagesLoaded[i] = 0;


            }

            if ( sum == 4 )
            {

                CImage img(height,width,N_channels);

                size_t imagesDrawn = 0;

                vector<size_t> v_imgsOrder(4);
                /*v_imgsOrder[0] = 2;
                v_imgsOrder[1] = 0;
                v_imgsOrder[2] = 1;
                v_imgsOrder[3] = 3;*/

                v_imgsOrder[0] = 2;
                v_imgsOrder[1] = 3;
                v_imgsOrder[2] = 0;
                v_imgsOrder[3] = 1;

                for ( size_t i = 0; i < 4; i++ )
                {
                    if ( v_sensorsToUse[i] )
                    {
                        if ( useDepthImg )
                        {
                            CImage imgDepth;
                            imgDepth.setFromMatrix(v_obs[v_imgsOrder[i]]->rangeImage);

                            img.drawImage(0,imgWidth*imagesDrawn,imgDepth);
                        }
                        else
                            img.drawImage(0,imgWidth*imagesDrawn,v_obs[v_imgsOrder[i]]->intensityImage);
                        imagesDrawn++;
                    }
                }


                CImage rotatedImg(width,height,N_channels);

                for ( size_t row = 0; row < 320; row++ )
                    for ( size_t col = 0; col < imgWidth*N_sensors; col++ )
                    {
                        u_int8_t r, g,b;

                        if ( useDepthImg )
                        {
                            r = g = b = *(img.get_unsafe(row,col,0));
                        }
                        else
                        {
                            r = *(img.get_unsafe(row,col,2));
                            g = *(img.get_unsafe(row,col,1));
                            b = *(img.get_unsafe(row,col,0));
                        }

                        TColor color(r,g,b);

                        rotatedImg.setPixel(col,320-row,color);

                     }

                rotatedImg.textOut(10,20,textToAdd,TColor(255,0,255));
                vid << rotatedImg;
                lastImg = rotatedImg;

                v_imagesLoaded.clear();
                v_imagesLoaded.resize(4,0);

            }
        }
    }

    for ( size_t i = 0; i < 8; i++ )
        vid << lastImg;

    cout << "   DONE!" << endl;

}

//-----------------------------------------------------------
//                          main
//-----------------------------------------------------------

int main(int argc, char **argv)
{

    cout << endl << "-----------------------------------------------------" << endl;
    cout <<         "                Create video app.                    " << endl;
    cout <<         "            [Object Labeling Tookit]                 " << endl;
    cout <<         "-----------------------------------------------------" << endl << endl;

    try
    {

        string i_rawlogFile;
        string o_videoFile;

        int fps = 10;

        bool batchMode;

        string datasetPath;
        vector<string> v_sessions;
        v_sessions.push_back("");
//        v_sessions.push_back("session1");
//        v_sessions.push_back("session2");
//        v_sessions.push_back("session3");

        vector<string> v_typesOfRooms;
        v_typesOfRooms.push_back("bathroom");
        v_typesOfRooms.push_back("bedroom");
        v_typesOfRooms.push_back("corridor");
        v_typesOfRooms.push_back("fullhouse");
        v_typesOfRooms.push_back("hall");
        v_typesOfRooms.push_back("kitchen");
        v_typesOfRooms.push_back("livingroom");
        v_typesOfRooms.push_back("masterroom");
        v_typesOfRooms.push_back("livingroomkitchen");

        vector<string> v_sequences;

        v_sensorsToUse.resize(4,true);

        //
        // Load parameters
        //

        bool sensorsReset = false;

        if ( argc >= 3 )
        {
            i_rawlogFile = argv[1];
            o_videoFile = argv[2];

            size_t arg = 3;

            while ( arg < argc )
            {
                if ( !strcmp(argv[arg],"-useDepthImg") )
                {
                    useDepthImg = true;
                }
                else if ( !strcmp(argv[arg],"-fps") )
                {
                    fps = atoi(argv[arg+1]);
                    arg++;
                }
                else if ( !strcmp(argv[arg],"-batchMode") )
                {
                    batchMode = true;
                    datasetPath = i_rawlogFile;
                }
                else if ( !strcmp(argv[arg],"-sensor") )
                {
                    if ( !sensorsReset )
                    {
                        v_sensorsToUse.clear();
                        v_sensorsToUse.resize(4,false);
                    }

                    sensorsReset = true;

                    string sensorLabel(argv[arg+1]);

                    if ( sensorLabel == "RGBD_1" )
                        v_sensorsToUse[0] = true;
                    else if ( sensorLabel == "RGBD_2" )
                        v_sensorsToUse[1] = true;
                    else if ( sensorLabel == "RGBD_3" )
                        v_sensorsToUse[2] = true;
                    else if ( sensorLabel == "RGBD_4" )
                        v_sensorsToUse[3] = true;
                    else
                    {
                        cout << "[Error] " << argv[arg+1] << "unknown sensor label" << endl;
                        return -1;
                    }

                    arg++;
                }
                else
                {
                    cout << "[Error] " << argv[arg] << " unknown paramter" << endl;
                    return -1;
                }

                arg++;
            }
        }
        else
        {
            cout << "Usage information. Two expected arguments: " << endl <<
                    " \t (1) Input rawlog file / path of the dataset in the system." << endl <<
                    " \t (2) Output video file." << endl;
            cout << "Then, optional parameters:" << endl <<
                    " \t -fps         : Frames per second of the output video." << endl <<
                    " \t -sensor      : Use information from this sensor." << endl <<
                    " \t -useDepthImg : Use depth images instead of RGB." << endl <<
                    " \t -batchMode   : Create a video with all the sequences into the dataset." << endl <<
                    " \t                If set, then the first app parameter is the dataset path." << endl;
            return 0;
        }

        if ( useDepthImg )
            cout << "  [INFO] Using depth image." << endl;
        if ( batchMode )
        {
            cout << "  [INFO] Using batch mode." << endl;

            for ( size_t i = 0; i < v_typesOfRooms.size(); i++ )
            {
                std::stringstream ss;
                ss << datasetPath << v_typesOfRooms[i];

                bool dirExists = true;
                size_t j = 1;

                while (dirExists)
                {
                    std::stringstream ss2;
                    ss2 << ss.str() << j;

                    string dir = ss2.str();

                    if ( mrpt::system::directoryExists(dir) )
                    {
                        std::stringstream  s3;
                        s3 << v_typesOfRooms[i] << j;
                        v_sequences.push_back(s3.str());
                        cout << "    " << dir << " exists" << endl;
                    }
                    else
                    {
                        dirExists = false;
                        cout << "    " << dir << " doesn't exist" << endl;
                    }

                    j++;
                }
            }
        }

        size_t N_sensors = v_sensorsToUse[0] + v_sensorsToUse[1]
                            + v_sensorsToUse[2] + v_sensorsToUse[3];

        CVideoFileWriter  vid;

        size_t imgWidth = ( useDepthImg ) ? 244 : 240;

        size_t width = imgWidth*N_sensors;
        size_t height = 320;

        size_t N_channels = ( useDepthImg ) ? 1 : 3;

        vid.open( o_videoFile,
                  fps,
                  TPixelCoord(width,height),
                  "MJPG",
                  (useDepthImg) ? false : true
                );

        if ( !vid.isOpen() )
        {
            cout << "  [ERROR] Opening video file." << endl;
            return -1;
        }



        //
        // Batch mode
        //

        if ( !batchMode )
        {
            cout << "Working with " << i_rawlogFile << endl;

            getImages( vid, i_rawlogFile, imgWidth, width, height, N_channels, string(""),N_sensors );

        }
        else
        {
            for ( size_t session_index = 0; session_index < v_sessions.size(); session_index++ )
                for ( size_t sequece_index = 0; sequece_index < v_sequences.size(); sequece_index++ )
                    for ( size_t rawlog_index = 1; rawlog_index < 4; rawlog_index++ )
                    {

                        std::stringstream ss;
                        ss << rawlog_index;
                        string i_rawlogFileName = datasetPath+"/"+v_sessions[session_index]+"/"+v_sequences[sequece_index]+"/"+ss.str()+".rawlog";
                        string textToAdd = v_sessions[session_index]+" / "+v_sequences[sequece_index] + " / " + ss.str();

                        getImages( vid, i_rawlogFileName,
                                   imgWidth, width,
                                   height, N_channels,
                                   textToAdd, N_sensors );
                    }
        }

        vid.close();

        cout << "  [INFO] Video completed!" << endl;

        return 0;

    } catch (exception &e)
    {
        cout << "HOMe exception caught: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        printf("Another exception!!");
        return -1;
    }
}
