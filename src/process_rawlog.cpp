/*---------------------------------------------------------------------------*
 |                             HOMe-toolkit                                  |
 |       A toolkit for working with the HOME Environment dataset (HOMe)      |
 |                                                                           |
 |              Copyright (C) 2015 Jose Raul Ruiz Sarmiento                  |
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

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/gui.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/threads.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/utils/CConfigFile.h>



#include <iostream>
#include <fstream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::obs;

//mrpt::gui::CDisplayWindow3D  win3D;

CPose3D hokuyoPose, RGBD_1Pose, RGBD_2Pose, RGBD_3Pose, RGBD_4Pose;
bool useDefaultIntrinsics;

void loadConfig( const string configFileName )
{
    CConfigFile config( configFileName );

    useDefaultIntrinsics = config.read_bool("GENERAL","use_default_intrinsics","",true);

    double x, y, z, yaw, pitch, roll;

    x       = config.read_double("hokuyo","x",0,true);
    y       = config.read_double("hokuyo","y",0,true);
    z       = config.read_double("hokuyo","z",0,true);
    yaw     = DEG2RAD(config.read_double("hokuyo","yaw",0,true));
    pitch	= DEG2RAD(config.read_double("hokuyo","pitch",0,true));
    roll	= DEG2RAD(config.read_double("hokuyo","roll",0,true));

    hokuyoPose.setFromValues(x,y,z,yaw,pitch,roll);

    x       = config.read_double("RGBD_1","x",0,true);
    y       = config.read_double("RGBD_1","y",0,true);
    z       = config.read_double("RGBD_1","z",0,true);
    yaw     = DEG2RAD(config.read_double("RGBD_1","yaw",0,true));
    pitch	= DEG2RAD(config.read_double("RGBD_1","pitch",0,true));
    roll	= DEG2RAD(config.read_double("RGBD_1","roll",0,true));

    RGBD_1Pose.setFromValues(x,y,z,yaw,pitch,roll);

    x       = config.read_double("RGBD_2","x",0,true);
    y       = config.read_double("RGBD_2","y",0,true);
    z       = config.read_double("RGBD_2","z",0,true);
    yaw     = DEG2RAD(config.read_double("RGBD_2","yaw",0,true));
    pitch	= DEG2RAD(config.read_double("RGBD_2","pitch",0,true));
    roll	= DEG2RAD(config.read_double("RGBD_2","roll",0,true));

    RGBD_2Pose.setFromValues(x,y,z,yaw,pitch,roll);

    x       = config.read_double("RGBD_3","x",0,true);
    y       = config.read_double("RGBD_3","y",0,true);
    z       = config.read_double("RGBD_3","z",0,true);
    yaw     = DEG2RAD(config.read_double("RGBD_3","yaw",0,true));
    pitch	= DEG2RAD(config.read_double("RGBD_3","pitch",0,true));
    roll	= DEG2RAD(config.read_double("RGBD_3","roll",0,true));

    RGBD_3Pose.setFromValues(x,y,z,yaw,pitch,roll);

    x       = config.read_double("RGBD_4","x",0,true);
    y       = config.read_double("RGBD_4","y",0,true);
    z       = config.read_double("RGBD_4","z",0,true);
    yaw     = DEG2RAD(config.read_double("RGBD_4","yaw",0,true));
    pitch	= DEG2RAD(config.read_double("RGBD_4","pitch",0,true));
    roll	= DEG2RAD(config.read_double("RGBD_4","roll",0,true));

    RGBD_4Pose.setFromValues(x,y,z,yaw,pitch,roll);
}

int main(int argc, char* argv[])
{
    try
    {
        // Set 3D window

        /*win3D.setWindowTitle("Sequential visualization");

        win3D.resize(400,300);

        win3D.setCameraAzimuthDeg(140);
        win3D.setCameraElevationDeg(20);
        win3D.setCameraZoom(6.0);
        win3D.setCameraPointingToPoint(2.5,0,0);

        mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

        opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-7,7,-7,7,0,1);
        obj->setColor(0.7,0.7,0.7);
        obj->setLocation(0,0,0);
        scene->insert( obj );

        mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
        gl_points->setPointSize(4.5);

        win3D.unlockAccess3DScene();*/

        CRawlog i_rawlog, o_rawlog, o_rawlogHokuyo, o_rawlogRGBD1, o_rawlogRGBD2, o_rawlogRGBD3, o_rawlogRGBD4;
        string hokuyo = "HOKUYO1";
        string RGBD_sensor1 = "RGBD_1";
        string RGBD_sensor2 = "RGBD_2";
        string RGBD_sensor3 = "RGBD_3";
        string RGBD_sensor4 = "RGBD_4";
        bool onlyHokuyo = false;

        string i_rawlogFilename;
        string o_rawlogFileName;

        if ( argc >= 3 )
        {
            i_rawlogFilename = argv[1];
            string configFileName = argv[2];

            loadConfig( configFileName );

            for ( size_t arg = 3; arg < argc; arg++ )
            {
                if ( !strcmp(argv[arg],"-only_hokuyo") )
                {
                    onlyHokuyo = true;
                    cout << "[INFO] Processing only hokuyo observations."  << endl;
                }
            }

        }
        else
        {
            cout << "Usage information. At least two expected arguments: " << endl <<
                    " \t (1) Rawlog file." << endl <<
                    " \t (2) Configuration file." << endl <<
            cout << "Then, optional parameters:" << endl <<
                    " \t -only_hokuyo : Process only hokuyo observations." << endl;

            return 0;
        }

        o_rawlogFileName.assign(i_rawlogFilename.begin(), i_rawlogFilename.end()-7);
        o_rawlogFileName += (onlyHokuyo) ? "_hokuyo" : "";
        o_rawlogFileName += "_processed.rawlog";

        if (!i_rawlog.loadFromRawLogFile(i_rawlogFilename))
            throw std::runtime_error("Couldn't open rawlog dataset file for input...");

        cout << "[INFO] Working with " << i_rawlogFilename << endl;

        TCamera defaultCameraParamsDepth;
        defaultCameraParamsDepth.nrows = 488;
        defaultCameraParamsDepth.scaleToResolution(320,244);

        TCamera defaultCameraParamsInt;
        defaultCameraParamsInt.scaleToResolution(320,240);

        for ( size_t obsIndex = 0; obsIndex < i_rawlog.size(); obsIndex++ )
        {
            cout << "Processing " << obsIndex << " of " << i_rawlog.size() << '\xd';
            mrpt::system::sleep(10);

            CObservationPtr obs = i_rawlog.getAsObservation(obsIndex);

            if ( obs->sensorLabel == hokuyo )
            {
                CObservation2DRangeScanPtr obs2D = CObservation2DRangeScanPtr(obs);
                obs2D->load();

                obs2D->setSensorPose(hokuyoPose);

                o_rawlogHokuyo.addObservationMemoryReference(obs2D);
                o_rawlog.addObservationMemoryReference(obs2D);
            }
            else if ( obs->sensorLabel == RGBD_sensor1 )
            {
                CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
                obs3D->load();

                obs3D->setSensorPose(RGBD_1Pose);

                if ( useDefaultIntrinsics )
                {
                    obs3D->cameraParams = defaultCameraParamsDepth;
                    obs3D->cameraParamsIntensity = defaultCameraParamsInt;
                }
                else
                {
                    obs3D->cameraParams.scaleToResolution(320,244);
                    obs3D->cameraParamsIntensity.scaleToResolution(320,240);
                }

                obs3D->project3DPointsFromDepthImage();

                o_rawlogRGBD1.addObservationMemoryReference(obs3D);
                o_rawlog.addObservationMemoryReference(obs3D);


                /*mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

                mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
                gl_points->setPointSize(4.5);
                mrpt::slam::CColouredPointsMap colouredMap;
                colouredMap.insertObservation( obs3D.pointer() );

                gl_points->loadFromPointsMap( &colouredMap );

                scene->insert( gl_points );

                win3D.unlockAccess3DScene();
                win3D.repaint();
                win3D.waitForKey();*/
            }
            else if ( obs->sensorLabel == RGBD_sensor2 )
            {
                CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
                obs3D->load();

                obs3D->setSensorPose(RGBD_2Pose);

                if ( useDefaultIntrinsics )
                {
                    obs3D->cameraParams = defaultCameraParamsDepth;
                    obs3D->cameraParamsIntensity = defaultCameraParamsInt;
                }
                else
                {
                    obs3D->cameraParams.scaleToResolution(320,244);
                    obs3D->cameraParamsIntensity.scaleToResolution(320,240);
                }

                obs3D->project3DPointsFromDepthImage();

                o_rawlogRGBD2.addObservationMemoryReference(obs3D);
                o_rawlog.addObservationMemoryReference(obs3D);
            }
            else if ( obs->sensorLabel == RGBD_sensor3 )
            {
                CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
                obs3D->load();

                obs3D->setSensorPose(RGBD_3Pose);

                if ( useDefaultIntrinsics )
                {
                    obs3D->cameraParams = defaultCameraParamsDepth;
                    obs3D->cameraParamsIntensity = defaultCameraParamsInt;
                }
                else
                {
                    obs3D->cameraParams.scaleToResolution(320,244);
                    obs3D->cameraParamsIntensity.scaleToResolution(320,240);
                }

                obs3D->project3DPointsFromDepthImage();

                o_rawlogRGBD3.addObservationMemoryReference(obs3D);
                o_rawlog.addObservationMemoryReference(obs3D);
            }
            else if ( obs->sensorLabel == RGBD_sensor4 )
            {
                CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
                obs3D->load();

                obs3D->setSensorPose(RGBD_4Pose);

                if ( useDefaultIntrinsics )
                {
                    obs3D->cameraParams = defaultCameraParamsDepth;
                    obs3D->cameraParamsIntensity = defaultCameraParamsInt;
                }
                else
                {
                    obs3D->cameraParams.scaleToResolution(320,244);
                    obs3D->cameraParamsIntensity.scaleToResolution(320,240);
                }

                obs3D->project3DPointsFromDepthImage();

                o_rawlogRGBD4.addObservationMemoryReference(obs3D);
                o_rawlog.addObservationMemoryReference(obs3D);

                /*mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

                mrpt::slam::CColouredPointsMap colouredMap;
                colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
                colouredMap.loadFromRangeScan( *obs3D );

                gl_points->loadFromPointsMap( &colouredMap );

                scene->insert( gl_points );

                win3D.unlockAccess3DScene();
                win3D.repaint();
                win3D.waitForKey();*/
            }
            else
                continue;
        }

        if ( !onlyHokuyo )
            o_rawlog.saveToRawLogFile( o_rawlogFileName );
        else
            o_rawlogHokuyo.saveToRawLogFile( o_rawlogFileName );

        cout << "[INFO] Rawlog saved as " << o_rawlogFileName << endl;

//        o_rawlogHokuyo.saveToRawLogFile( o_rawlogFilePath + "_" + hokuyo + ".rawlog" );
//        o_rawlogRGBD1.saveToRawLogFile( o_rawlogFilePath + "_" + RGBD_sensor1 + ".rawlog" );
//        o_rawlogRGBD2.saveToRawLogFile( o_rawlogFilePath + "_" + RGBD_sensor2 + ".rawlog" );
//        o_rawlogRGBD3.saveToRawLogFile( o_rawlogFilePath + "_" + RGBD_sensor3 + ".rawlog" );
//        o_rawlogRGBD4.saveToRawLogFile( o_rawlogFilePath + "_" + RGBD_sensor4 + ".rawlog" );

        return 0;

    } catch (exception &e)
    {
        cout << "MRPT exception caught: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        printf("Another exception!!");
        return -1;
    }
}

