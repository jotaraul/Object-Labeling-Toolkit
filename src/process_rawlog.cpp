/*---------------------------------------------------------------------------*
 |                         Object Labeling Toolkit                           |
 |            A set of software components for the management and            |
 |                      labeling of RGB-D datasets                           |
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

#include <mrpt/gui.h>
#include <mrpt/math/utils.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CConfigFile.h>

#include <iostream>
#include <fstream>

#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
    #include <clams/discrete_depth_distortion_model.h>
#endif

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::obs;

using namespace std;

// TODOS:
// Enable the use of more than a laser scanner

// Visualization purposes
//mrpt::gui::CDisplayWindow3D  win3D;

vector<CPose3D> v_laser_sensorPoses; // Poses of the 2D laser scaners in the robot

struct TRGBD_Sensor{
    CPose3D pose;
    string  sensorLabel;
    bool    loadIntrinsicParameters;
};

vector<TRGBD_Sensor> v_RGBD_sensors;  // Poses and labels of the RGBD devices in the robot

bool useDefaultIntrinsics;

#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
    // Intrinsic model to undistort the depth image
    clams::DiscreteDepthDistortionModel intrinsic_model;
#endif


//-----------------------------------------------------------
//
//                      loadConfig
//
//-----------------------------------------------------------

void loadConfig( const string configFileName )
{
    CConfigFile config( configFileName );

    cout << "[INFO] loading component options from " << configFileName << endl;

    useDefaultIntrinsics = config.read_bool("GENERAL","use_default_intrinsics","",true);

    //
    // Load 2D laser scanners info
    //

    double x, y, z, yaw, pitch, roll;

    CPose3D laser_pose;
    string sensorLabel;
    int sensorIndex = 1;
    bool keepLoading = true;

    while ( keepLoading )
    {
        sensorLabel = mrpt::format("HOKUYO%i",sensorIndex);

        if ( config.sectionExists(sensorLabel) )
        {
            x       = config.read_double(sensorLabel,"x",0,true);
            y       = config.read_double(sensorLabel,"y",0,true);
            z       = config.read_double(sensorLabel,"z",0,true);
            yaw     = DEG2RAD(config.read_double(sensorLabel,"yaw",0,true));
            pitch	= DEG2RAD(config.read_double(sensorLabel,"pitch",0,true));
            roll	= DEG2RAD(config.read_double(sensorLabel,"roll",0,true));

            laser_pose.setFromValues(x,y,z,yaw,pitch,roll);
            v_laser_sensorPoses.push_back( laser_pose );
            cout << "[INFO] loaded extrinsic calibration for " << sensorLabel << endl;

            sensorIndex++;
        }
        else
            keepLoading = false;
    }

    //
    // Load RGBD devices info
    //

    sensorIndex = 1;
    keepLoading = true;

    while ( keepLoading )
    {
        sensorLabel = mrpt::format("RGBD_%i",sensorIndex);

        if ( config.sectionExists(sensorLabel) )
        {
            x       = config.read_double(sensorLabel,"x",0,true);
            y       = config.read_double(sensorLabel,"y",0,true);
            z       = config.read_double(sensorLabel,"z",0,true);
            yaw     = DEG2RAD(config.read_double(sensorLabel,"yaw",0,true));
            pitch	= DEG2RAD(config.read_double(sensorLabel,"pitch",0,true));
            roll	= DEG2RAD(config.read_double(sensorLabel,"roll",0,true));

            bool loadIntrinsic = config.read_bool(sensorLabel,"loadIntrinsic",0,true);

            TRGBD_Sensor RGBD_sensor;
            RGBD_sensor.pose.setFromValues(x,y,z,yaw,pitch,roll);
            RGBD_sensor.sensorLabel = sensorLabel;
            RGBD_sensor.loadIntrinsicParameters = loadIntrinsic;
            v_RGBD_sensors.push_back( RGBD_sensor );

            cout << "[INFO] loaded extrinsic calibration for " << sensorLabel << endl;

            sensorIndex++;
        }
        else
            keepLoading = false;
    }

    #ifdef USING_CLAMS_INTRINSIC_CALIBRATION


    #endif


}

//-----------------------------------------------------------
//
//                      getSensorPos
//
//-----------------------------------------------------------


size_t getSensorPos( const string label )
{
    for ( size_t i = 0; i < v_RGBD_sensors.size(); i++ )
    {
        if ( v_RGBD_sensors[i].sensorLabel == label )
            return i;
    }

    return -1;
}

//-----------------------------------------------------------
//
//                    showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "Usage information. At least two expected arguments: " << endl <<
            " \t (1) Rawlog file." << endl <<
            " \t (2) Configuration file." << endl <<
            cout << "Then, optional parameters:" << endl <<
            " \t -h             : Shows this help." << endl <<
            " \t -only_hokuyo : Process only hokuyo observations." << endl;
}


//-----------------------------------------------------------
//main
//                          main
//
//-----------------------------------------------------------

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

        //
        // Useful variables
        //

        CRawlog i_rawlog, o_rawlog, o_rawlogHokuyo, o_rawlogRGBD;
        string hokuyo = "HOKUYO1";
        bool onlyHokuyo = false;
        bool onlyRGBD = false;

        string configFileName;

        string i_rawlogFilename;
        string o_rawlogFileName;

        TCamera defaultCameraParamsDepth;
        defaultCameraParamsDepth.nrows = 488;
        defaultCameraParamsDepth.scaleToResolution(320,244);

        TCamera defaultCameraParamsInt;
        defaultCameraParamsInt.scaleToResolution(320,240);

        //
        // Load paramteres
        //

        if ( argc >= 3 )
        {
            i_rawlogFilename = argv[1];
            configFileName = argv[2];

            for ( size_t arg = 3; arg < argc; arg++ )
            {
                if ( !strcmp(argv[arg],"-only_hokuyo") )
                {
                    onlyHokuyo = true;
                    cout << "[INFO] Processing only hokuyo observations."  << endl;
                }
                if ( !strcmp(argv[arg],"-only_rgbd") )
                {
                    onlyRGBD = true;
                    cout << "[INFO] Processing only rgbd observations."  << endl;
                }
                else if ( !strcmp(argv[arg],"-h") )
                {
                    showUsageInformation();
                    return 0;
                }
                else
                {
                    cout << "[ERROR] Unknown option " << argv[arg] << endl;
                    showUsageInformation();
                    return -1;
                }
            }

        }
        else
        {
            showUsageInformation();

            return 0;
        }

        //
        // Load config information
        //

        loadConfig( configFileName );

        //
        // Open rawlog file
        //

        if (!i_rawlog.loadFromRawLogFile(i_rawlogFilename))
            throw std::runtime_error("Couldn't open rawlog dataset file for input...");

        cout << "[INFO] Working with " << i_rawlogFilename << endl;

        //
        // Process rawlog
        //

        for ( size_t obsIndex = 0; obsIndex < i_rawlog.size(); obsIndex++ )
        {
            // Show progress

            cout << "Processing " << obsIndex << " of " << i_rawlog.size() << '\xd';
            cout.flush();

            CObservationPtr obs = i_rawlog.getAsObservation(obsIndex);

            // Observation from a laser range scan device

            if ( obs->sensorLabel == hokuyo )
            {
                CObservation2DRangeScanPtr obs2D = CObservation2DRangeScanPtr(obs);
                obs2D->load();

                obs2D->setSensorPose(v_laser_sensorPoses[0]);

                if ( onlyHokuyo)
                    o_rawlogHokuyo.addObservationMemoryReference(obs2D);
                else
                    o_rawlog.addObservationMemoryReference(obs2D);
            }
            else
            {
                // RGBD observation?

                size_t RGBD_sensorIndex = getSensorPos(obs->sensorLabel);

                if ( RGBD_sensorIndex >= 0 )
                {
                    TRGBD_Sensor &sensor = v_RGBD_sensors[RGBD_sensorIndex];
                    CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
                    obs3D->load();

                    obs3D->setSensorPose( sensor.pose );

                    if ( useDefaultIntrinsics )
                    {
                        obs3D->cameraParams = defaultCameraParamsDepth;
                        obs3D->cameraParamsIntensity = defaultCameraParamsInt;
                    }
                    else
                    {
                        if ( sensor.loadIntrinsicParameters )
                        {
                            CConfigFile config( configFileName );

                            obs3D->cameraParams.loadFromConfigFile(sensor.sensorLabel + "_depth",config);
                            obs3D->cameraParams.loadFromConfigFile(sensor.sensorLabel + "_intensity",config);
                        }
                        else
                        {
                            obs3D->cameraParams.scaleToResolution(320,244);
                            obs3D->cameraParamsIntensity.scaleToResolution(320,240);
                        }

                    }

                    // Apply depth intrinsic calibration?
                    #ifdef USING_CLAMS_INTRINSIC_CALIBRATION
                        // Intrinsic model to undistort the depth image
                        clams::DiscreteDepthDistortionModel intrinsic_model;

                    #endif



                    obs3D->project3DPointsFromDepthImage();                    

                    if ( onlyRGBD)
                        o_rawlogRGBD.addObservationMemoryReference(obs3D);
                    else
                        o_rawlog.addObservationMemoryReference(obs3D);

                    // Visualization purposes

                    /*
                    mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

                    mrpt::slam::CColouredPointsMap colouredMap;
                    colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
                    colouredMap.loadFromRangeScan( *obs3D );

                    gl_points->loadFromPointsMap( &colouredMap );

                    scene->insert( gl_points );

                    win3D.unlockAccess3DScene();
                    win3D.repaint();
                    win3D.waitForKey();*/
                }
            }

        }

        //
        // Save processed rawlog to file
        //

        o_rawlogFileName.assign(i_rawlogFilename.begin(), i_rawlogFilename.end()-7);
        o_rawlogFileName += (onlyHokuyo) ? "_hokuyo" : "";
        o_rawlogFileName += (onlyRGBD) ? "_rgbd" : "";
        o_rawlogFileName += "_processed.rawlog";

        if ( onlyHokuyo )
            o_rawlogHokuyo.saveToRawLogFile( o_rawlogFileName );
        else if ( onlyRGBD)
            o_rawlogRGBD.saveToRawLogFile( o_rawlogFileName );
        else
            o_rawlog.saveToRawLogFile( o_rawlogFileName );

        cout << "[INFO] Rawlog saved as " << o_rawlogFileName << endl;

        return 0;

    } catch (exception &e)
    {
        cout << "Exception caught: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        printf("Another exception!!");
        return -1;
    }
}

