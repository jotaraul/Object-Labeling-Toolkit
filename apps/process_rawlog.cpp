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
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/interp_fit.h>

#include <numeric>
#include <iostream>
#include <fstream>

#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
    #include <clams/discrete_depth_distortion_model.h>
#endif
#include "processing.hpp"
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

//
// Useful variables
//

string i_rawlogFilename;
string configFileName;
bool setCalibrationParameters = true;
int decimate = 0;
bool keepOnlyProcessed = false;
float removeEmptyObs = 0.0;
float removeWeirdObs = 0.0;
bool onlyRemove3DPointClouds;
bool saveAsPlainText;

string replaceSensorLabel;  // Current sensor label
string replaceSensorLabelAs;// New sensor label

struct TCalibrationConfig{

    bool only2DLaser;
    bool onlyRGBD;

    bool useDefaultIntrinsics;
    bool applyCLAMS;
    bool scaleDepthInfo;
    bool  equalizeRGBHistograms;
    double truncateDepthInfo;
    bool project3DPointClouds;
    bool remove3DPointClouds;    

    TCalibrationConfig() : only2DLaser(false),
        onlyRGBD(false), useDefaultIntrinsics(true), equalizeRGBHistograms(false),
        truncateDepthInfo(0), project3DPointClouds(false), remove3DPointClouds(false)
    {}

} calibConfig;

struct TScaleCalibration{

    float resolution;
    float lowerRange;
    float higerRange;
    string referenceSensor;
    string scaledSensor;
    string i_scaleCalibrationFile;
    CVectorFloat scaleMultipliers;

    TScaleCalibration()
    {}
};

vector<TScaleCalibration> v_scaleCalibrations;

struct TRGBD_Sensor{
    CPose3D pose;
    string  sensorLabel;
    bool    loadIntrinsicParameters;
    int     N_obsProcessed;
#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
    // Intrinsic model to undistort the depth image of an RGBD sensor
    clams::DiscreteDepthDistortionModel depth_intrinsic_model;
#endif
    TRGBD_Sensor() : N_obsProcessed(0)
    {}
};

vector<TRGBD_Sensor> v_RGBD_sensors;  // Poses and labels of the RGBD devices in the robot

vector<CPose3D> v_laser_sensorPoses; // Poses of the 2D laser scaners in the robot


//-----------------------------------------------------------
//
//                    showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "  Usage information. At least one expected argument: " << endl <<
            "    (1) Rawlog file." << endl;
    cout << "  Then, optional parameters:" << endl <<
            "    -config        : Configuration file." << endl <<
            "    -h             : Shows this help." << endl <<
            "    -only_hokuyo   : Process only hokuyo observations." << endl <<
            "    -only_rgbd     : Process only RGB-D observations." << endl  <<
            "    -replaceLabel <l1> <l2>: Replace observations with label <l1> by label <l2>." << endl <<
            "    -project3DPointClouds: Project 3D point clouds from depth images." << endl <<
            "    -removeEmptyObs <num> : Remove empty (depth) observations with a factor of null measurments higher than <num>" << endl <<
            "    -removeWeirdObs <num> : Set to 0 (null) observations with a factor of valid measurements lower than <num>" << endl <<
            "    -remove3DPointClouds: Remove all the point clouds within RGBD observations."
            "    -keepOnlyProcessed: Keep only the observations that have been processed." << endl <<
            "    -decimate <num>: Decimate rawlog keeping only one of each <num> observations." << endl <<
            "    -saveAsPlainText: Save the rawlog as different plain text files. " << endl << endl;
}


//-----------------------------------------------------------
//
//                loadScaleCalibrationFromFile
//
//-----------------------------------------------------------

void loadScaleCalibrationFromFile(const string fileName )
{
    TScaleCalibration sc;

    sc.i_scaleCalibrationFile = fileName;

    ifstream f( sc.i_scaleCalibrationFile.c_str() );

    if ( !f.is_open() )
    {
        cerr << endl << "    [ERROR] Opening file " << sc.i_scaleCalibrationFile << endl;
        return;
    }

    string bin;
    f >> bin >> sc.referenceSensor;
    f >> bin >> sc.scaledSensor;
    f >> bin >> sc.resolution;
    f >> bin >> sc.lowerRange;
    f >> bin >> sc.higerRange;

    int N_scales = 1+1/sc.resolution*(sc.higerRange-sc.lowerRange);

    sc.scaleMultipliers.resize(N_scales);

    for ( int i = 0; i < N_scales; i++ )
    {
        f >> sc.scaleMultipliers(i);
        //cout << "Scale:" << sc.scaleMultipliers(i) << endl;
    }

    v_scaleCalibrations.push_back(sc);
}


//-----------------------------------------------------------
//
//                      loadConfig
//
//-----------------------------------------------------------

void loadConfig()
{
    CConfigFile config( configFileName );

    cout << "  [INFO] Loading component options from " << configFileName << endl;

    //
    // Load execution mode

    setCalibrationParameters = config.read_bool("GENERAL","set_calibration_parameters",true,true);

    //
    // Load calibration parameters

    if ( setCalibrationParameters )
    {
        calibConfig.useDefaultIntrinsics  = config.read_bool("CALIBRATION","use_default_intrinsics",true,true);
        calibConfig.equalizeRGBHistograms = config.read_bool("CALIBRATION","equalize_RGB_histograms",false,true);
        calibConfig.truncateDepthInfo     = config.read_double("CALIBRATION","truncate_depth_info",0,true);
        calibConfig.applyCLAMS            = config.read_bool("CALIBRATION","apply_CLAMS_Calibration_if_available",false,true);
        calibConfig.scaleDepthInfo        = config.read_bool("CALIBRATION","scale_depth_info",false,true);
        calibConfig.project3DPointClouds  = config.read_bool("CALIBRATION","project_3D_point_clouds",false,true);
        calibConfig.remove3DPointClouds  = config.read_bool("CALIBRATION","remove_3D_point_clouds",false,true);

        //
        // Load 2D laser scanners info
        //

        double x, y, z, yaw, pitch, roll;

        CPose3D laser_pose;
        string sensorLabel;
        int sensorIndex = 1;
        bool keepLoading = true;

        cout << "  [INFO] Loaded extrinsic calibration for ";

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

                cout << sensorLabel << " ";

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

                string depthScaleModelPath = config.read_string(sensorLabel,"depth_scale_model_path","",false);

                if ( !depthScaleModelPath.empty() && calibConfig.scaleDepthInfo )
                    loadScaleCalibrationFromFile(depthScaleModelPath);

                bool loadIntrinsic = config.read_bool(sensorLabel,"loadIntrinsic",0,true);

                TRGBD_Sensor RGBD_sensor;
                RGBD_sensor.pose.setFromValues(x,y,z,yaw,pitch,roll);
                RGBD_sensor.sensorLabel = sensorLabel;
                RGBD_sensor.loadIntrinsicParameters = loadIntrinsic;

#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
                if ( calibConfig.applyCLAMS )
                {
                    // Load CLAMS intrinsic model for depth camera
                    string DepthIntrinsicModelpath = config.read_string(sensorLabel,"DepthIntrinsicModelpath","",true);
                    RGBD_sensor.depth_intrinsic_model.load(DepthIntrinsicModelpath);
                }
#endif

                v_RGBD_sensors.push_back( RGBD_sensor );

                cout << sensorLabel << " ";

                sensorIndex++;
            }
            else
                keepLoading = false;
        }

        cout << endl;
    }

}


//-----------------------------------------------------------
//
//                getSensorPosInScalecalib
//
//-----------------------------------------------------------

int getSensorPosInScalecalib( const string label)
{
    for ( size_t i = 0; i < v_scaleCalibrations.size(); i++ )
    {
        if ( v_scaleCalibrations[i].scaledSensor == label )
            return i;
    }

    return -1;
}

//-----------------------------------------------------------
//
//                      getSensorPos
//
//-----------------------------------------------------------

int getSensorPos( const string label )
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
//                        decimateRawlog
//
//-----------------------------------------------------------

void decimateRawlog()
{
    //
    // Check input rawlog
    //

    if (!mrpt::system::fileExists(i_rawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << i_rawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }
    else if (configFileName.empty())
    {
        cerr << "  [ERROR] Information needs to be loaded from a configuration file." << endl;
        return;
    }

    CFileGZInputStream i_rawlog(i_rawlogFilename);

    cout << "  [INFO] Decimating rawlog " << i_rawlogFilename << endl;
    cout << "  [INFO] Taking one of each " << decimate << " obs" << endl;


    string o_rawlogFileName;

    //
    // Set output rawlog file
    //

    o_rawlogFileName.assign(i_rawlogFilename.begin(),
                            i_rawlogFilename.end()-7);
    o_rawlogFileName += (calibConfig.only2DLaser) ? "_hokuyo" : "";
    o_rawlogFileName += (calibConfig.onlyRGBD) ? "_rgbd" : "";
    o_rawlogFileName += "_decimated.rawlog";

    CFileGZOutputStream o_rawlog(o_rawlogFileName);

    //
    // Process rawlog

    size_t N_sensors = v_RGBD_sensors.size();
    vector<CObservation3DRangeScanPtr> v_obs(N_sensors); // Current set of obs
    vector<bool> v_obs_loaded(N_sensors,false); // Track the sensors with an obs loaded
    size_t setOfObsIndex = 0;

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;

    cout << "    Process: ";
    cout.flush();

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,observations,obs,obsIndex) )
    {
        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        if ( obs->sensorLabel == "HOKUYO1" )
        {
//            CObservation2DRangeScanPtr obs2D = CObservation2DRangeScanPtr(obs);
//            obs2D->load();

//            obs2D->setSensorPose(v_laser_sensorPoses[0]);

//            o_rawlog << obs2D;
        }
        else if ( !calibConfig.only2DLaser )
        {
            // RGBD observation?

            size_t RGBD_sensorIndex = getSensorPos(obs->sensorLabel);

            if ( RGBD_sensorIndex >= 0 )
            {
                TRGBD_Sensor &sensor = v_RGBD_sensors[RGBD_sensorIndex];
                CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
                obs3D->load();

                sensor.N_obsProcessed++;

                v_obs[RGBD_sensorIndex]        = obs3D;
                v_obs_loaded[RGBD_sensorIndex] = true;

                double sum = 0;

                for ( size_t i_sensor = 0; i_sensor < N_sensors; i_sensor++ )
                    sum += v_obs_loaded[i_sensor];

                if ( sum == N_sensors )
                {
                    v_obs_loaded.clear();
                    v_obs_loaded.resize(N_sensors,false);

                    if (setOfObsIndex%decimate)
                    {
                        setOfObsIndex++;
                        continue;
                    }

                    setOfObsIndex++;

                    v_obs_loaded.clear();
                    v_obs_loaded.resize(N_sensors,false);

                    for ( size_t i=0; i < N_sensors; i++ )
                        o_rawlog << v_obs[i];

                }
            }
        }
    }

    cout << endl << "    Number of RGBD observations processed: " << endl;

    for ( size_t i = 0; i < v_RGBD_sensors.size(); i++ )
        cout << "      " << v_RGBD_sensors[i].sensorLabel
             << ": " << v_RGBD_sensors[i].N_obsProcessed << endl;

    cout << "  [INFO] Rawlog saved as " << o_rawlogFileName << endl << endl;
}

//-----------------------------------------------------------
//
//                      processRawlog
//
//-----------------------------------------------------------

void processRawlog()
{
    //
    // Check input rawlog
    //

    if (!mrpt::system::fileExists(i_rawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << i_rawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    CFileGZInputStream i_rawlog(i_rawlogFilename);

    cout << "  [INFO] Working with " << i_rawlogFilename << endl;

    cout << "  [INFO] Equalizing RGB histograms   : ";

    if (!calibConfig.equalizeRGBHistograms)
        cout << "off" << endl;
    else
        cout << "on" << endl;

    cout << "  [INFO] Truncating depth information: ";

    if ( !calibConfig.truncateDepthInfo )
        cout << "off" << endl;
    else
        cout << "on (from " << calibConfig.truncateDepthInfo << "m)" << endl;

    cout << "  [INFO] Scaling depth info          : ";    

    if ( !calibConfig.scaleDepthInfo )
        cout << "off" << endl;
    else
        cout << "on" << endl;

    cout << "  [INFO] Apply CLAMS calibration     : ";

    if ( calibConfig.applyCLAMS )
#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
        cout << "on" << endl;
#else
       cout << "off (CLAMS is not available in the system)" << endl;
#endif
    else
        cout << "off" << endl;

    cout << "  [INFO] Projecting 3D point clouds  : ";

    if ( !calibConfig.project3DPointClouds )
        cout << "off" << endl;
    else
        cout << "on" << endl;

    cout << "  [INFO] Removing 3D point clouds    : ";

    if ( !calibConfig.remove3DPointClouds )
        cout << "off" << endl;
    else
        cout << "on" << endl;

    string o_rawlogFileName;

    //
    // Set output rawlog file
    //

    o_rawlogFileName.assign(i_rawlogFilename.begin(),
                            i_rawlogFilename.end()-7);
    o_rawlogFileName += (calibConfig.only2DLaser) ? "_hokuyo" : "";
    o_rawlogFileName += (calibConfig.onlyRGBD) ? "_rgbd" : "";
    o_rawlogFileName += "_processed.rawlog";

    CFileGZOutputStream o_rawlog(o_rawlogFileName);

    // Default params
    TCamera defaultCameraParamsDepth;
    defaultCameraParamsDepth.nrows = 488;
    defaultCameraParamsDepth.scaleToResolution(320,244);

    TCamera defaultCameraParamsInt;
    defaultCameraParamsInt.scaleToResolution(320,240);

    //
    // Process rawlog
    //

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;

    cout << "    Process: ";
    cout.flush();

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,observations,obs,obsIndex) )
    {
        // Check that it is an observation
        if ( !obs )
            continue;

        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        // Observation from a laser range scan device

        if ( obs->sensorLabel == "HOKUYO1" )
        {
            if ( calibConfig.onlyRGBD )
                continue;

            CObservation2DRangeScanPtr obs2D = CObservation2DRangeScanPtr(obs);
            obs2D->load();

            obs2D->setSensorPose(v_laser_sensorPoses[0]);

            o_rawlog << obs2D;
        }
        else if ( !calibConfig.only2DLaser )
        {
            // RGBD observation?

            size_t RGBD_sensorIndex = getSensorPos(obs->sensorLabel);

            if ( RGBD_sensorIndex >= 0 )
            {
                TRGBD_Sensor &sensor = v_RGBD_sensors[RGBD_sensorIndex];
                CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
                obs3D->load();

                sensor.N_obsProcessed++;

                obs3D->setSensorPose( sensor.pose );

                if ( calibConfig.useDefaultIntrinsics )
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

                // Set the relative pose as pure rotation. For more info. see:
                // http://reference.mrpt.org/stable/classmrpt_1_1obs_1_1_c_observation3_d_range_scan.html
                obs3D->relativePoseIntensityWRTDepth = CPose3D(0,0,0,DEG2RAD(-90),0,DEG2RAD(-90));

                // Apply depth intrinsic calibration?
#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
                if ( calibConfig.applyCLAMS )
                {
                    // Undistort Depth image
                    Eigen::MatrixXf depthMatrix = obs3D->rangeImage;
                    v_RGBD_sensors[RGBD_sensorIndex].depth_intrinsic_model.undistort(&depthMatrix);

                    obs3D->rangeImage = depthMatrix;
                }
#endif

                // Scale depth info?
                if ( calibConfig.scaleDepthInfo )
                {
                    int pos = getSensorPosInScalecalib(obs3D->sensorLabel);

                    if ( pos >= 0 )
                    {
                        TScaleCalibration &sc = v_scaleCalibrations[pos];
                        int offset = std::floor(sc.lowerRange*(1/sc.resolution));

                        size_t N_cols = obs3D->rangeImage.cols();
                        size_t N_rows = obs3D->rangeImage.rows();
                        for ( size_t row = 0; row < N_rows; row++ )
                            for ( size_t col = 0; col < N_cols; col++ )
                            {
                                float value = obs3D->rangeImage(row,col);

                                if ( value < sc.lowerRange )
                                    obs3D->rangeImage(row,col) *= sc.scaleMultipliers(0);
                                else if ( value > sc.higerRange )
                                    obs3D->rangeImage(row,col) *= sc.scaleMultipliers(sc.scaleMultipliers.rows()-1);
                                else
                                {
                                    int pos = std::floor(value*(1/sc.resolution));
                                    pos -= offset;

                                    obs3D->rangeImage(row,col) *= sc.scaleMultipliers(pos);
                                }
                            }
                    }
                }

                // Truncate Range image and 3D points?
                if ( calibConfig.truncateDepthInfo )
                {
                    size_t N_cols = obs3D->rangeImage.cols();
                    size_t N_rows = obs3D->rangeImage.rows();
                    for ( size_t row = 0; row < N_rows; row++ )
                        for ( size_t col = 0; col < N_cols; col++ )
                            if( obs3D->rangeImage(row,col) > calibConfig.truncateDepthInfo )
                                obs3D->rangeImage(row,col) = 0;
                }

                // Project 3D points from the depth image or remove them if present
                if ( calibConfig.project3DPointClouds && !calibConfig.remove3DPointClouds )
                    obs3D->project3DPointsFromDepthImage();
                else if ( calibConfig.remove3DPointClouds )
                {
                    obs3D->points3D_x.clear();
                    obs3D->points3D_y.clear();
                    obs3D->points3D_z.clear();

                    obs3D->hasPoints3D = false;
                }

                // Equalize histogram of RGB images?
                if ( calibConfig.equalizeRGBHistograms )
                    obs3D->intensityImage.equalizeHistInPlace();

                o_rawlog << obs3D;

                /*if ( onlyRGBD )
                    o_rawlogRGBD.addObservationMemoryReference(obs3D);
                else
                    o_rawlog.addObservationMemoryReference(obs3D);*/

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

    cout << endl << "    Number of RGBD observations processed: " << endl;

    for ( size_t i = 0; i < v_RGBD_sensors.size(); i++ )
        cout << "      " << v_RGBD_sensors[i].sensorLabel
             << ": " << v_RGBD_sensors[i].N_obsProcessed << endl;

    cout << "  [INFO] Rawlog saved as " << o_rawlogFileName << endl << endl;

}

void replaceLabel()
{
    //
    // Check input rawlog
    //

    if (!mrpt::system::fileExists(i_rawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << i_rawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    CFileGZInputStream i_rawlog(i_rawlogFilename);

    cout << "  [INFO] Processing rawlog " << i_rawlogFilename << endl;
    cout << "  [INFO] Replacing sensor label " << replaceSensorLabel
             << " by " << replaceSensorLabelAs << endl;


    string o_rawlogFileName;

    //
    // Set output rawlog file
    //

    o_rawlogFileName.assign(i_rawlogFilename.begin(),
                            i_rawlogFilename.end()-7);
    o_rawlogFileName += (calibConfig.only2DLaser) ? "_hokuyo" : "";
    o_rawlogFileName += (calibConfig.onlyRGBD) ? "_rgbd" : "";
    o_rawlogFileName += "_processed.rawlog";

    CFileGZOutputStream o_rawlog(o_rawlogFileName);

    //
    // Process rawlog
    //

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;
    size_t obsLabelsReplaced = 0;

    cout << "    Process: ";
    cout.flush();

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,observations,obs,obsIndex) )
    {
        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        if ( obs->sensorLabel == replaceSensorLabel )
        {
            obs->sensorLabel = replaceSensorLabelAs;

            o_rawlog << obs;

            obsLabelsReplaced++;
        }
        else if (!keepOnlyProcessed)
            o_rawlog << obs;
    }

    cout << endl << "    Number of observation labels replaced: "
         << obsLabelsReplaced << endl;

    cout << "  [INFO] Rawlog saved as " << o_rawlogFileName << endl << endl;
}


//-----------------------------------------------------------
//
//                     loadParameters
//
//-----------------------------------------------------------

int loadParameters(int argc, char* argv[])
{
    if ( argc >= 2 )
    {
        i_rawlogFilename = argv[1];

        for ( size_t arg = 2; arg < argc; arg++ )
        {
            if ( !strcmp(argv[arg],"-decimate") )
            {
                decimate = atoi(argv[arg+1]);
                arg++;
            }
            else if ( !strcmp(argv[arg],"-config") )
            {
                configFileName = argv[arg+1];
                arg++;
            }
            else if ( !strcmp(argv[arg],"-keepOnlyProcessed") )
            {
                keepOnlyProcessed = true;
                cout << "  [INFO] Keeping only processed observations."  << endl;
            }
            else if ( !strcmp(argv[arg],"-remove3DPointClouds") )
            {
                onlyRemove3DPointClouds = true;
                cout << "  [INFO] Remove 3D point clouds."  << endl;
            }
            else if ( !strcmp(argv[arg],"-removeEmptyObs") )
            {
                removeEmptyObs = atof(argv[arg+1]);;
                cout << "  [INFO] Removing empty observations with a factor of null measuremets of "
                     << removeEmptyObs << endl;
                arg++;
            }
            else if ( !strcmp(argv[arg],"-removeWeirdObs") )
            {
                removeWeirdObs = atof(argv[arg+1]);;
                cout << "  [INFO] Setting to 0 (null) observations with a factor of valid measuremets lower than "
                     << removeWeirdObs << endl;
                arg++;
            }
            else if ( !strcmp(argv[arg],"-replaceLabel") )
            {
                replaceSensorLabel = argv[arg+1];
                replaceSensorLabelAs = argv[arg+2];
                arg+=2;
            }
            else if ( !strcmp(argv[arg],"-only_2DLaser") )
            {
                calibConfig.only2DLaser = true;
                cout << "  [INFO] Processing only hokuyo observations."  << endl;
            }
            else if ( !strcmp(argv[arg],"-saveAsPlainText") )
            {
                saveAsPlainText = true;
                cout << "  [INFO] Saving as plain text."  << endl;
            }
            else if ( !strcmp(argv[arg],"-only_rgbd") )
            {
                calibConfig.onlyRGBD = true;
                cout << "  [INFO] Processing only rgbd observations."  << endl;
            }
            else if ( !strcmp(argv[arg],"-h") )
            {
                showUsageInformation();
                return 0;
            }
            else
            {
                cout << "  [ERROR] Unknown option " << argv[arg] << endl;
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

    return 1;
}


//-----------------------------------------------------------
//
//                   removeEmptyObservations
//
//-----------------------------------------------------------

void removeEmptyObservations()
{
    //
    // Check input rawlog
    //

    if (!mrpt::system::fileExists(i_rawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << i_rawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    CFileGZInputStream i_rawlog(i_rawlogFilename);

    cout << "  [INFO] Processing rawlog " << i_rawlogFilename << endl;
    cout << "  [INFO] Removing 3D observations with a factor of invalid measurements higher than "
         << removeEmptyObs << endl;

    string o_rawlogFileName;

    //
    // Set output rawlog file
    //

    o_rawlogFileName.assign(i_rawlogFilename.begin(),
                            i_rawlogFilename.end()-7);
    o_rawlogFileName += (calibConfig.only2DLaser) ? "_hokuyo" : "";
    o_rawlogFileName += (calibConfig.onlyRGBD) ? "_rgbd" : "";
    o_rawlogFileName += "_processed.rawlog";

    CFileGZOutputStream o_rawlog(o_rawlogFileName);

    //
    // Process rawlog
    //

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;
    size_t obsRemoved = 0;

    cout << "    Process: ";
    cout.flush();

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,observations,obs,obsIndex) )
    {
        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        if (IS_CLASS(obs, CObservation3DRangeScan))
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            int rows = obs3D->rangeImage.rows();
            int cols = obs3D->rangeImage.cols();

            int invalidMeasurements = 0;

            for ( int r = 0; r < rows; r++ )
                for ( int c = 0; c < cols; c++ )
                    if ( !obs3D->rangeImage(r,c) )
                        invalidMeasurements++;

            float factor = invalidMeasurements / (float)(rows*cols);

            if ( factor < removeEmptyObs )
                o_rawlog << obs3D;
            else
                obsRemoved++;
        }


    }

    cout << endl << "    Number of observation removed: "
         << obsRemoved << " of " << obsIndex << endl;

    cout << "  [INFO] Rawlog saved as " << o_rawlogFileName << endl << endl;
}


//-----------------------------------------------------------
//
//                   removeWeirdObservations
//
//-----------------------------------------------------------

void removeWeirdObservations()
{
    //
    // Check input rawlog
    //

    if (!mrpt::system::fileExists(i_rawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << i_rawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    CFileGZInputStream i_rawlog(i_rawlogFilename);

    cout << "  [INFO] Processing rawlog " << i_rawlogFilename << endl;

    string o_rawlogFileName;

    //
    // Set output rawlog file
    //

    o_rawlogFileName.assign(i_rawlogFilename.begin(),
                            i_rawlogFilename.end()-7);
    o_rawlogFileName += (calibConfig.only2DLaser) ? "_hokuyo" : "";
    o_rawlogFileName += (calibConfig.onlyRGBD) ? "_rgbd" : "";
    o_rawlogFileName += "_processed.rawlog";

    CFileGZOutputStream o_rawlog(o_rawlogFileName);

    //
    // Process rawlog
    //

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;
    size_t obsSet = 0;

    cout << "    Process: ";
    cout.flush();

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,observations,obs,obsIndex) )
    {
        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        if (IS_CLASS(obs, CObservation3DRangeScan))
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            int rows = obs3D->rangeImage.rows();
            int cols = obs3D->rangeImage.cols();

            int validMeasurements = 0;

            for ( int r = 0; r < rows; r++ )
                for ( int c = 0; c < cols; c++ )
                    if ( obs3D->rangeImage(r,c) )
                        validMeasurements++;

            float factor = validMeasurements / (float)(rows*cols);

            //cout << "Index [" << obsIndex-1 << "] valids " << validMeasurements
            //     << " of " << (float)(rows*cols) << " factor " << factor;

            if ( factor > removeWeirdObs )
                o_rawlog << obs3D;
            else
            {
                for ( int r = 0; r < rows; r++ )
                    for ( int c = 0; c < cols; c++ )
                        obs3D->rangeImage(r,c) = 0;

                if ( obs3D->hasPoints3D )
                    obs3D->project3DPointsFromDepthImage();

                o_rawlog << obs3D;

                //cout << " set to 0!";
                obsSet++;
            }

            //cout << endl;
        }


    }

    cout << endl << "    Number of observations set to 0 (null): "
         << obsSet << " of " << obsIndex << endl;

    cout << "  [INFO] Rawlog saved as " << o_rawlogFileName << endl << endl;
}


//-----------------------------------------------------------
//
//                   removeEmptyObservations
//
//-----------------------------------------------------------

void remove3DPointClouds()
{
    //
    // Check input rawlog
    //

    if (!mrpt::system::fileExists(i_rawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << i_rawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    CFileGZInputStream i_rawlog(i_rawlogFilename);

    cout << "  [INFO] Processing rawlog " << i_rawlogFilename << endl;
    cout << "  [INFO] Removing 3D point clouds " << endl;

    string o_rawlogFileName;

    //
    // Set output rawlog file
    //

    o_rawlogFileName.assign(i_rawlogFilename.begin(),
                            i_rawlogFilename.end()-7);
    o_rawlogFileName += (calibConfig.only2DLaser) ? "_hokuyo" : "";
    o_rawlogFileName += (calibConfig.onlyRGBD) ? "_rgbd" : "";
    o_rawlogFileName += "_processed.rawlog";

    CFileGZOutputStream o_rawlog(o_rawlogFileName);

    //
    // Process rawlog
    //

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;
    size_t pointCloudsRemoved = 0;

    cout << "    Process: ";
    cout.flush();

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,observations,obs,obsIndex) )
    {
        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        if (IS_CLASS(obs, CObservation3DRangeScan))
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            if ( obs3D->hasPoints3D || obs3D->points3D_x.size() )
            {
                obs3D->points3D_x.clear();
                obs3D->points3D_y.clear();
                obs3D->points3D_z.clear();

                obs3D->hasPoints3D = false;

                pointCloudsRemoved++;
            }

            o_rawlog << obs3D;
        }


    }

    cout << endl << "    Number of point clouds removed: "
         << pointCloudsRemoved << " of " << obsIndex << endl;

    cout << "  [INFO] Rawlog saved as " << o_rawlogFileName << endl << endl;
}


//-----------------------------------------------------------
//
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

        cout << endl << "-----------------------------------------------------" << endl;
        cout <<         "              Process rawlog app.                    " << endl;
        cout <<         "           [Object Labeling Tookit]                  " << endl;
        cout <<         "-----------------------------------------------------" << endl << endl;


        //
        // Load paramteres

        int res = loadParameters(argc,argv);

        if (res<0)
            return -1;
        else if (!res)
            return 0;

        //
        // Load config information

        if ( !configFileName.empty() )
            loadConfig();

        //
        // What to do?

        if ( decimate )
            decimateRawlog();

        else if ( !replaceSensorLabel.empty() )
            replaceLabel();

        else if ( removeEmptyObs )
            removeEmptyObservations();

        else if ( removeWeirdObs )
            removeWeirdObservations();

        else if ( onlyRemove3DPointClouds )
            remove3DPointClouds();

        else if ( saveAsPlainText )
        {

            string withoutExtension = i_rawlogFilename.substr(0,i_rawlogFilename.size()-7);

            OLT::CSaveAsPlainText editor;

            editor.setOption("output_file",withoutExtension+".txt");
            editor.setOption("output_obs_dir",withoutExtension+"/");
            editor.setOption("generate_point_clouds",0);

            editor.setInputRawlog(i_rawlogFilename);
            editor.process();
        }

        else if ( setCalibrationParameters )
            processRawlog();

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

