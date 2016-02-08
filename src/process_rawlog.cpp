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
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/interp_fit.h>

#include "opencv2/imgproc/imgproc.hpp"

#include <numeric>
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

//
// Useful variables
//

string configFileName;
bool setCalibrationParameters = true;
bool computeScaleBetweenRGBDSensors = false;

struct TCalibrationConfig{

    bool only2DLaser;
    bool onlyRGBD;
    bool useDefaultIntrinsics;
    bool applyCLAMS;
    bool scaleDepthInfo;
    int  equalizeRGBHistograms;
    double truncateDepthInfo;
    string i_rawlogFilename;

    TCalibrationConfig() : only2DLaser(false),
        onlyRGBD(false), useDefaultIntrinsics(true), equalizeRGBHistograms(false),
        truncateDepthInfo(0), i_rawlogFilename("")
    {}

} calibConfig;

struct TScaleCalibrationConfig{

    float resolution;
    float lowerRange;
    float higerRange;
    string referenceSensor;
    string scaledSensor;
    string o_scaleCalibrationFile;
    bool   generateInverseScale;
    bool   computeTransitiveScale;
    string referenceScaleCalibrationFile;
    string scaledScaleCalibrationFile;
    vector<string> v_referenceRawlogs;
    vector<string> v_scaledRawlogs;

    TScaleCalibrationConfig()
    {}
} scaleCalibrationConfig;

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
    computeScaleBetweenRGBDSensors  = config.read_bool("GENERAL","compute_scale_between_RGBD_sensors",false,false);

    // Check the processing modes activated by the user
    if (( setCalibrationParameters && computeScaleBetweenRGBDSensors ) // both
       || ( !setCalibrationParameters && !computeScaleBetweenRGBDSensors ) ) // anyone
        throw std::logic_error("You have to chose between"
           "set_calibration_parameters or compute_sacale_between_RGBD_sensors");


    //
    // Load calibration parameters

    if ( setCalibrationParameters )
    {
        calibConfig.useDefaultIntrinsics  = config.read_bool("CALIBRATION","use_default_intrinsics",true,true);
        calibConfig.equalizeRGBHistograms = config.read_int("CALIBRATION","equalize_RGB_histograms",0,true);
        calibConfig.truncateDepthInfo     = config.read_double("CALIBRATION","truncate_depth_info",0,true);
        calibConfig.applyCLAMS            = config.read_bool("CALIBRATION","apply_CLAMS_Calibration_if_available",false,true);
        calibConfig.scaleDepthInfo        = config.read_bool("CALIBRATION","scale_depth_info",false,true);


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

                if ( !depthScaleModelPath.empty() )
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

    //
    // Load parameters for computing the scale between two RGBD sensors

    else if ( computeScaleBetweenRGBDSensors )
    {
        TScaleCalibrationConfig &scc = scaleCalibrationConfig;

        scc.resolution = config.read_double("SCALE_BETWEEN_RGBD_SENSORS","resolution",0,true);
        scc.lowerRange = config.read_double("SCALE_BETWEEN_RGBD_SENSORS","lower_range",0,true);
        scc.higerRange = config.read_double("SCALE_BETWEEN_RGBD_SENSORS","higher_range",0,true);
        scc.referenceSensor = config.read_string("SCALE_BETWEEN_RGBD_SENSORS","reference_sensor","",true);
        scc.scaledSensor = config.read_string("SCALE_BETWEEN_RGBD_SENSORS","scaled_sensor","",true);
        scc.o_scaleCalibrationFile = config.read_string("SCALE_BETWEEN_RGBD_SENSORS","scale_calibration_file","",true);
        scc.generateInverseScale = config.read_bool("SCALE_BETWEEN_RGBD_SENSORS","generate_inverse_scale",false,true);
        scc.computeTransitiveScale = config.read_bool("SCALE_BETWEEN_RGBD_SENSORS","compute_transitive_scale",false,true);

        if ( scc.computeTransitiveScale )
        {
            scc.referenceScaleCalibrationFile = config.read_string("SCALE_BETWEEN_RGBD_SENSORS","reference_scale_calibration_file","",true);;
            scc.scaledScaleCalibrationFile =  config.read_string("SCALE_BETWEEN_RGBD_SENSORS","scaled_scale_calibration_file","",true);;
        }

        if ( scc.generateInverseScale )
        {
            string aux = scc.referenceSensor;
            scc.referenceSensor = scc.scaledSensor;
            scc.scaledSensor = aux;
        }

        vector<string> keys;
        config.getAllKeys("SCALE_BETWEEN_RGBD_SENSORS",keys);

        int rawlogIndex = 1;
        bool keepLoading = true;

        while ( keepLoading )
        {
            string referenceRawlog = mrpt::format("reference_rawlog_%i",rawlogIndex);
            string scaledRawlog = mrpt::format("scaled_rawlog_%i",rawlogIndex);

            if ( (std::find(keys.begin(), keys.end(), referenceRawlog) != keys.end())
                 && (std::find(keys.begin(), keys.end(), scaledRawlog) != keys.end() ) )
            {
                scc.v_referenceRawlogs.push_back(
                            config.read_string("SCALE_BETWEEN_RGBD_SENSORS",referenceRawlog,"",true) );
                scc.v_scaledRawlogs.push_back(
                            config.read_string("SCALE_BETWEEN_RGBD_SENSORS",scaledRawlog,"",true) );

                rawlogIndex++;
            }
            else
                keepLoading = false;
         }

        if ( scc.generateInverseScale )
        {
            vector<string> aux = scc.v_referenceRawlogs;
            scc.v_referenceRawlogs = scc.v_scaledRawlogs;
            scc.v_scaledRawlogs = aux;
        }
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
//                      equalizeCLAHE
//
//-----------------------------------------------------------

void equalizeCLAHE( CObservation3DRangeScanPtr obs3D )
{
    cv::Mat cvImg = cv::cvarrToMat( obs3D->intensityImage.getAs<IplImage>() );

    cv::Mat lab_image;
    cv::cvtColor(cvImg, lab_image, CV_BGR2HSV);

    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[2], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[2]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    cv::Mat image_clahe;
    cv::cvtColor(lab_image, image_clahe, CV_HSV2BGR);

    IplImage* final;
    final = cvCreateImage(cvSize(image_clahe.cols,image_clahe.rows),8,3);
    IplImage ipltemp=image_clahe;
    cvCopy(&ipltemp,final);

    //cout << "N channels: " << final->nChannels << endl;

    obs3D->intensityImage.setFromIplImageReadOnly(final);

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
//
//                      processRawlog
//
//-----------------------------------------------------------

void processRawlog()
{
    //
    // Check input rawlog
    //

    if (!mrpt::system::fileExists(calibConfig.i_rawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << calibConfig.i_rawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    CFileGZInputStream i_rawlog(calibConfig.i_rawlogFilename);

    cout << "  [INFO] Working with " << calibConfig.i_rawlogFilename << endl;
    if (!calibConfig.equalizeRGBHistograms)
        cout << "  [INFO] Not equalizing RGB histograms" << endl;
    else if ( calibConfig.equalizeRGBHistograms == 1 )
        cout << "  [INFO] Regular RGB histogram equalization" << endl;
    else if ( calibConfig.equalizeRGBHistograms == 2 )
        cout << "  [INFO] CLAHE RGB histogram equalization" << endl;
    else
        cerr << "  [ERROR] Unkwnon RGB histogram equalization" << endl;

    if ( !calibConfig.truncateDepthInfo )
        cout << "  [INFO] Not truncating depth information" << endl;
    else
        cout << "  [INFO] Truncating depth information from a distance of "
             << calibConfig.truncateDepthInfo << "m" << endl;

    if ( !calibConfig.scaleDepthInfo )
        cout << "  [INFO] Not scaling depth info" << endl;
    else
        cout << "  [INFO] Scaling depth info " << endl;


    if ( calibConfig.applyCLAMS )
#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
        cout << "  [INFO] Undistorting depth images using CLAMS intrinsic calibartion" << endl;
#else
       cout << "  [INFO] Not undistorting depth images using CLAMS, CLAMS is not available" << endl;
#endif
    else
        cout << "  [INFO] Not undistorting depth images using CLAMS." << endl;

    cout.flush();

    string o_rawlogFileName;

    //
    // Set output rawlog file
    //

    o_rawlogFileName.assign(calibConfig.i_rawlogFilename.begin(),
                            calibConfig.i_rawlogFilename.end()-7);
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

                // Project 3D points from the depth image
                obs3D->project3DPointsFromDepthImage();

                // Equalize histogram of RGB images?
                if ( calibConfig.equalizeRGBHistograms == 1 )
                    obs3D->intensityImage.equalizeHistInPlace();
                else if ( calibConfig.equalizeRGBHistograms == 2 )
                    equalizeCLAHE(obs3D);

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


//-----------------------------------------------------------
//
//                computeMeanDepthFromRawlog
//
//-----------------------------------------------------------

float computeMeanDepthFromRawlog(const string rawlogFile, const string sensorLabel)
{
    TScaleCalibrationConfig scc = scaleCalibrationConfig;
    vector<float> v_meanDepths;

    CFileGZInputStream rawlogStream(rawlogFile);

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;

    //
    // Get mean depth value for each RGBD observation

    while ( CRawlog::getActionObservationPairOrObservation(rawlogStream,action,observations,obs,obsIndex) )
    {
        // Check if it is an observation
        if ( !obs )

            continue;
        // RGBD observation? and from the sensor which data are we processing?

        //if ( obs->sensorLabel == sensorLabel )
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            int N_elements = 0;
            double sum = 0.0;

            for ( size_t row = 0; row < obs3D->rangeImage.rows(); row++ )
                for ( size_t col = 0; col < obs3D->rangeImage.cols(); col++ )
                {
                    float value = obs3D->rangeImage(row,col);
                    if ( value )
                    {
                        sum += value;
                        N_elements++;
                    }
                }

            float meanDistance = sum / (float)N_elements;
            v_meanDepths.push_back(meanDistance);
        }
    }

    // Compute mean of the mean distances :)

    float sum = std::accumulate(v_meanDepths.begin(), v_meanDepths.end(), 0.0);
    float mean = sum / (float)v_meanDepths.size();

    return mean;

}

//-----------------------------------------------------------
//
//               saveScaleCalibrationToFile
//
//-----------------------------------------------------------

void saveScaleCalibrationToFile(CVectorFloat &v_meanDepths,
                                CVectorFloat &v_depthScales)
{
    TScaleCalibrationConfig &scc = scaleCalibrationConfig;

    ofstream f( scc.o_scaleCalibrationFile.c_str() );

    if ( !f.is_open() )
        cout << "    [ERROR] Opening file " << scc.o_scaleCalibrationFile << endl;

    cout << "  [INFO] Saving depth scale calibration results to "
         << scc.o_scaleCalibrationFile << endl;

    f << "Reference_sensor " << scc.referenceSensor << endl;
    f << "Scaled_sensor " << scc.scaledSensor << endl;
    f << "Resolution " << scc.resolution << endl;
    f << "Lower_depth " << scc.lowerRange << endl;
    f << "Higher_depth " << scc.higerRange << endl;

    CVectorFloat xs,ys;
    linspace(scc.lowerRange,scc.higerRange,
             1+1/scc.resolution*(scc.higerRange-scc.lowerRange),
             xs);

    mrpt::math::leastSquareLinearFit(xs,ys,v_meanDepths,v_depthScales);

    for ( int i = 0; i < ys.rows(); i++ )
    {
        //cout << "Depth: " << xs(i) << " scale:" << ys(i) << endl;
        f << ys(i) << " ";
    }

    cout << "  [INFO] Done! " << endl << endl;
}


//-----------------------------------------------------------
//
//                       computeScale
//
//-----------------------------------------------------------

void computeScale()
{

    //
    // Show configuration parameters

    TScaleCalibrationConfig &scc = scaleCalibrationConfig;
    size_t N_rawlogs = scc.v_referenceRawlogs.size();
    const size_t N_sensors = 2;

    cout << "  [INFO] Computing depth scale between RGBD sensors with the "
            "following configuration:" << endl;

    cout << "    Reference sensor: " << scc.referenceSensor << endl;
    cout << "    Scaled sensor   : " << scc.scaledSensor << endl;
    cout << "    Resolution      : " << scc.resolution << endl;
    cout << "    Lower range     : " << scc.lowerRange << endl;
    cout << "    Higer range     : " << scc.higerRange << endl;
    cout << "    Sacale calib. file: " << scc.o_scaleCalibrationFile << endl;
    cout << "    # of sensors    : " << N_sensors << endl;
    cout << "    # of rawlogs    : " << N_rawlogs << endl;

    for ( size_t i_rawlog = 0; i_rawlog < N_rawlogs; i_rawlog++ )
    {
        if (!mrpt::system::fileExists(scc.v_referenceRawlogs[i_rawlog]))
        {
            cerr << "    [ERROR] Rawlog file " << scc.v_referenceRawlogs[i_rawlog]
                 << " doesn't exist." << endl;
            return;
        }

        if (!mrpt::system::fileExists(scc.v_scaledRawlogs[i_rawlog]))
        {
            cerr << "    [ERROR] Rawlog file " << scc.v_scaledRawlogs[i_rawlog]
                 << " doesn't exist." << endl;
            return;
        }

        cout << "      Rawlog of reference sensor " << i_rawlog <<" name : " <<
                scc.v_referenceRawlogs[i_rawlog] << endl;
        cout << "      Rawlog of scaled sensor " << i_rawlog <<" name    : " <<
                scc.v_scaledRawlogs[i_rawlog] << endl;
    }

    cout << "  [INFO] Calibrating scales " << endl;

    //
    // Data to be filled

    CVectorFloat v_depthScales(N_rawlogs);
    CVectorFloat v_meanDepths(N_rawlogs);

    //
    // Process rawlogs preparing data to calibrate the scale

    for ( size_t i_rawlog = 0; i_rawlog < N_rawlogs; i_rawlog++ )
    {

        double refSensorMeanDepth = computeMeanDepthFromRawlog(scc.v_referenceRawlogs[i_rawlog],
                                                               scc.referenceSensor );
        double scaledSensorMeanDepth = computeMeanDepthFromRawlog(scc.v_scaledRawlogs[i_rawlog],
                                                               scc.scaledSensor );
        v_meanDepths(i_rawlog) = scaledSensorMeanDepth;
        v_depthScales(i_rawlog) = refSensorMeanDepth/scaledSensorMeanDepth;

        cout << "    refSensorMeanDepth: " << refSensorMeanDepth <<
                " scaledSensorMeanDepth: " << scaledSensorMeanDepth <<
                " scale: " << v_depthScales[i_rawlog] << endl;
    }

    //
    // Save depth scale calibration to file

    saveScaleCalibrationToFile(v_meanDepths,v_depthScales);
}


//-----------------------------------------------------------
//
//                          main
//
//-----------------------------------------------------------

void computeTransitiveScale()
{

    //
    // Show configuration parameters

    TScaleCalibrationConfig &scc = scaleCalibrationConfig;

    cout << "  [INFO] Computing transitive depth scale between RGBD sensors with the "
            "following configuration:" << endl;

    cout << "    Reference scale calibration file: " << scc.referenceScaleCalibrationFile << endl;
    cout << "    Scaled scale calibration file   : " << scc.scaledScaleCalibrationFile << endl;
    cout << "    Reference sensor                : " << scc.referenceSensor << endl;
    cout << "    Scaled sensor                   : " << scc.scaledSensor << endl;

    loadScaleCalibrationFromFile( scc.referenceScaleCalibrationFile );
    loadScaleCalibrationFromFile( scc.scaledScaleCalibrationFile );

    if ( v_scaleCalibrations[0].referenceSensor != scc.referenceSensor )
        cout << "[WARNING!] Reference sensor in " << scc.referenceScaleCalibrationFile
             << " doesn't match reference sensor label set in the configuration file" << endl;

    if ( v_scaleCalibrations[1].referenceSensor != scc.scaledSensor )
        cout << "[WARNING!] Reference sensor in " << scc.referenceScaleCalibrationFile
             << " doesn't match scaled sensor label set in the configuration file" << endl;

    if ( v_scaleCalibrations[0].resolution != v_scaleCalibrations[1].resolution )
        throw logic_error("Scale calibration files have a different resolution.");

    if ( v_scaleCalibrations[0].lowerRange != v_scaleCalibrations[1].lowerRange )
        throw logic_error("Scale calibration files have a different lowerRange.");

    if ( v_scaleCalibrations[0].higerRange != v_scaleCalibrations[1].higerRange )
        throw logic_error("Scale calibration files have a different higerRange.");

    ofstream f( scc.o_scaleCalibrationFile.c_str() );

    if ( !f.is_open() )
        cout << "    [ERROR] Opening file " << scc.o_scaleCalibrationFile << endl;

    cout << "  [INFO] Saving depth scale calibration results to "
         << scc.o_scaleCalibrationFile << endl;

    f << "Reference_sensor " << scc.referenceSensor << endl;
    f << "Scaled_sensor " << scc.scaledSensor << endl;
    f << "Resolution " << v_scaleCalibrations[0].resolution << endl;
    f << "Lower_depth " << v_scaleCalibrations[0].lowerRange << endl;
    f << "Higher_depth " << v_scaleCalibrations[0].higerRange << endl;

    for ( int i = 0; i < v_scaleCalibrations[0].scaleMultipliers.size(); i++ )
    {
        f << v_scaleCalibrations[0].scaleMultipliers[i]/v_scaleCalibrations[1].scaleMultipliers[i] << " ";
    }

    cout << "  [INFO] Done! " << endl << endl;
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
        //

        calibConfig.i_rawlogFilename = argv[1];
        configFileName = argv[2];

        if ( argc >= 3 )
        {


            for ( size_t arg = 3; arg < argc; arg++ )
            {
                if ( !strcmp(argv[arg],"-only_2DLaser") )
                {
                    calibConfig.only2DLaser = true;
                    cout << "  [INFO] Processing only hokuyo observations."  << endl;
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

        //
        // Load config information

        loadConfig();

        //
        // What to do?

        if ( setCalibrationParameters )
            processRawlog();

        else if ( computeScaleBetweenRGBDSensors && !scaleCalibrationConfig.computeTransitiveScale )
            computeScale();

        else if ( computeScaleBetweenRGBDSensors && scaleCalibrationConfig.computeTransitiveScale )
            computeTransitiveScale();

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

