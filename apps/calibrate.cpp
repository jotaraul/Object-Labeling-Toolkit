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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils.h>
#include <mrpt/system/os.h>

#include "opencv2/imgproc/imgproc.hpp"

#include <numeric>
#include <iostream>
#include <fstream>

#ifdef USING_CLAMS_INTRINSIC_CALIBRATION
    #include <clams/discrete_depth_distortion_model.h>
#endif

#define NUM_ASUS_SENSORS 4

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

bool fromHomogeneusMatrixToPose;
string homogeneusMatrixFile;

string configFileName;
bool computeScaleBetweenRGBDSensors = false;

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


    computeScaleBetweenRGBDSensors  = config.read_bool("GENERAL","compute_scale_between_RGBD_sensors",false,false);

    //
    // Load parameters for computing the scale between two RGBD sensors

    if ( computeScaleBetweenRGBDSensors )
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
//                    showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "Usage information. At least one expected arguments: " << endl <<
            "    (1) Configuration file." << endl;
   cout << "Then, optional parameters:" << endl <<
            "    -h                         : Shows this help." << endl <<
            "    -fromHomogeneusMatrixToPose: Show 3D pose from a homogeneus matrix within a file." << endl;

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

inline Eigen::Matrix4f getPoseEigenMatrix(const mrpt::poses::CPose3D & pose)
{
  Eigen::Matrix4f pose_mat;
  mrpt::math::CMatrixDouble44 pose_mat_mrpt;
  pose.getHomogeneousMatrix(pose_mat_mrpt);
  pose_mat << pose_mat_mrpt(0,0), pose_mat_mrpt(0,1), pose_mat_mrpt(0,2), pose_mat_mrpt(0,3),
              pose_mat_mrpt(1,0), pose_mat_mrpt(1,1), pose_mat_mrpt(1,2), pose_mat_mrpt(1,3),
              pose_mat_mrpt(2,0), pose_mat_mrpt(2,1), pose_mat_mrpt(2,2), pose_mat_mrpt(2,3),
              pose_mat_mrpt(3,0), pose_mat_mrpt(3,1), pose_mat_mrpt(3,2), pose_mat_mrpt(3,3) ;
  return  pose_mat;
}

//-----------------------------------------------------------
//
//             showPoseFromHomogeneusMatrix
//
//-----------------------------------------------------------

void showPoseFromHomogeneusMatrix()
{
    Eigen::Matrix4f homoMatrix;
    cout << "  [INFO] Loading homogeneus matrix from file: " << homogeneusMatrixFile << endl;
    homoMatrix.loadFromTextFile( homogeneusMatrixFile );

    cout << endl << "    Homogeneus matrix" << endl;
    cout << homoMatrix << endl << endl;

    mrpt::math::CMatrixDouble44 homoMatrixMRPT(homoMatrix);
    mrpt::poses::CPose3D rel_pose(homoMatrixMRPT);

    cout << "Relative pos: " << rel_pose << endl;

    mrpt::poses::CPose3D rgbd1_pose(0.271,-0.035,1.015,DEG2RAD(-45),DEG2RAD(0),DEG2RAD(-90));

    cout << "Pose rgbd1  : " << rgbd1_pose << endl;
    cout << "Pose final  : " << rgbd1_pose + rel_pose << endl << endl;

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
        cout <<         "                 Calibrate app.                      " << endl;
        cout <<         "           [Object Labeling Tookit]                  " << endl;
        cout <<         "-----------------------------------------------------" << endl << endl;

        //
        // Load paramteres
        //        

        if ( argc >= 1 )
        {
            for ( size_t arg = 1; arg < argc; arg++ )
            {
                if ( !strcmp(argv[arg],"-h") )
                {
                    showUsageInformation();
                    return 0;
                }
                else if ( !strcmp(argv[arg],"-fromHomogeneusMatrixToPose") )
                {
                    fromHomogeneusMatrixToPose = true;
                    homogeneusMatrixFile = argv[arg+1];

                    arg++;
                    cout << "  [INFO] Showing 3D pose from homogeneus matrix within a file." << endl;
                }
                else if ( !strcmp(argv[arg],"-config") )
                {
                    configFileName = homogeneusMatrixFile = argv[arg];
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
        // What to do?

        if ( fromHomogeneusMatrixToPose )
            showPoseFromHomogeneusMatrix();
        else
        {
            //
            // Load config information

            loadConfig();

            if ( computeScaleBetweenRGBDSensors && !scaleCalibrationConfig.computeTransitiveScale )
                computeScale();

            else if ( computeScaleBetweenRGBDSensors && scaleCalibrationConfig.computeTransitiveScale )
                computeTransitiveScale();
        }

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



//! Return the rotation vector of the input pose
/*inline Eigen::Matrix4f getPoseEigenMatrix(const mrpt::poses::CPose3D & pose)
{
  Eigen::Matrix4f pose_mat;
  mrpt::math::CMatrixDouble44 pose_mat_mrpt;
  pose.getHomogeneousMatrix(pose_mat_mrpt);
  pose_mat << pose_mat_mrpt(0,0), pose_mat_mrpt(0,1), pose_mat_mrpt(0,2), pose_mat_mrpt(0,3),
              pose_mat_mrpt(1,0), pose_mat_mrpt(1,1), pose_mat_mrpt(1,2), pose_mat_mrpt(1,3),
              pose_mat_mrpt(2,0), pose_mat_mrpt(2,1), pose_mat_mrpt(2,2), pose_mat_mrpt(2,3),
              pose_mat_mrpt(3,0), pose_mat_mrpt(3,1), pose_mat_mrpt(3,2), pose_mat_mrpt(3,3) ;
  return  pose_mat;
}
int main (int argc, char ** argv)
{

    cout << "Change the reference system of the calibrattion of RGBD360 multisensor\n";
    float angle_offset = 22.5; //45
    Eigen::Matrix4f rot_offset = Eigen::Matrix4f::Identity(); rot_offset(1,1) = rot_offset(2,2) = cos(DEG2RAD(angle_offset)); rot_offset(1,2) = -sin(DEG2RAD(angle_offset)); rot_offset(2,1) = -rot_offset(1,2);
    // Load initial calibration
    cout << "Load initial calibration\n";
    mrpt::poses::CPose3D pose[NUM_ASUS_SENSORS];
    pose[0] = mrpt::poses::CPose3D(0.285, 0, 1.015, DEG2RAD(0), DEG2RAD(1.3), DEG2RAD(-90));
    pose[1] = mrpt::poses::CPose3D(0.271, -0.031, 1.015, DEG2RAD(-45), DEG2RAD(0), DEG2RAD(-90));
    pose[2] = mrpt::poses::CPose3D(0.271, 0.031, 1.125, DEG2RAD(45), DEG2RAD(2), DEG2RAD(-89));
    pose[3] = mrpt::poses::CPose3D(0.24, -0.045, 0.975, DEG2RAD(-90), DEG2RAD(1.5), DEG2RAD(-90));
//    int rgbd180_arrangement[4] = {1,8,2,7};

    Eigen::Matrix4f Rt_[4];
    Eigen::Matrix4f Rt_raul[4];
    Eigen::Matrix4f relative_edu[4];
    Eigen::Matrix4f relative_raul[4];
    Eigen::Matrix4f Rt_raul_new[4];

    Eigen::Matrix4f change_ref = Eigen::Matrix4f::Zero();
//    change_ref(0,1) = 1.f;
//    change_ref(1,2) = 1.f;
//    change_ref(2,0) = 1.f;
//    change_ref(3,3) = 1.f;

//    change_ref(0,2) = 1.f;
//    change_ref(1,1) = -1.f;
//    change_ref(2,0) = 1.f;
//    change_ref(3,3) = 1.f;

    change_ref(0,2) = 1.f;
    change_ref(1,0) = -1.f;
    change_ref(2,1) = -1.f;
    change_ref(3,3) = 1.f;


    for(size_t sensor_id=0; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
    {
        Rt_[sensor_id].loadFromTextFile( mrpt::format("Rt_0%i.txt",sensor_id+1) );
        Rt_raul[sensor_id] = getPoseEigenMatrix( pose[sensor_id] ); //.inverse();
        //cout << " Rt_raul \n" << pose[sensor_id].getHomogeneousMatrixVal() << endl << Rt_raul[sensor_id] << endl;

        if(sensor_id > 0)
        {
            relative_edu[sensor_id] = Rt_[0].inverse() * Rt_[sensor_id];
            //relative_raul[sensor_id] = change_ref.transpose() * relative_edu[sensor_id] * change_ref;
            relative_raul[sensor_id] = change_ref * relative_edu[sensor_id] * change_ref.inverse();

            //relative_edu[sensor_id] = Rt_[0] * Rt_[sensor_id];
            //relative_raul[sensor_id] = relative_edu[sensor_id] * change_ref;

            Rt_raul_new[sensor_id] = Rt_raul[0] * relative_raul[sensor_id];

            cout << " rel edu \n" << relative_edu[sensor_id] << endl;
            cout << " rel edu to raul \n" << relative_raul[sensor_id] << endl;
            cout << " rel raul \n" << Rt_raul[0].inverse() * Rt_raul[sensor_id] << endl;
            cout << " ref sensor \n" <<Rt_raul[0] << endl;
            cout << " raul \n" << Rt_raul[sensor_id] << endl;
            cout << " new \n" << Rt_raul_new[sensor_id] << endl;

            mrpt::math::CMatrixDouble44 matrix(Rt_raul_new[sensor_id]);
            mrpt::poses::CPose3D poseFinal(matrix);

            cout << "Pose final paquito: " << endl << poseFinal << endl;


            mrpt::system::pause();

            ofstream calibFile;
              string calibFileName = mrpt::format("ref_Rt_0%u.txt", sensor_id);
              calibFile.open(calibFileName.c_str());
              if (calibFile.is_open())
              {
                calibFile << Rt_raul_new[sensor_id];
                calibFile.close();
              }
              else
                cout << "Unable to open file " << calibFileName << endl;
        }
    }
    Rt_raul_new[0] = Rt_raul[0];

  cout << "EXIT\n";

  return (0);
}*/
