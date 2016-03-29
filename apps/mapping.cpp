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

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/gui.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/maps/PCL_adapters.h>
#include <mrpt/maps/CColouredPointsMap.h>

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>

#include <mrpt/slam/CICP.h>

// ICP 3D MRPT

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/system/threads.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CFrustum.h>

// ICP 3D PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/warp_point_rigid_3d.h>

#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

//#include <pcl/registration/lum.h>

// NDT PCL
#include <pcl/registration/ndt.h>

// DIFODO
#include <Difodo_multi_datasets.h>

//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

#include <iostream>
#include <fstream>

#include <numeric> // std::accumulate

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace std;
using namespace mrpt::math;

using namespace pcl;

//
// Configuration
//

string i_rawlogFileName;
string o_rawlogFileName;

CFileGZInputStream i_rawlog;
CFileGZOutputStream o_rawlog;

string simpleMapFile;

bool initialGuessICP2D  = true;
bool initialGuessDifodo = false;
bool difodoVisualization = false;
bool initialGuessGICP   = false;
bool refineLocalization = false;
bool accumulatePast     = false;
bool useKeyPoses        = false;
bool smooth3DObs        = false;
bool useOverlappingObs  = false;
bool manuallyFix        = false;
bool processBySensor    = false;
bool processInBlock     = false;
bool visualize2DResults = false;
bool propagateCorrections = true;
size_t decimation       = 1;
size_t decimateMemory   = 0;
double scoreThreshold   = 0.0;

string refinationMethod;

CPose3D lastGoodICPpose;
bool oneGoodICPPose = false;

bool skip_window=false;
//int  ICP_method = (int) icpClassic;
int  ICP_method = (int) icpLevenbergMarquardt;

CPose2D		initialPose(0.8f,0.0f,(float)DEG2RAD(0.0f));
gui::CDisplayWindowPlotsPtr	win = CDisplayWindowPlotsPtr(new CDisplayWindowPlots("ICP results"));

ofstream trajectoryFile("trajectory.txt",ios::trunc);

CDisplayWindow3DPtr window;
CDisplayWindow3DPtr window2;
CDisplayWindow3DPtr window3;

//Increase this values to get more precision. It will also increase run time.
const size_t HOW_MANY_YAWS=360;
const size_t HOW_MANY_PITCHS=360;

const size_t SECS_PER_MIN = 60;
const size_t SECS_PER_HOUR = 3600;

const double KEY_POSE_DIST_THRESHOLD = 0.3;
const double KEY_POSE_ANGLE_THRESHOLD = DEG2RAD((float)20);

struct TRobotPose
{
    TTimeStamp  time;
    TPose2D     pose;
    float      goodness;
};

struct T3DRangeScan
{
    CObservation3DRangeScanPtr obs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHullCloud;
    vector<pcl::Vertices> polygons;

    T3DRangeScan() : obs(new CObservation3DRangeScan()),
        convexHullCloud(new pcl::PointCloud<pcl::PointXYZ>()) {}
};

vector< TRobotPose >    v_robotPoses;
vector< T3DRangeScan >  v_3DRangeScans;
vector< CObservation3DRangeScanPtr > v_pending3DRangeScans;
vector< double >    v_refinementGoodness;
vector<string>      RGBD_sensors;

struct TTime
{
    float icp2D;
    float difodo;
    float refine;
    float smoothing;
    float overlapping;
    TTime() : icp2D(0), difodo(0), refine(0), smoothing(0), overlapping(0)
    {}
}time_measures;


//-----------------------------------------------------------
//
//                    showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "  Usage information. Two expected arguments: " << endl <<
            "    (1) Rawlog file." << endl <<
            "    (2) Points map file." << endl;
    cout << "  Then, optional parameters:" << endl <<
            "    -h             : Show this help." << endl <<
            "    -disable_ICP2D : Disable ICP2D as an initial guess for robot localization." << endl <<
            "    -enable_difodo : Enable DIFODO as initial guess for robot localization." << endl <<
            "    -enable_difodoVisualization: Enable DIFODO visualization." << endl <<
            "    -enable_initialGuessGICP : Use GICP to get the initial guess." << endl <<
            "    -enable_ICP    : Enable ICP to refine the RGBD-sensors location." << endl <<
            "    -enable_GICP   : Enable GICP to refine the RGBD-sensors location." << endl <<
            "    -enable_NDT    : Enable NDT to refine the RGBD-sensors location." << endl <<
            "    -enable_ICPNL  : Enable ICP non linear to refine the RGBD-sensors location." << endl <<
            "    -enable_ICPWN  : Enable ICP with normals to refine the RGBD-sensors location." << endl <<
            "    -enable_memory : Accumulate 3D point clouds already registered." << endl <<
            "    -enable_smoothing: Enable smoothing of the 3D point clouds." << endl <<
            "    -enable_keyPoses : Enable the use of key poses only." << endl <<
            "    -enable_manuallyFix <score>: Enable manual alignment when poor score." << endl <<
            "    -enable_processInBlock: Enable the processing of the obs from all sensors in block." << endl <<
            "    -enable_processBySensor: Align all the obs from a sensor with all the obs of other one." << endl <<
            "    -enable_RGBDdecimation: Permits to decimate the number of RGB observations to refine the sensors pose." << endl <<
            "    -enable_memoryDecimation <num>: Decimate the number of obs in the memory by <num>." << endl <<
            "    -enable_visualize2DResults: Visualize localization results in 2D." << endl <<
            "    -disable_propagateCorrections: Disable the propagation of corrections during the refinement." << endl;

}


//-----------------------------------------------------------
//
//                      loadParameters
//
//-----------------------------------------------------------

int loadParameters(int argc, char **argv)
{

    if ( argc < 2 )
    {
        showUsageInformation();
        return 0;
    }

    i_rawlogFileName = argv[1];

    cout << "  [INFO] Loading parameters from command line..." << endl;

    for ( size_t arg = 2; arg < argc; arg++ )
    {
        if ( !strcmp(argv[arg],"-disable_ICP2D") )
        {
            initialGuessICP2D = false;
            cout << "  [INFO] Disabled ICP2D to guess the robot localization."  << endl;
        }
        else if ( !strcmp(argv[arg],"-enable_difodo") )
        {
            initialGuessDifodo = true;
            initialGuessICP2D = false;
            cout << "  [INFO] Enabled Difodo odometry to guess the robot localization."  << endl;
        }
        else if ( !strcmp(argv[arg],"-enable_difodoVisualization") )
        {
            difodoVisualization = true;
            cout << "  [INFO] Enabled Difodo visualization."  << endl;
        }
        else if ( !strcmp(argv[arg], "-map") )
        {
            simpleMapFile = argv[arg+1];
            cout << "  [INFO] Using geometric map file: " << simpleMapFile << endl;
            arg++;
        }
        else if ( !strcmp(argv[arg], "-enable_initialGuessGICP") )
        {
            initialGuessGICP = true;
            cout << "  [INFO] Enabled GICP for the computation of the initial guess."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_ICP") )
        {
            refineLocalization = true;
            refinationMethod    = "ICP";  // MRPT
            cout << "  [INFO] Enabled ICP."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_GICP") )
        {
            refineLocalization = true;
            refinationMethod     = "GICP"; // PCL
            cout << "  [INFO] Enabled GICP."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_NDT") )
        {
            refineLocalization = true;
            refinationMethod     = "NDT"; // PCL
            cout << "  [INFO] Enabled NDT."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_ICPNL") )
        {
            refineLocalization = true;
            refinationMethod     = "ICPNL"; // PCL
            cout << "  [INFO] Enabled ICP non linear."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_ICPWN") )
        {
            refineLocalization = true;
            refinationMethod     = "ICPWN"; // PCL
            cout << "  [INFO] Enabled ICP with normals."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_memory") )
        {
            accumulatePast   = true;
            cout << "  [INFO] Enabled refinement memory."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_keyPoses") )
        {
            useKeyPoses = true;
            cout << "  [INFO] Enabled key poses."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_overlapping") )
        {
            useOverlappingObs = true;
            cout << "  [INFO] Enabled overlapping obs."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_smoothing") )
        {
            smooth3DObs  = true;
            cout << "  [INFO] Enabled smoothing."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_manuallyFix") )
        {
            scoreThreshold = atof(argv[arg+1]);
            arg++;

            manuallyFix   = true;
            cout << "  [INFO] Enabled manually fixing of missaligned RGB-D obs. ";
            cout << "    Score threshold: " << scoreThreshold << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_processInBlock") )
        {
            processInBlock   = true;
            cout << "  [INFO] Enabled processInBlock."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_visualize2DResults") )
        {
            visualize2DResults   = true;
            cout << "  [INFO] Enabled 2D results visualization."  << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_processBySensor") )
        {
            processBySensor   = true;
            cout << "  [INFO] Enabled processBySensor." << endl;
        }
        else if ( !strcmp(argv[arg], "-disable_propagateCorrections") )
        {
            propagateCorrections   = false;
            cout << "  [INFO] Disabled propagation of corrections during refinement." << endl;
        }

        else if ( !strcmp(argv[arg], "-enable_RGBDdecimation") )
        {
            decimation = atoi(argv[arg+1]);
            arg += 1;

            cout << "  [INFO] Enabled RGBD decimation, using only 1 of each ";
            cout << decimation << " RGBD observations." << endl;
        }
        else if ( !strcmp(argv[arg], "-enable_memoryDecimation") )
        {
            decimateMemory = atoi(argv[arg+1]);

            arg += 1;
            cout << "  [INFO] Enabled memory decimation, using only 1 of each " <<
                     decimateMemory << " RGBD observations in memory." << endl;
        }
        else if ( !strcmp(argv[arg], "-h") )
        {
            showUsageInformation();
            return 0;
        }
        else
        {
            cout << "  [Error] " << argv[arg] << " unknown paramter" << endl;
            showUsageInformation();
            return -1;
        }

    }

    cout << "  [INFO] Loading parameters from command line... DONE!" << endl;

    o_rawlogFileName = i_rawlogFileName.substr(0,i_rawlogFileName.size()-7);

    //
    // Set the output rawlog name
    //

    if ( initialGuessICP2D )
        o_rawlogFileName += "_located-ICP2D";
    else if ( initialGuessDifodo )
        o_rawlogFileName += "_located-DIFODO";
    else
        o_rawlogFileName += "_located";

    if ( refineLocalization && refinationMethod == "GICP" )
        o_rawlogFileName += "-GICP";
    else if ( refineLocalization && refinationMethod == "ICP" )
        o_rawlogFileName += "-ICP";
    else if ( refineLocalization && refinationMethod == "ICPNL" )
        o_rawlogFileName += "-ICPNL";
    else if ( refineLocalization && refinationMethod == "ICPWN" )
        o_rawlogFileName += "-ICPWN";
    else if ( refineLocalization && refinationMethod == "NDT" )
        o_rawlogFileName += "-NDT";

    if ( refineLocalization && accumulatePast )
        o_rawlogFileName += "-memory";

    if ( smooth3DObs )
        o_rawlogFileName += "-smoothed";

    o_rawlogFileName += ".rawlog";
}


//-----------------------------------------------------------
//
//               showPerformanceMeasurements
//
//-----------------------------------------------------------

void showPerformanceMeasurements()
{
    TTimeStamp totalTime;
    TTimeParts totalTimeParts;
    totalTime = secondsToTimestamp(time_measures.refine);
    timestampToParts(totalTime,totalTimeParts);

    cout << "  ---------------------------------------------------" << endl;
    cout << "                   Time statistics" << endl;
    cout << "  ---------------------------------------------------" << endl;
    cout << "    [INFO] time spent by the icp2D process: " << time_measures.icp2D << " sec." << endl;
    cout << "    [INFO] time spent by difodo           : " << time_measures.difodo << " sec." << endl;
    cout << "    [INFO] time spent smoothing           : " << time_measures.smoothing << " sec." << endl;
    cout << "    [INFO] time spent computing hulls     : " << time_measures.overlapping << " sec." << endl;
    cout << "    [INFO] time spent by the icp3D process: " ;

    if ( time_measures.refine )
    {
        cout << (unsigned)totalTimeParts.hour << " hours " <<
                (unsigned)totalTimeParts.minute << " min. " <<
                (unsigned)totalTimeParts.second << " sec." << endl;
    }
    else
        cout << time_measures.refine << " sec." << endl;

    cout << "---------------------------------------------------" << endl;
}

//-----------------------------------------------------------
//
//                     visualizeResults
//
//-----------------------------------------------------------

void visualizeResults()
{
    cout << "  [INFO] Visualizing 2D results ...";
    cout.flush();

    win->hold_on();

    CVectorDouble coord_x, coord_y;
    for ( size_t pos_index = 0; pos_index < v_robotPoses.size(); pos_index++ )
    {
        coord_x.push_back(v_robotPoses[pos_index].pose[0]);
        coord_y.push_back(v_robotPoses[pos_index].pose[1]);
    }

    win->plot(coord_x,coord_y,"-m.3");
    win->plot(coord_x,coord_y,"k.9");

    CVectorDouble coord_x2, coord_y2;

    for ( size_t pos_index = 0; pos_index < v_3DRangeScans.size(); pos_index++ )
    {
        CPose3D pose;
        CVectorDouble v_coords;
        v_3DRangeScans[pos_index].obs ->getSensorPose(pose);
        pose.getAsVector(v_coords);
        coord_x2.push_back(v_coords[0]);
        coord_y2.push_back(v_coords[1]);
    }

    win->plot(coord_x2,coord_y2,"g.8");

    v_3DRangeScans.clear(); // Release some memory!

    win->waitForKey();
}

//-----------------------------------------------------------
//
//                      smoothObss
//
//-----------------------------------------------------------

void smoothObss()
{
    cout << "  [INFO] Smoothing point clouds... ";

    for ( size_t i = 0; i < v_3DRangeScans.size(); i++ )
    {

        CObservation3DRangeScanPtr obs3D = CObservation3DRangeScan::Create();
        obs3D = v_3DRangeScans[i].obs;

        // Remove the sensor pose

        obs3D.make_unique();
        obs3D->sensorPose.setFromValues(0,0,0,0,0,0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud( new pcl::PointCloud<pcl::PointXYZ>() );
        obs3D->project3DPointsFromDepthImageInto( *pcl_cloud, true );

        pcl_cloud->height = 240;
        pcl_cloud->width = 320;

        // Apply bilateral filter

        pcl::FastBilateralFilter<pcl::PointXYZ> m_bilateralFilter;

        m_bilateralFilter.setInputCloud (pcl_cloud);

        m_bilateralFilter.setSigmaS (10);
        m_bilateralFilter.setSigmaR (0.05);
        //m_bilateralFilter.setEarlyDivision (m_bilateralFilterConf.earlyDivision);

        m_bilateralFilter.filter (*pcl_cloud);

        // Fill the original obs

        obs3D->points3D_x.clear();
        obs3D->points3D_y.clear();
        obs3D->points3D_z.clear();

        size_t N_points = pcl_cloud->points.size();
        obs3D->points3D_x.resize(N_points);
        obs3D->points3D_y.resize(N_points);
        obs3D->points3D_z.resize(N_points);

        for ( size_t point_index = 0;
              point_index < N_points;
              point_index++ )
        {
            v_3DRangeScans[i].obs->points3D_x[point_index] = pcl_cloud->points[point_index].x;
            v_3DRangeScans[i].obs->points3D_y[point_index] = pcl_cloud->points[point_index].y;
            v_3DRangeScans[i].obs->points3D_z[point_index] = pcl_cloud->points[point_index].z;
        }
    }

    cout << "done!" << endl;
}


//-----------------------------------------------------------
//
//                    computeConvexHulls
//
//-----------------------------------------------------------

void computeConvexHulls()
{
    cout << "  [INFO] Computing convex hulls to check overlapping... ";

    for ( size_t i = 0; i < v_3DRangeScans.size(); i++ )
    {
        // Get point cloud of old observation
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud ( new pcl::PointCloud<pcl::PointXYZ>());
        v_3DRangeScans[i].obs->project3DPointsFromDepthImageInto(*pointCloud,true);

        pcl::ConvexHull<pcl::PointXYZ> convex_hull;
        convex_hull.setInputCloud(pointCloud);
        convex_hull.setDimension(3);
        convex_hull.reconstruct(*v_3DRangeScans[i].convexHullCloud,
                                v_3DRangeScans[i].polygons);
    }

    cout << " done." << endl;
}


//-----------------------------------------------------------
//
//                      isKeyPose
//
//-----------------------------------------------------------

bool isKeyPose( CPose3D &pose, CPose3D &lastPose)
{
    CVectorDouble pose_v;
    pose.getAsVector( pose_v );

    CVectorDouble lastPose_v;
    lastPose.getAsVector( lastPose_v );

    //    cout << "Last pose    : " << lastPose << endl;
    //    cout << "Current pose : " << pose << endl;

    float dist = sqrt(pow(pose_v[0]-lastPose_v[0],2) +
                      pow(pose_v[1]-lastPose_v[1],2) ); // x-y distance

    float angle = abs( pose_v[3]-lastPose_v[3] ); // Diff in yaw

    //    cout << "Distance: " << dist << " and angle: " << RAD2DEG(angle) << endl;

    if ( ( dist > KEY_POSE_DIST_THRESHOLD )
         || ( angle > KEY_POSE_ANGLE_THRESHOLD ) )
        return true;
    else
        return false;
}


//-----------------------------------------------------------
//
//                      trajectoryICP2D
//
//-----------------------------------------------------------

void trajectoryICP2D( string &simpleMapFile,
                      CObservation2DRangeScanPtr obs2D,
                      double &goodness )
{

    //        CSimplePointsMap map1,map2;
    //        map1.load2D_from_text_file("map1.txt");
    //        map2.load2D_from_text_file("map2.txt");
    //        CPose3D poseMap2(-5.657933, 1.118298, 0, 2.858798, 0, 0);
    //        map1.insertAnotherMap(&map2,poseMap2);
    //        map1.save2D_to_text_file("map3.txt");

    CSimplePointsMap		m1,m2;

    float					runningTime;
    CICP::TReturnInfo		info;
    CICP					ICP;

    m1.load2D_from_text_file(simpleMapFile);

    if ( initialGuessGICP )
    {
        CObservation2DRangeScanPtr obs2Daux = obs2D;
        obs2Daux.make_unique();

        obs2Daux->setSensorPose(initialPose);

        m2.insertObservation( obs2Daux.pointer() );

        CVectorDouble xs, ys, zs, xs2, ys2, zs2;
        m1.getAllPoints(xs,ys,zs);
        m2.getAllPoints(xs2,ys2,zs2);

        PointCloud<PointXYZ>::Ptr	cloud_old   (new PointCloud<PointXYZ>());
        PointCloud<PointXYZ>::Ptr	cloud_new   (new PointCloud<PointXYZ>());
        PointCloud<PointXYZ>::Ptr   cloud_trans (new PointCloud<PointXYZ>());

        cloud_old->clear();
        cloud_new->clear();
        cloud_trans->clear();

        for ( size_t i = 0; i < xs.size(); i+= ( ( accumulatePast ) ? 2 : 1) )
            cloud_old->push_back(PointXYZ(xs[i],ys[i],zs[i]));

        for ( size_t i = 0; i < xs2.size(); i+= ( ( accumulatePast ) ? 1 : 1) )
            cloud_new->push_back(PointXYZ(xs2[i],ys2[i],zs2[i]));

        GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> gicp;

        gicp.setInputSource(cloud_new);
        gicp.setInputTarget(cloud_old);

        gicp.setMaxCorrespondenceDistance (0.2); // 0.5
        gicp.setMaximumIterations (20); // 10
        gicp.setTransformationEpsilon (1e-5); // 1e-5
        gicp.setRotationEpsilon (1e-5); // 1e-5

        cout << "Doing GICP...";

        gicp.align(*cloud_trans);

        double score;
        score = gicp.getFitnessScore(); // Returns the squared average error between the aligned input and target

        cout << " done! Average error: " << sqrt(score) << " meters" << endl;
        goodness = sqrt(score);

        // Obtain the transformation that aligned cloud_source to cloud_source_registered
        Eigen::Matrix4f transformation = gicp.getFinalTransformation();

        CPose3D estimated_pose;

        CMatrixDouble33 rot_matrix;
        for (unsigned int i=0; i<3; i++)
            for (unsigned int j=0; j<3; j++)
                rot_matrix(i,j) = transformation(i,j);

        estimated_pose.setRotationMatrix(rot_matrix);
        estimated_pose.x(transformation(0,3));
        estimated_pose.y(transformation(1,3));
        estimated_pose.z(transformation(2,3));

        CVectorDouble coords;
        estimated_pose.getAsVector( coords );

        CPose2D estimated_poseAux;

        estimated_poseAux.x( coords[0] );
        estimated_poseAux.y( coords[1] );
        estimated_poseAux.phi( coords[3] );

        initialPose = estimated_poseAux + initialPose;

        cout << "Estimation          : " << estimated_pose << endl;
        cout << "Estimated pose in 2D: " << estimated_poseAux << endl;
        cout << "Final pose          : " << initialPose << endl;

        m2.clear();
        obs2Daux->setSensorPose(initialPose);

        m2.insertObservationPtr(obs2Daux);

        if (!skip_window && visualize2DResults)
        {
            // Reference map:
            vector<float>   map1_xs, map1_ys, map1_zs;
            m1.getAllPoints(map1_xs,map1_ys,map1_zs);
            win->plot( map1_xs, map1_ys, "b.3", "map1");

            // Translated map:
            vector<float>   map2_xs, map2_ys, map2_zs;
            m2.getAllPoints(map2_xs,map2_ys,map2_zs);
            win->plot( map2_xs, map2_ys, "r.3", "map2");
            win->axis(-1,10,-6,6);
            win->axis_equal();

            /*cout << "Close the window to exit" << endl;
            win->waitForKey();*/
            mrpt::system::sleep(0);
        }
    }
    else
    {
        m2.insertObservation( obs2D.pointer() );

        // -----------------------------------------------------
        //	ICP.options.ICP_algorithm = icpLevenbergMarquardt;
        //	ICP.options.ICP_algorithm = icpClassic;
        ICP.options.ICP_algorithm = (TICPAlgorithm)ICP_method;

        ICP.options.maxIterations			= 800;
        ICP.options.thresholdAng			= DEG2RAD(10.0f);
        ICP.options.thresholdDist			= 0.75f;
        ICP.options.ALFA					= 0.99f;
        ICP.options.smallestThresholdDist	= 0.05f;
        ICP.options.doRANSAC = false;

        //    ICP.options.dumpToConsole();
        // -----------------------------------------------------

        CPosePDFPtr pdf = ICP.Align(
                    &m1,
                    &m2,
                    initialPose,
                    &runningTime,
                    (void*)&info);

        printf("    ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n    -> ",
               runningTime*1000,
               info.nIterations,
               runningTime*1000.0f/info.nIterations,
               info.goodness*100 );

        goodness = info.goodness*100;

        cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;
        initialPose = pdf->getMeanVal();

        CPosePDFGaussian  gPdf;
        gPdf.copyFrom(*pdf);

        CSimplePointsMap m2_trans = m2;
        m2_trans.changeCoordinatesReference( gPdf.mean );

        if (!skip_window && visualize2DResults)
        {
            CMatrixFloat COV22 =  CMatrixFloat( CMatrixDouble( gPdf.cov ));
            COV22.setSize(2,2);
            Eigen::Vector2f MEAN2D(2);
            MEAN2D(0) = gPdf.mean.x();
            MEAN2D(1) = gPdf.mean.y();

            // Reference map:
            vector<float>   map1_xs, map1_ys, map1_zs;
            m1.getAllPoints(map1_xs,map1_ys,map1_zs);
            win->plot( map1_xs, map1_ys, "b.3", "map1");

            // Translated map:
            vector<float>   map2_xs, map2_ys, map2_zs;
            m2_trans.getAllPoints(map2_xs,map2_ys,map2_zs);
            win->plot( map2_xs, map2_ys, "r.3", "map2");

            // Uncertainty
            win->plotEllipse(MEAN2D(0),MEAN2D(1),COV22,3.0,"b2", "cov");

            win->axis(-1,10,-6,6);
            win->axis_equal();

            /*cout << "Close the window to exit" << endl;
            win->waitForKey();*/
            mrpt::system::sleep(0);
        }
    }

    trajectoryFile << initialPose << endl;

}


//-----------------------------------------------------------
//
//                 processPending3DRangeScans
//
//-----------------------------------------------------------

void processPending3DRangeScans()
{
    size_t N_robotPoses = v_robotPoses.size();

    // More than one hokuyo observation?
    if ( N_robotPoses > 1 )
    {
        TRobotPose &rp1 = v_robotPoses[N_robotPoses-2];
        TRobotPose &rp2 = v_robotPoses[N_robotPoses-1];

        double tdPositions = timeDifference(rp1.time,rp2.time);

        // Okey, set pose of all the pending scans
        for ( size_t obs_index = 0;
              obs_index < v_pending3DRangeScans.size();
              obs_index++ )
        {
            CPose3D pose;
            v_pending3DRangeScans[obs_index]->getSensorPose(pose);

            TTimeStamp &obsTime = v_pending3DRangeScans[obs_index]->timestamp;

            double tdPos1Obs = timeDifference(rp1.time,obsTime);

            // Get an approximation of where was gathered the 3D range scan
            double interpolationFactor = ( tdPos1Obs / tdPositions );

            CPose2D posPoseDif = CPose2D(rp2.pose) - CPose2D(rp1.pose);

            CVectorDouble v_coords;
            posPoseDif.getAsVector(v_coords);
            CPose2D intermediatePose(v_coords[0]*interpolationFactor,
                                     v_coords[1]*interpolationFactor,
                                     v_coords[2]*interpolationFactor);

            CVectorDouble v_coordsIntermediatePose;
            intermediatePose.getAsVector( v_coordsIntermediatePose );

            CVectorDouble v_rp1Pose;
            CPose2D(rp1.pose).getAsVector( v_rp1Pose );

            CPose2D poseToSum = CPose2D(rp1.pose) + intermediatePose;

            CPose3D finalPose = poseToSum + pose;

            CVectorDouble coords;
            finalPose.getAsVector(coords);
            //finalPose.setFromValues(coords[0],coords[1],coords[2],coords[3],coords[4]-DEG2RAD(90),coords[5]-DEG2RAD(90));
            finalPose.setFromValues(coords[0],coords[1],coords[2],coords[3],coords[4],coords[5]);

            /*cout << "Obs initial pose:       " << pose << endl;
            cout << "interpolationFactor:    " << interpolationFactor << endl;
            cout << "rp1.pose:               " << rp1.pose << endl;
            cout << "rp2.pose:               " << rp2.pose << endl;
            cout << "intermediatePose:       " << intermediatePose << endl;
            cout << "poseToSum:              " << poseToSum << endl;
            cout << "finalPose:              " << finalPose << endl;*/

            v_pending3DRangeScans[obs_index]->setSensorPose( finalPose );

            T3DRangeScan Obs3D;
            Obs3D.obs = v_pending3DRangeScans[obs_index];
            v_3DRangeScans.push_back( Obs3D );
        }
    }

    v_pending3DRangeScans.clear();

}


//-----------------------------------------------------------
//
//                    manuallyFixAlign
//
//-----------------------------------------------------------

void manuallyFixAlign( vector<T3DRangeScan> &v_obs,
                       vector<T3DRangeScan> &v_obs2,
                       CPose3D &estimated_pose )
{
    cout << "  [INFO] Manually fixing. Initial pose transf: " << estimated_pose << endl;

    TPose3D pose;

    double OFFSET = 0.1;
    double OFFSET_ANGLES = 0.1;

    // GL points
    mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
    gl_points->setPointSize(2);

    // Coloured point cloud
    CColouredPointsMap colouredMap;
    colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
    colouredMap.insertionOptions.minDistBetweenLaserPoints = 0.01;

    for ( size_t obs_index = 0; obs_index < v_obs2.size(); obs_index++ )
    {
        CDisplayWindow3D windowManuallyFix("GICP-3D: manually aligning scans",500,500);

        COpenGLScenePtr sceneToAlign=COpenGLScene::Create();

        sceneToAlign = windowManuallyFix.get3DSceneAndLock();

        opengl::CGridPlaneXYPtr plane1=CGridPlaneXY::Create(-20,20,-20,20,0,1);
        plane1->setColor(0.3,0.3,0.3);

        sceneToAlign->insert(plane1);

        // Show in Window

        windowManuallyFix.setCameraElevationDeg(15);
        windowManuallyFix.setCameraAzimuthDeg(90);
        windowManuallyFix.setCameraZoom(15);

        for ( size_t i = 0; i < v_obs.size(); i++ )
        {
            CObservation3DRangeScanPtr obs3D = v_obs[i].obs;
            mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
            gl_points->setPointSize(2);
            //gl_points->setColorA(0.5);

            CColouredPointsMap colouredMap;
            colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
            colouredMap.insertionOptions.minDistBetweenLaserPoints = 0.01;
            colouredMap.loadFromRangeScan( *obs3D );

            gl_points->loadFromPointsMap( &colouredMap );

            sceneToAlign->insert( gl_points );
        }

        CObservation3DRangeScanPtr obs3D = v_obs2[obs_index].obs;

        cout << "    Initial pose:                              " << obs3D->sensorPose << endl;

        //Frustum
        float fovh = M_PI*62.5/180.0;	//Larger FOV because depth is registered with color
        float fovv = M_PI*48.5/180.0;

        opengl::CFrustumPtr FOV = opengl::CFrustum::Create(0.3, 2, 57.3*fovh, 57.3*fovv, 1.f, true, false);
        FOV->setColor(0.7,0.7,0.7);
        FOV->setPose(obs3D->sensorPose);
        sceneToAlign->insert( FOV );

        if ( !processInBlock )
            colouredMap.loadFromRangeScan( *obs3D );
        else
        {
            CColouredPointsMap colouredMapAux;
            colouredMapAux.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
            colouredMapAux.insertionOptions.minDistBetweenLaserPoints = 0.01;
            colouredMapAux.loadFromRangeScan( *obs3D );

            colouredMap.addFrom(colouredMapAux);
        }

        gl_points->loadFromPointsMap( &colouredMap );
        gl_points->setPose(estimated_pose);
        pose = gl_points->getPose();
        //cout << "working with pose" << pose << " when set was: " << estimated_pose << endl;
        sceneToAlign->insert( gl_points );

        windowManuallyFix.unlockAccess3DScene();
        windowManuallyFix.forceRepaint();

        if ( ( !processInBlock ) || ( obs_index == v_obs2.size()-1 ) )
        {
            sceneToAlign = windowManuallyFix.get3DSceneAndLock();
            windowManuallyFix.addTextMessage(0.02,0.06+0.03*0, "[Rot]   'Ins': +yaw 'Del': -yaw 'Home': +pitch 'End': -pitch 'Pag-up': +roll 'Pag-down': -roll", TColorf(1,1,1),10,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
            windowManuallyFix.addTextMessage(0.02,0.06+0.03*1, "[Moves] 'up': +x 'down': -x 'left': +y 'right': -y '1': +z '0': -z", TColorf(1,1,1),11,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
            windowManuallyFix.addTextMessage(0.02,0.06+0.03*2, "[Offsets]  's': reduce 'b': increment 'r': reset", TColorf(1,1,1),12,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
            windowManuallyFix.unlockAccess3DScene();

            while ( windowManuallyFix.isOpen() )
            {

                if ( windowManuallyFix.keyHit() )
                {
                    sceneToAlign = windowManuallyFix.get3DSceneAndLock();

                    int key = windowManuallyFix.getPushedKey();

                    vector<double> v_pose;
                    CVectorDouble v_pose2;

                    switch ( key )
                    {

                    // Control the "amount" of size and angles increment/decrement
                    case ( 'r' ):
                    {
                        OFFSET = 0.02;
                        OFFSET_ANGLES = 0.02;

                        break;
                    }
                    case ( 'b' ):
                    {
                        OFFSET = OFFSET*5;
                        OFFSET_ANGLES = OFFSET_ANGLES*5;

                        break;
                    }
                    case ( 's' ):
                    {
                        OFFSET = OFFSET*0.1;
                        OFFSET_ANGLES = OFFSET_ANGLES*0.1;

                        break;
                    }

                        // SIZE

                    case ( MRPTK_LEFT ) : // y axis left
                    {
                        CPose3D move(0,OFFSET,0,0,0,0);
                        gl_points->setPose(move+pose);

                        break;
                    }
                    case ( MRPTK_DOWN ) : // x axis down
                    {
                        CPose3D move(-OFFSET,0,0,0,0,0);
                        gl_points->setPose(move+pose);

                        break;
                    }
                    case ( MRPTK_RIGHT ) : // y axis right
                    {
                        CPose3D move(0,-OFFSET,0,0,0,0);
                        gl_points->setPose(move+pose);

                        break;
                    }
                    case ( MRPTK_UP ) : // x axis up
                    {
                        CPose3D move(OFFSET,0,0,0,0,0);
                        gl_points->setPose(move+pose);

                        break;
                    }
                    case ('1') : // z axis up
                    {
                        CPose3D move(0,0,OFFSET,0,0,0);
                        gl_points->setPose(move+pose);

                        break;
                    }
                    case ('0') : // z axis down255
                    {
                        CPose3D move(0,0,-OFFSET,0,0,0);
                        gl_points->setPose(move+pose);

                        break;
                    }
                    case ( MRPTK_INSERT ) : // yaw
                    {
                        pose.getAsVector(v_pose);
                        CPose3D move(0,0,0,OFFSET_ANGLES,0,0);
                        CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                        CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                        gl_points->setPose(poseFinal);

                        break;
                    }
                    case ( MRPTK_DELETE ) : // -yaw
                    {
                        pose.getAsVector(v_pose);
                        CPose3D move(0,0,0,-OFFSET_ANGLES,0,0);
                        CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                        CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                        gl_points->setPose(poseFinal);
                        break;
                    }
                    case ( MRPTK_HOME ) : // pitch
                    {
                        CPose3D move(0,0,0,0,-OFFSET_ANGLES,0);
                        pose.getAsVector(v_pose);
                        CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                        CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                        gl_points->setPose(poseFinal);

                        break;
                    }
                    case ( MRPTK_END ) : // -pitch
                    {
                        CPose3D move(0,0,0,0,OFFSET_ANGLES,0);
                        pose.getAsVector(v_pose);
                        CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                        CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                        gl_points->setPose(poseFinal);

                        break;
                    }
                    case ( MRPTK_PAGEUP ) : // roll
                    {
                        CPose3D move(0,0,0,0,0,-OFFSET_ANGLES);
                        pose.getAsVector(v_pose);
                        CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                        CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                        gl_points->setPose(poseFinal);

                        break;
                    }
                    case ( MRPTK_PAGEDOWN ) : // -roll
                    {
                        CPose3D move(0,0,0,0,0,OFFSET_ANGLES);
                        pose.getAsVector(v_pose);
                        CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                        CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                        gl_points->setPose(poseFinal);

                        break;
                    }

                    case ('R') : // RESET
                    {
                        CPose3D move(0,0,0,0,0,0);
                        gl_points->setPose(move);

                        break;
                    }

                    default:
                        break;
                    }

                    pose = gl_points->getPose();

                    windowManuallyFix.unlockAccess3DScene();
                    windowManuallyFix.repaint();
                }
            }

            //windowManuallyFix.waitForKey();
        }
    }

    estimated_pose = pose;

    cout << "  [INFO] Final pose: " << estimated_pose << endl;
}


//-----------------------------------------------------------
//
//              preparePointCloudsForRefinement
//
//-----------------------------------------------------------

void preparePointCloudsForRefinement(vector<T3DRangeScan> &v_obs,
                                     vector<T3DRangeScan> &v_obs2,
                                     PointCloud<PointXYZ>::Ptr cloud_old,
                                     PointCloud<PointXYZ>::Ptr	cloud_new)
{

    if (!initialGuessICP2D && !initialGuessDifodo)
    {
        for (size_t i=0; i < v_obs2.size(); i++)
        {
            CPose3D pose;
            v_obs[v_obs.size()-v_obs2.size()+i].obs->getSensorPose(pose);
            v_obs2[i].obs->setSensorPose(pose);
        }
    }

    // Show the scanned points:
    CSimplePointsMap	M1,M2;
    //M1.insertionOptions.minDistBetweenLaserPoints = 0.01;
    //M2.insertionOptions.minDistBetweenLaserPoints = 0.01;

    CTicTac clock;
    clock.Tic();

    // Insert observations into a points map

    for ( size_t i = 0; i < v_obs.size(); i++ )
    {

        if ( !useOverlappingObs )
        {
            // Memory decimation?
            if ( !decimateMemory  || (i < RGBD_sensors.size()*3) || !( i%decimateMemory ) )
                M1.insertObservationPtr( v_obs[i].obs );
        }
        else
        {
            // Now check if this convex hull overlaps with the convex hull of the
            // new observations

            size_t j = 0;
            bool insert = false;
            while ( !insert && ( j < v_obs2.size() ) )
            {
                // Check overlapping
                pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());

                // Point cloud new with old
                pcl::CropHull<pcl::PointXYZ> cropHull;
                cropHull.setInputCloud( v_obs2[j].convexHullCloud );
                cropHull.setHullIndices( v_obs[i].polygons);
                cropHull.setHullCloud( v_obs[i].convexHullCloud);
                cropHull.setDim(3);

                std::vector<int> indices;
                cropHull.filter(indices);
                cropHull.filter(*outputCloud);

                if ( !outputCloud->size() ) // Exists overlap?
                {
                    // Point cloud old with new. This is needed becouse we are
                    // checking only a few points on the point clouds boundaries
                    cropHull.setInputCloud( v_obs[i].convexHullCloud );
                    cropHull.setHullIndices( v_obs2[j].polygons );
                    cropHull.setHullCloud( v_obs2[j].convexHullCloud );
                    cropHull.setDim(3);

                    cropHull.filter(indices);
                    cropHull.filter(*outputCloud);

                    if ( outputCloud->size() )
                        insert = true;
                }
                else
                    insert = true;

                j++;
            }

            if ( insert )
                M1.insertObservationPtr( v_obs[i].obs );
        }

    }

    for ( size_t i = 0; i < v_obs2.size(); i++ )
        M2.insertObservationPtr( v_obs2[i].obs );

    cout << "    Time spent inserting points: " << clock.Tac() << " s." << endl;

    cout << "    Getting points... points 1: ";

    CVectorDouble xs, ys, zs, xs2, ys2, zs2;
    M1.getAllPoints(xs,ys,zs);
    M2.getAllPoints(xs2,ys2,zs2);

    cout << xs.size() << " points 2: " << xs2.size() << " ... done" << endl;

    //cout << "Inserting points...";

    for ( size_t i = 0; i < xs.size(); i+= ( ( accumulatePast ) ? 2 : 1) )
        cloud_old->push_back(PointXYZ(xs[i],ys[i],zs[i]));

    for ( size_t i = 0; i < xs2.size(); i+= ( ( accumulatePast ) ? 1 : 1) )
        cloud_new->push_back(PointXYZ(xs2[i],ys2[i],zs2[i]));
}

//-----------------------------------------------------------
//
//                   getRGBDSensorIndex
//
//-----------------------------------------------------------

size_t getRGBDSensorIndex( const string sensorLabel )
{
    for ( size_t i_sensor = 0; i_sensor < RGBD_sensors.size(); i_sensor++ )
        if ( sensorLabel == RGBD_sensors[i_sensor] )
            return i_sensor;
}


//-----------------------------------------------------------
//
//                   getRGBDSensorIndex
//
//-----------------------------------------------------------

void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
           pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );

  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud ( cloud );

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  normalEstimator.setKSearch ( 15 );
  normalEstimator.compute ( *normals );

  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}

//-----------------------------------------------------------
//
//                   refineLocationGICP
//
//-----------------------------------------------------------

void refineLocationGICP( vector<T3DRangeScan> &v_obs,
                         vector<T3DRangeScan> &v_obs2,
                         CPose3D              &correction)
{
    PointCloud<PointXYZ>::Ptr cloud_old   (new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_new   (new PointCloud<PointXYZ>());

    preparePointCloudsForRefinement(v_obs,v_obs2,cloud_old,cloud_new);

    // Check if the cloud has points (crashes if so)
    if ( cloud_new->points.size() < 100 )
        return;

    CTicTac clock;
    PointCloud<PointXYZ>::Ptr   cloud_trans (new PointCloud<PointXYZ>());

    GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> gicp;

    //cout << "Setting input clouds...";

    gicp.setInputSource(cloud_new);
    gicp.setInputTarget(cloud_old);

    //cout << "done"  << endl;
    //cout << "Setting parameters...";

    //ICP options
    gicp.setMaxCorrespondenceDistance (0.15); // 0.5
    // Set the maximum number of iterations (criterion 1)
    gicp.setMaximumIterations (35); // 10
    // Set the transformation tras epsilon (criterion 2)
    gicp.setTransformationEpsilon (1e-5); // 1e-5
    // Set the transformation rot epsilon (criterion 3)
    gicp.setRotationEpsilon (1e-5); // 1e-5

    //cout << "done"  << endl;

    cout << "    Doing GICP...";

    clock.Tic();
    gicp.align(*cloud_trans);

    double score;
    score = gicp.getFitnessScore(); // Returns the squared average error between the aligned input and target
    bool converged = gicp.hasConverged();

    v_refinementGoodness.push_back(sqrt(score));

    cout << " done! Converged: " << converged << " Average error: " << sqrt(score) << " meters" <<
            " time spent: " << clock.Tac() << " s." << endl;

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = gicp.getFinalTransformation();

    CMatrixDouble33 rot_matrix;
    for (unsigned int i=0; i<3; i++)
        for (unsigned int j=0; j<3; j++)
            rot_matrix(i,j) = transformation(i,j);

    correction.setRotationMatrix(rot_matrix);
    correction.x(transformation(0,3));
    correction.y(transformation(1,3));
    correction.z(transformation(2,3));

    if ( ( sqrt(score) > scoreThreshold && manuallyFix )
            || ( !converged && manuallyFix ) )
        manuallyFixAlign( v_obs, v_obs2, correction );

    if ( processBySensor )
        cout << correction;
}


//-----------------------------------------------------------
//
//                   refineLocationGICPWN
//
//-----------------------------------------------------------

void refineLocationICPWN( vector<T3DRangeScan> &v_obs,
                         vector<T3DRangeScan> &v_obs2,
                         CPose3D              &correction)
{
    PointCloud<PointXYZ>::Ptr cloud_old   (new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_new   (new PointCloud<PointXYZ>());

    preparePointCloudsForRefinement(v_obs,v_obs2,cloud_old,cloud_new);

    // Check if the cloud has points (crashes if so)
    if ( cloud_new->points.size() < 100 )
        return;

    CTicTac clock;
    PointCloud<PointXYZ>::Ptr   cloud_trans (new PointCloud<PointXYZ>());

    // prepare could with normals
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_new_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_old_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_trans_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );

    addNormal( cloud_new, cloud_new_normals );
    addNormal( cloud_old, cloud_old_normals );
    addNormal( cloud_trans, cloud_trans_normals );

    IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icpwn;

    //cout << "Setting input clouds...";

    icpwn.setInputSource(cloud_new_normals);
    icpwn.setInputTarget(cloud_old_normals);

    //cout << "done"  << endl;
    //cout << "Setting parameters...";

    //ICP options
    icpwn.setMaxCorrespondenceDistance (0.1); // 0.5
    // Set the maximum number of iterations (criterion 1)
    icpwn.setMaximumIterations (35); // 10
    // Set the transformation tras epsilon (criterion 2)
    icpwn.setTransformationEpsilon (1e-5); // 1e-5

    //cout << "done"  << endl;

    cout << "    Doing GICP...";

    clock.Tic();
    icpwn.align(*cloud_trans_normals);

    double score;
    score = icpwn.getFitnessScore(); // Returns the squared average error between the aligned input and target
    bool converged = icpwn.hasConverged();

    v_refinementGoodness.push_back(sqrt(score));

    cout << " done! Converged: " << converged << " Average error: " << sqrt(score) << " meters" <<
            " time spent: " << clock.Tac() << " s." << endl;

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icpwn.getFinalTransformation();

    CMatrixDouble33 rot_matrix;
    for (unsigned int i=0; i<3; i++)
        for (unsigned int j=0; j<3; j++)
            rot_matrix(i,j) = transformation(i,j);

    correction.setRotationMatrix(rot_matrix);
    correction.x(transformation(0,3));
    correction.y(transformation(1,3));
    correction.z(transformation(2,3));

    if (( sqrt(score) > scoreThreshold && manuallyFix )
            || ( !converged && manuallyFix ) )
        manuallyFixAlign( v_obs, v_obs2, correction );

    if ( processBySensor )
        cout << correction;
}


//-----------------------------------------------------------
//
//                   refineLocationICPNL
//
//-----------------------------------------------------------

void refineLocationICPNL( vector<T3DRangeScan> &v_obs,
                          vector<T3DRangeScan> &v_obs2,
                          CPose3D              &correction)
{
    PointCloud<PointXYZ>::Ptr cloud_old   (new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_new   (new PointCloud<PointXYZ>());

    preparePointCloudsForRefinement(v_obs,v_obs2,cloud_old,cloud_new);

    // Check if the cloud has points (crashes if so)
    if ( cloud_new->points.size() < 100 )
        return;

    CTicTac clock;
    PointCloud<PointXYZ>::Ptr   cloud_trans (new PointCloud<PointXYZ>());

    IterativeClosestPointNonLinear<PointXYZ, PointXYZ> icpnl;

    //cout << "Setting input clouds...";

    icpnl.setInputSource(cloud_new);
    icpnl.setInputTarget(cloud_old);

    //cout << "done"  << endl;
    //cout << "Setting parameters...";

    //ICP options
    icpnl.setMaxCorrespondenceDistance (0.1); // 0.5
    // Set the maximum number of iterations (criterion 1)
    icpnl.setMaximumIterations (20); // 10
    // Set the transformation tras epsilon (criterion 2)
    icpnl.setTransformationEpsilon (1e-5); // 1e-5


    //cout << "done"  << endl;

    cout << "    Doing ICPNL...";

    clock.Tic();
    icpnl.align(*cloud_trans);

    double score;
    score = icpnl.getFitnessScore(); // Returns the squared average error between the aligned input and target

    v_refinementGoodness.push_back(sqrt(score));

    cout << " done! Average error: " << sqrt(score) << " meters" <<
            " time spent: " << clock.Tac() << " s." << endl;

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icpnl.getFinalTransformation();

    CMatrixDouble33 rot_matrix;
    for (unsigned int i=0; i<3; i++)
        for (unsigned int j=0; j<3; j++)
            rot_matrix(i,j) = transformation(i,j);

    correction.setRotationMatrix(rot_matrix);
    correction.x(transformation(0,3));
    correction.y(transformation(1,3));
    correction.z(transformation(2,3));

    if ( sqrt(score) > scoreThreshold && manuallyFix )
        manuallyFixAlign( v_obs, v_obs2, correction );

    if ( processBySensor )
        cout << correction;
}


//-----------------------------------------------------------
//
//                   refineLocationNDT
//
//-----------------------------------------------------------

void refineLocationNDT( vector<T3DRangeScan> &v_obs,
                        vector<T3DRangeScan> &v_obs2,
                        CPose3D              &correction)
{
    PointCloud<PointXYZ>::Ptr cloud_old   (new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_new   (new PointCloud<PointXYZ>());

    preparePointCloudsForRefinement(v_obs,v_obs2,cloud_old,cloud_new);

    // Check if the cloud has points (crashes if so)
    if ( cloud_new->points.size() < 100 )
        return;

    CTicTac clock;
    PointCloud<PointXYZ>::Ptr   cloud_trans (new PointCloud<PointXYZ>());

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (0.2);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);

    // Setting point cloud to be aligned.
    ndt.setInputSource (cloud_new);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (cloud_old);

    cout << "    Doing NDT...";

    clock.Tic();
    // Calculating required rigid transform to align the input cloud to the target cloud.
    ndt.align (*cloud_trans);

    float score = ndt.getFitnessScore ();
    v_refinementGoodness.push_back(sqrt(score));
    bool converged = ndt.hasConverged();

    std::cout << "    done! Normal Distributions Transform has converged:" << converged
              << "    score: " << sqrt(score) << " time spent: " << clock.Tac() << endl;

    if (!ndt.hasConverged())
    {
        cout << "Not converged! " << endl;
        mrpt::system::pause();
    }

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = ndt.getFinalTransformation ();

    CMatrixDouble33 rot_matrix;
    for (unsigned int i=0; i<3; i++)
        for (unsigned int j=0; j<3; j++)
            rot_matrix(i,j) = transformation(i,j);

    correction.setRotationMatrix(rot_matrix);
    correction.x(transformation(0,3));
    correction.y(transformation(1,3));
    correction.z(transformation(2,3));

    if (( sqrt(score) > scoreThreshold && manuallyFix )
            || ( !converged && manuallyFix ) )
        manuallyFixAlign( v_obs, v_obs2, correction );

    if ( processBySensor )
        cout << correction;
}


//-----------------------------------------------------------
//
//                      refineLocationICP
//
//-----------------------------------------------------------

void refineLocationICP( vector<T3DRangeScan> &v_obs,
                        vector<T3DRangeScan> &v_obs2,
                        CPose3D              &correction)
{
    PointCloud<PointXYZ>::Ptr cloud_old   (new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_new   (new PointCloud<PointXYZ>());

    preparePointCloudsForRefinement(v_obs,v_obs2,cloud_old,cloud_new);

    // Check if the cloud has points (crashes if so)
    if ( cloud_new->points.size() < 100 )
        return;

    CTicTac clock;
    PointCloud<PointXYZ>::Ptr   cloud_trans (new PointCloud<PointXYZ>());

    // Initializing Normal Distributions Transform (NDT).
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (35);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-5);

    // Setting point cloud to be aligned.
    icp.setInputSource (cloud_new);
    // Setting point cloud to be aligned to.
    icp.setInputTarget (cloud_old);

    cout << "    Doing ICP...";

    clock.Tic();
    // Calculating required rigid transform to align the input cloud to the target cloud.
    icp.align (*cloud_trans);

    float score = icp.getFitnessScore ();
    v_refinementGoodness.push_back(sqrt(score));

    std::cout << "    done! converged:" << icp.hasConverged ()
              << "    score: " << sqrt(score) << " time spent: " << clock.Tac() << endl;

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();

    CMatrixDouble33 rot_matrix;
    for (unsigned int i=0; i<3; i++)
        for (unsigned int j=0; j<3; j++)
            rot_matrix(i,j) = transformation(i,j);

    correction.setRotationMatrix(rot_matrix);
    correction.x(transformation(0,3));
    correction.y(transformation(1,3));
    correction.z(transformation(2,3));

    if (( sqrt(score) > scoreThreshold && manuallyFix )
            || ( !icp.hasConverged() && manuallyFix ) )
        manuallyFixAlign( v_obs, v_obs2, correction );

    if ( processBySensor )
        cout << correction;
}


//-----------------------------------------------------------
//
//                  refineLocationICPMRPT
//
//-----------------------------------------------------------

void refineLocationICPMRPT( vector<T3DRangeScan> &v_obs,
                        vector<T3DRangeScan> &v_obs2,
                        CPose3D correction )
{

    if (!initialGuessICP2D && !initialGuessDifodo)
    {
        for (size_t i=0; i < v_obs2.size(); i++)
        {
            CPose3D pose;
            v_obs[v_obs.size()-v_obs2.size()+i].obs->getSensorPose(pose);
            v_obs2[i].obs->setSensorPose(pose);
        }
    }

    // Show the scanned points:
    CSimplePointsMap	M1,M2;

    cout << "Getting points, points 1: ";

    for ( size_t i = 0; i < v_obs.size(); i++ )
    {
        // Memory decimation?
        if ( !decimateMemory  || (i < RGBD_sensors.size()*3) || !( i%decimateMemory ) )
            M1.insertObservationPtr( v_obs[i].obs );
    }

    for ( size_t i = 0; i < v_obs2.size(); i++ )
        M2.insertObservationPtr( v_obs2[i].obs );

    cout << M1.size() << ", points 2: " << M2.size() << " done" << endl;

    COpenGLScenePtr scene1=COpenGLScene::Create();
    COpenGLScenePtr scene2=COpenGLScene::Create();
    COpenGLScenePtr scene3=COpenGLScene::Create();

    opengl::CGridPlaneXYPtr plane1=CGridPlaneXY::Create(-20,20,-20,20,0,1);
    plane1->setColor(0.3,0.3,0.3);
    scene1->insert(plane1);
    scene2->insert(plane1);
    scene3->insert(plane1);

    CSetOfObjectsPtr  PTNS1 = CSetOfObjects::Create();
    CSetOfObjectsPtr  PTNS2 = CSetOfObjects::Create();

    CPointsMap::COLOR_3DSCENE_R = 1;
    CPointsMap::COLOR_3DSCENE_G = 0;
    CPointsMap::COLOR_3DSCENE_B = 0;
    M1.getAs3DObject(PTNS1);

    CPointsMap::COLOR_3DSCENE_R = 0;
    CPointsMap::COLOR_3DSCENE_G = 0;
    CPointsMap::COLOR_3DSCENE_B = 1;
    M2.getAs3DObject(PTNS2);

    scene2->insert( PTNS1 );
    scene2->insert( PTNS2 );

    // --------------------------------------
    // Do the ICP-3D
    // --------------------------------------
    float run_time;
    CICP	icp;
    CICP::TReturnInfo	icp_info;

    icp.options.thresholdDist = 0.40;
    icp.options.thresholdAng = 0;

    CPose3DPDFPtr pdf= icp.Align3D(
                &M2,          // Map to align
                &M1,          // Reference map
                CPose3D() ,   // Initial gross estimate
                &run_time,
                &icp_info);

    CPose3D  mean = pdf->getMeanVal();

    cout << "    ICP run took " << run_time << " secs." << endl;
    cout << "    Goodness: " << 100*icp_info.goodness << "% , # of iterations= " << icp_info.nIterations << endl;
    cout << "    ICP output: mean= " << mean << endl;

    // Aligned maps:
    CSetOfObjectsPtr  PTNS2_ALIGN = CSetOfObjects::Create();

    M2.changeCoordinatesReference( CPose3D()-mean );
    M2.getAs3DObject(PTNS2_ALIGN);

    scene3->insert( PTNS1 );
    scene3->insert( PTNS2_ALIGN );

    // Show in Windows:

    window2->get3DSceneAndLock()=scene2;
    window2->unlockAccess3DScene();

    window3->get3DSceneAndLock()=scene3;
    window3->unlockAccess3DScene();


    mrpt::system::sleep(20);
    window2->forceRepaint();

    window2->setCameraElevationDeg(15);
    window2->setCameraAzimuthDeg(90);
    window2->setCameraZoom(15);

    window3->setCameraElevationDeg(15);
    window3->setCameraAzimuthDeg(90);
    window3->setCameraZoom(15);

    //cout << "Press any key to exit..." << endl;
    //window->waitForKey();

    double score = 100*icp_info.goodness;
    v_refinementGoodness.push_back( score );

    correction = mean;

    if ( sqrt(score) > scoreThreshold && manuallyFix )
        manuallyFixAlign( v_obs, v_obs2, correction );

}


//-----------------------------------------------------------
//
//                       refine
//
//-----------------------------------------------------------

void refine()
{

    size_t N_sensors = RGBD_sensors.size();
    size_t N_scans = v_3DRangeScans.size();

    if ( !v_3DRangeScans.size() )
        return;

    bool obsHasPoints3D = v_3DRangeScans[0].obs->hasPoints3D;

    if ( processBySensor )
    {
        cout << "  -------------------------------------------------" << endl;
        cout << "       Getting relative position among devices " << refinationMethod << endl;
        cout << "  -------------------------------------------------" << endl;

        vector< vector <T3DRangeScan> > v_allObs(N_sensors);  // Past set of obs

        for ( size_t obsIndex = 0; obsIndex < N_scans; obsIndex++ )
        {
            T3DRangeScan obs = v_3DRangeScans[obsIndex];
            size_t sensorIndex = getRGBDSensorIndex(obs.obs->sensorLabel);

            if ( !obsHasPoints3D )
                obs.obs->project3DPointsFromDepthImage();

            v_allObs[sensorIndex].push_back(obs);
        }

        for ( size_t device_index = 1; device_index < N_sensors; device_index++ )
        {
            cout << "Relative transformation from device " << device_index;
            cout << " to 0"<< endl;

            CPose3D correction;

            refineLocationGICP( v_allObs[0], v_allObs[device_index], correction );

            for ( size_t i = 0; i < v_allObs[device_index].size(); i++ )
                v_allObs[device_index][i].obs->sensorPose =
                        correction + v_allObs[device_index][i].obs->sensorPose;

            cout << " <-- this"<< endl;
        }
    }
    else
    {

        cout << "  -------------------------------------------------" << endl;
        cout << "          Refining sensor poses using " << refinationMethod << endl;
        cout << "  -------------------------------------------------" << endl;


        vector<T3DRangeScan> v_obs;  // Past set of obs
        vector<T3DRangeScan> v_obsC(N_sensors); // Current set of obs
        vector<size_t> v_obsC_indices(N_sensors); // Indices in v_3DRangeScans vector of the current set of obs
        vector<size_t> v_obsToDelete; // If using key poses, not key obs to delete
        vector<bool> v_obs_loaded(N_sensors,false); // Track the sensors with an obs loaded

        bool first = true;
        size_t set_index = 0;

        CTicTac clockEllapsedICP3D;
        clockEllapsedICP3D.Tic();

        for ( size_t obsIndex = 0; obsIndex < N_scans; obsIndex++ )
        {
            //cout << "200: " << v_3DRangeScans[200].obs->sensorPose << endl;

            CTicTac clockLoop;
            clockLoop.Tic();

            T3DRangeScan &obs = v_3DRangeScans[obsIndex];

            size_t sensorIndex = getRGBDSensorIndex(obs.obs->sensorLabel);

            if ( !obsHasPoints3D )
                obs.obs->project3DPointsFromDepthImage();

            v_obsC[sensorIndex]       = obs;
            v_obs_loaded[sensorIndex] = true;
            v_obsC_indices[sensorIndex] = obsIndex;

            //cout << "Sensor index " << sensorIndex;
            //cout << " included " << obs.obs->sensorLabel << endl;

            double sum = 0;

            for ( size_t i_sensor = 0; i_sensor < N_sensors; i_sensor++ )
                sum += v_obs_loaded[i_sensor];

            if ( sum == N_sensors )
            {
                v_obs_loaded.clear();
                v_obs_loaded.resize(N_sensors,false);

                cout << "    Working set of obs index... " << set_index++ <<
                        " of approx. " << (double)N_scans/N_sensors << endl;

                if ( first )
                {
                    v_obs = v_obsC;
                    first = false;

                    if ( manuallyFix )
                    {
                        // TODO: Load this from config to do it more general
                        // Sensors order:
                        vector<string> v_RGBDs_order(4);
                        v_RGBDs_order[0] = "RGBD_4";
                        v_RGBDs_order[1] = "RGBD_3";
                        v_RGBDs_order[2] = "RGBD_1";
                        v_RGBDs_order[3] = "RGBD_2";

                        vector<T3DRangeScan> v_aligned;

                        v_aligned.push_back(v_obs[getRGBDSensorIndex(v_RGBDs_order[0])]);

                        TTimeStamp t1 = v_obs[0].obs->timestamp;

                        for ( size_t i = 1; i < N_sensors; i++ )
                        {
                            size_t i_sensor = getRGBDSensorIndex(v_RGBDs_order[i]);
                            cout << "    Time diff: " << timeDifference(t1,
                                     v_obs[getRGBDSensorIndex(v_RGBDs_order[i_sensor])].obs->timestamp)
                                 << endl;

                            CPose3D estimated_pose;
                            //v_obs[i_sensor].obs->getSensorPose(estimated_pose);

                            vector<T3DRangeScan> v_toAligne;
                            v_toAligne.push_back(v_obs[i_sensor]);
                            manuallyFixAlign( v_aligned, v_toAligne, estimated_pose );

                            CPose3D pose;
                            v_obs[i_sensor].obs->getSensorPose( pose );

                            CPose3D finalPose = estimated_pose + pose;
                            v_obs[i_sensor].obs->setSensorPose(finalPose);

                            cout << "    Pose of " << v_RGBDs_order[i] <<
                                    " " << v_obs[i_sensor].obs->sensorPose << endl;

                            v_aligned.push_back( v_obs[i_sensor] );

                            // Propagate the correction to the remaining obs to process
                            if ( propagateCorrections )
                            {
                                for ( size_t i_obs = obsIndex+1; i_obs < N_scans; i_obs++ )
                                {
                                    if ( v_3DRangeScans[i_obs].obs->sensorLabel ==
                                         v_RGBDs_order[i_sensor] )
                                    {
                                        v_3DRangeScans[i_obs].obs->sensorPose =
                                                estimated_pose + v_3DRangeScans[i_obs].obs->sensorPose;
                                    }
                                }
                            }
                        }
                    }

                    continue;
                }

                // Check if it's a key pose
                if ( useKeyPoses )
                {
                    bool keyPose = true;

                    vector<CPose3D> v_poses, v_posesC;

                    size_t N_prevObs = v_obs.size();

                    for ( int i_sensor = N_sensors-1; i_sensor >= 0; i_sensor-- )
                        v_obs[N_prevObs-i_sensor].obs->getSensorPose(v_poses[i_sensor]);

                    for ( size_t i_sensor = 0; i_sensor < N_sensors; i_sensor++ )
                        v_obsC[i_sensor].obs->getSensorPose(v_posesC[i_sensor]);

                    for ( size_t i_sensor = 0; i_sensor < N_sensors; i_sensor++ )
                        keyPose = keyPose && isKeyPose(v_posesC[i_sensor],v_poses[i_sensor]);

                    if (!keyPose)
                    {
                        cout << "    Not a key pose, moving to the next one." << endl;
                        cout << "    ---------------------------------------------------" << endl;

                        for ( size_t i_sensor = 0; i_sensor < N_sensors; i_sensor++ )
                            v_obsToDelete.push_back(v_obsC_indices[i_sensor]);

                        continue;
                    }
                }

                // Build vectors with individual observations
                vector< vector<T3DRangeScan> > v_isolatedObs(N_sensors);
                for ( size_t i_sensor = 0; i_sensor < N_sensors; i_sensor++ )
                {
                    v_isolatedObs[i_sensor]= vector<T3DRangeScan>(1,v_obsC[i_sensor]);
                    //cout << i_sensor << "  " << v_isolatedObs[i_sensor][0].obs->sensorPose << endl;
                }


                if ( processInBlock )
                {
                    CPose3D correction;

                    if ( refinationMethod == "GICP" )
                        refineLocationGICP( v_obs, v_obsC,correction );
                    else if ( refinationMethod == "ICP")
                        refineLocationICP( v_obs, v_obsC,correction );
                    else if ( refinationMethod == "ICPNL")
                        refineLocationICPNL( v_obs, v_obsC,correction );
                    else if ( refinationMethod == "ICPWN")
                        refineLocationICPWN( v_obs, v_obsC,correction );
                    else if ( refinationMethod == "NDT")
                        refineLocationNDT( v_obs, v_obsC,correction );

                    for ( size_t i = 0; i < v_obsC.size(); i++ )
                        v_obsC[i].obs->sensorPose =
                                correction + v_obsC[i].obs->sensorPose;

                    // Propagate the correction to the remaining obs to process
                    if ( propagateCorrections )
                    {
                        #pragma omp parallel for
                        for ( size_t i_obs = obsIndex+1; i_obs < N_scans; i_obs++ )
                            v_3DRangeScans[i_obs].obs->sensorPose =
                                        correction + v_3DRangeScans[i_obs].obs->sensorPose;
                    }
                }
                else
                {
                    #pragma omp parallel for
                    for ( size_t i_sensor = 0; i_sensor < N_sensors; i_sensor++ )
                    {
                        CPose3D correction;

                        if ( refinationMethod == "GICP" )
                            refineLocationGICP( v_obs, v_isolatedObs[i_sensor],correction );
                        else if ( refinationMethod == "ICP")
                            refineLocationICP( v_obs, v_isolatedObs[i_sensor],correction );
                        else if ( refinationMethod == "ICPNL")
                            refineLocationICPNL( v_obs, v_isolatedObs[i_sensor],correction );
                        else if ( refinationMethod == "ICPWN")
                            refineLocationICPWN( v_obs, v_obsC,correction );
                        else if ( refinationMethod == "NDT")
                            refineLocationNDT( v_obs, v_isolatedObs[i_sensor],correction );

                        // Compose correction with initial guess
                        CObservation3DRangeScanPtr obs = v_isolatedObs[i_sensor][0].obs;

                        CPose3D pose;
                        obs->getSensorPose( pose );

                        CPose3D finalPose = correction + pose;
                        obs->setSensorPose(finalPose);

                        // Propagate the correction to the remaining obs to process
                        if ( propagateCorrections )
                        {
                            for ( size_t i_obs = obsIndex+1; i_obs < N_scans; i_obs++ )
                            {
                                if ( v_3DRangeScans[i_obs].obs->sensorLabel ==
                                     RGBD_sensors[i_sensor] )
                                {
                                    v_3DRangeScans[i_obs].obs->sensorPose =
                                            correction + v_3DRangeScans[i_obs].obs->sensorPose;
                                }
                            }
                        }

                    }
                }



                if ( accumulatePast )
                    v_obs.insert(v_obs.end(), v_obsC.begin(), v_obsC.end() );
                else
                    v_obs = v_obsC;

                // Time Statistics

                cout << "    Time ellapsed      : " <<
                        size_t(clockLoop.Tac()  / SECS_PER_MIN) << " min. " <<
                        size_t(clockLoop.Tac()) % SECS_PER_MIN << " s." << endl;

                cout << "    Total time ellapsed: " <<
                        size_t(clockEllapsedICP3D.Tac()  / SECS_PER_MIN) << " min. " <<
                        size_t(clockEllapsedICP3D.Tac()) % SECS_PER_MIN  << " s." << endl;

                cout << "    ---------------------------------------------------" << endl;
            }
        }
    }

    if ( !obsHasPoints3D )
    {
        for ( size_t obsIndex = 0; obsIndex < N_scans; obsIndex++ )
        {
            v_3DRangeScans[obsIndex].obs->points3D_x.clear();
            v_3DRangeScans[obsIndex].obs->points3D_y.clear();
            v_3DRangeScans[obsIndex].obs->points3D_z.clear();

            v_3DRangeScans[obsIndex].obs->hasPoints3D = false;
        }
    }

    cout << "  [INFO] Goodness mean : "
         << std::accumulate(v_refinementGoodness.begin(), v_refinementGoodness.end(), 0.0 ) / v_refinementGoodness.size()
         << endl
         << "               maximum : " << *max_element( v_refinementGoodness.begin(), v_refinementGoodness.end() )
         << endl
         << "               minimum : " << *min_element( v_refinementGoodness.begin(), v_refinementGoodness.end() )
         << endl;


}


//-----------------------------------------------------------
//
//                   computeInitialGuessDifodo
//
//-----------------------------------------------------------

void computeInitialGuessDifodo()
{
    cout << endl;
    cout << "  -------------------------------------------------" << endl;
    cout << "         Computing initial poses with Difodo" << endl;
    cout << "  -------------------------------------------------" << endl;

    CDifodoDatasets odo;

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;

    // Difodo configuration parameters
    vector<CPose3D> v_poses;
    vector<unsigned int> v_cameras_order;

    unsigned int rows;
    unsigned int cols;

    //
    // Get difodo configuration parameters according to the current dataset

    bool exit = false, first = true;

    while ( !exit && CRawlog::getActionObservationPairOrObservation(i_rawlog,
                                            action,observations,obs,obsIndex) )
    {
        // 3D range scan observation
        if ( IS_CLASS(obs, CObservation3DRangeScan) )
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            string label = obs3D->sensorLabel;

            if ( find(RGBD_sensors.begin(), RGBD_sensors.end(), label)
                 == RGBD_sensors.end() )
            {
                RGBD_sensors.push_back(label);
                unsigned int cam_index = atoi((label.substr(label.size()-1,label.size())).c_str());
                v_cameras_order.push_back(cam_index);
                v_poses.push_back(obs3D->sensorPose);
            }

            if ( RGBD_sensors.size() == 4 )
                exit = true;

            if (first)
            {
                rows = obs3D->rangeImage.rows();
                cols = obs3D->rangeImage.cols();
                first = false;
            }
        }
    }

    cout << "  [INFO] Cameras order: ";
    for ( size_t i = 0; i < v_cameras_order.size(); i++ )
            cout << v_cameras_order[i] << " ";
    cout << endl;

    //
    // Load configuration

    odo.loadConfiguration(rows,cols,v_poses,i_rawlogFileName,
                          v_cameras_order,difodoVisualization);

    //
    // Main operation

    odo.initializeScene();

    int pushed_key = 0;
    bool working = 0, stop = 0;

    //Necessary step before starting
    odo.reset();

    vector<float> v_difodoRunTimes;

    while(!odo.dataset_finished)
    {
        odo.loadFrame();
        odo.odometryCalculation();

        cout << endl << "    Difodo runtime(ms): " << odo.execution_time;
        v_difodoRunTimes.push_back(odo.execution_time);

        odo.updateScene();

        vector<mrpt::obs::CObservation3DRangeScanPtr> &v_obs = odo.v_processedObs;

        for ( size_t i = 0; i < v_obs.size(); i++ )
        {
            v_obs[i]->setSensorPose(odo.global_pose+odo.cam_pose[i]);
            T3DRangeScan obs3D;
            obs3D.obs = v_obs[i];
            v_3DRangeScans.push_back(obs3D);
            //o_rawlog << v_obs[i];
        }
    }

    float sum = accumulate(v_difodoRunTimes.begin(),v_difodoRunTimes.end(),0.0);
    float avg = sum / (float)v_difodoRunTimes.size();
    cout << endl << "  [INFO] Difodo average run time: " << avg/1000 << " sec. " << endl;
}

//-----------------------------------------------------------
//
//                   computeInitialGuessICP2D
//
//-----------------------------------------------------------

void computeInitialGuessICP2D()
{
    cout << endl;
    cout << "  -------------------------------------------------" << endl;
    cout << "         Computing initial poses with ICP2D" << endl;
    cout << "  -------------------------------------------------" << endl;

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;
    vector<int> RGBDobsPerSensor;

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,
                                                           action,observations,obs,obsIndex) )
    {
        // 2D laser observation
        if ( IS_CLASS(obs, CObservation2DRangeScan) )
        {
            CObservation2DRangeScanPtr obs2D = CObservation2DRangeScanPtr(obs);
            obs2D->load();
            double goodness;

            trajectoryICP2D(simpleMapFile,obs2D,goodness);

            TRobotPose robotPose;
            robotPose.pose = initialPose;
            robotPose.time = obs2D->timestamp;
            robotPose.goodness = goodness;

            if ( ( !initialGuessGICP && ( goodness > 80 ) )
                 || ( initialGuessGICP && goodness < 1 ) )
                v_robotPoses.push_back( robotPose );
            else
                v_pending3DRangeScans.clear();

            // Process pending 3D range scans, if any
            if ( !v_pending3DRangeScans.empty() )
                processPending3DRangeScans();
        }
        // 3D range scan observation
        else if ( IS_CLASS(obs, CObservation3DRangeScan) )
        {
            string &label = obs->sensorLabel;

            if ( find(RGBD_sensors.begin(), RGBD_sensors.end(), label)
                 == RGBD_sensors.end() )
            {
                RGBD_sensors.push_back(label);
                RGBDobsPerSensor.push_back(0);

                //For synchornization with the other possible devices
                for ( int i = 0; i < RGBDobsPerSensor.size()-1; i++ )
                    RGBDobsPerSensor[i] = 0;
            }

            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            // Check decimation and insert the observation
            if ( !(RGBDobsPerSensor[getRGBDSensorIndex(label)] % decimation) )
                v_pending3DRangeScans.push_back( obs3D );

            RGBDobsPerSensor[getRGBDSensorIndex(label)]++;
        }
        else
            continue;
    }

    trajectoryFile.close();
}


//-----------------------------------------------------------
//
//                 computeInitialGuessOnlyFill
//
//-----------------------------------------------------------

void computeInitialGuessOnlyFill()
{
    cout << endl;
    cout << "  -------------------------------------------------" << endl;
    cout << "              Filling initial poses" << endl;
    cout << "  -------------------------------------------------" << endl;

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;
    vector<int> RGBDobsPerSensor;

    while ( CRawlog::getActionObservationPairOrObservation(i_rawlog,
                                                           action,observations,obs,obsIndex) )
    {
        if ( IS_CLASS(obs, CObservation3DRangeScan) )
        {
            string &label = obs->sensorLabel;

            // New sensor?
            if ( find(RGBD_sensors.begin(), RGBD_sensors.end(), label)
                 == RGBD_sensors.end() )
            {
                RGBD_sensors.push_back(label);
                RGBDobsPerSensor.push_back(0);

                //For synchornization with the other possible devices
                for ( int i = 0; i < RGBDobsPerSensor.size()-1; i++ )
                    RGBDobsPerSensor[i] = 0;
            }

            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            T3DRangeScan obs;
            obs.obs = obs3D;

            // Check decimation and insert the observation
            if ( !(RGBDobsPerSensor[getRGBDSensorIndex(label)] % decimation) )
                v_3DRangeScans.push_back( obs );

            RGBDobsPerSensor[getRGBDSensorIndex(label)]++;
        }
    }
}


//-----------------------------------------------------------
//
//                          main
//
//-----------------------------------------------------------

int main(int argc, char **argv)
{
    cout << endl << "-----------------------------------------------------" << endl;
    cout <<         "                  Mapping app.                    " << endl;
    cout <<         "            [Object Labeling Tookit]                 " << endl;
    cout <<         "-----------------------------------------------------" << endl << endl;

    try
    {
        CTicTac clock;

        //
        // Load parameters

        int res = loadParameters(argc,argv);

        if ( res < 0 ) // error while loading parameteres
            return -1;
        else if ( !res ) // Just shown the help
            return 0;

        mrpt::system::sleep(4000); // Just to visualize the parameters loaded

        //
        //  Check the input rawlog file

        if (!mrpt::system::fileExists(i_rawlogFileName))
            cout << "  [ERROR] Couldn't open rawlog dataset file " <<
                    i_rawlogFileName << endl;

        cout << "  [INFO] Working with " << i_rawlogFileName << endl;

        i_rawlog.open(i_rawlogFileName);
        o_rawlog.open(o_rawlogFileName);

        //
        // Create the reference objects:

        if ( refineLocalization && refinationMethod == "ICPMRPTrefineLocationICPMRPT" )
        {
            window = CDisplayWindow3DPtr(new CDisplayWindow3D("ICP-3D: scene",500,500));
            window2 = CDisplayWindow3DPtr(new CDisplayWindow3D("ICP-3D: UNALIGNED scans",500,500));
            window3 = CDisplayWindow3DPtr(new CDisplayWindow3D("ICP-3D: ICP-ALIGNED scans",500,500));
        }

        //
        // Compute initial guess

        if ( initialGuessICP2D )
        {
            clock.Tic();
            computeInitialGuessICP2D();
            time_measures.icp2D = clock.Tac();

        }
        else if ( initialGuessDifodo )
        {
            clock.Tic();
            computeInitialGuessDifodo();
            time_measures.difodo = clock.Tac();
        }
        else // If not using ICP2D initial guess, just fill the vector of 3D obs
        {
            computeInitialGuessOnlyFill();
        }

        cout << "  [INFO] Number of RGBD observations to work with: ";
        cout << v_3DRangeScans.size() << endl;

        //
        // Smooth3DObs?

        if ( smooth3DObs )
        {
            clock.Tic();
            smoothObss(); // TODO: This colud be leveraged by DIFODO, refactor it for that
            time_measures.smoothing = clock.Tac();
        }

        //
        // Overlaping?

        if ( useOverlappingObs )
        {
            clock.Tic();
            computeConvexHulls();
            time_measures.overlapping = clock.Tac();

        }

        //
        // Refine localization

        if ( refineLocalization )
        {
            clock.Tic();
            refine();
            time_measures.refine = clock.Tac();
        }

        //
        // Prompt performance measurements

        showPerformanceMeasurements();

        //
        // Save processed observations to file

        cout << "  [INFO] Saving obs to rawlog file " << o_rawlogFileName << " ...";

        cout.flush();

        for ( size_t obs_index = 0; obs_index < v_3DRangeScans.size(); obs_index++ )
        {
            // Restore point cloud
            if ( smooth3DObs )
                v_3DRangeScans[obs_index].obs->project3DPointsFromDepthImage();

            CSensoryFrame SF;
            SF.insert( v_3DRangeScans[obs_index].obs );

            o_rawlog << v_3DRangeScans[obs_index].obs;
        }

        o_rawlog.close();

        cout << " completed." << endl;

        //
        // Visualize 2D results

        if ( visualize2DResults )
        {
            visualizeResults();
        }

        cout << " done! See you soon!" << endl;
        return 0;

    } catch (exception &e)
    {
        cout << "OLT exception caught: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        printf("Another exception!!");
        return -1;
    }
}
