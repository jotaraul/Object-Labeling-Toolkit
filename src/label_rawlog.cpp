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
#include <mrpt/maps/CColouredPointsMap.h>

#include <mrpt/math.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/threads.h>
#include <mrpt/opengl.h>
#include <mrpt/utils/CConfigFile.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mrpt/maps/PCL_adapters.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

#include <pcl/filters/voxel_grid.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt;
using namespace std;

//
//  Data types
//

struct TSegmentedRegion
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointIndices       indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHullCloud;
    vector<pcl::Vertices>   polygons;
    string                  label;
    size_t                  track_id;
    TPoint3D                color;
    mrpt::opengl::CBoxPtr   box;

    TSegmentedRegion() : convexHullCloud(new pcl::PointCloud<pcl::PointXYZ>()),
        box( CBox::Create() )
    {}
};

struct TLabelledBox
{
    CBoxPtr                 box;
    string                  label;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHullCloud;
    vector<pcl::Vertices>   polygons;

    TLabelledBox() : convexHullCloud(new pcl::PointCloud<pcl::PointXYZ>())
    {}
};

struct TConfiguration
{
    bool      visualizeLabels;
    string    rawlogFile;
    string    labelledScene;
};

//
//  Global variables
//

mrpt::gui::CDisplayWindow3D     win3D;
mrpt::opengl::COpenGLScenePtr   labelledScene;

TConfiguration                  configuration;

vector<vector<CPose3D> >        v_posesPerSensor;
vector<TLabelledBox>            v_labelled_boxes;
map<string,TPoint3D>            m_labels; // Map <label,pos in the colors vector>


vector<vector< uint8_t > > v_colors;


void  loadLabelledScene();

//-----------------------------------------------------------
//                        loadConfig
//-----------------------------------------------------------

void loadConfig( string const configFile )
{
    CConfigFile config( configFile );

    configuration.visualizeLabels             = config.read_bool("GENERAL","visualizeLabels",0,true);

    configuration.rawlogFile                = config.read_string("GENERAL","rawlogFile","",true);
    configuration.labelledScene             = config.read_string("GENERAL","labelledScene","",true);

    vector<string> v_labelNames;
    config.read_vector("LABELS","labelNames",vector<string>(0),v_labelNames,true);

    size_t magicNumber = ceil(pow(v_labelNames.size(),1.0/3.0));

    cout << "Magic number: " << magicNumber << endl;

    vector<TPoint3D> v_colors;

    for ( double r = 0; r < magicNumber; r+= 1 )
        for ( double g = 0; g < magicNumber; g+= 1 )
            for ( double b = 0; b < magicNumber; b+= 1 )
                v_colors.push_back(TPoint3D(1.0*(1-r/(magicNumber-1)),
                                    1.0*(1-g/(magicNumber-1)),
                                    1.0*(1-b/(magicNumber-1))));


    for ( size_t i_label = 0; i_label < v_labelNames.size(); i_label++ )
            m_labels[v_labelNames[i_label]] = v_colors[i_label];


    cout << "[INFO] Rawlog file   : " << configuration.rawlogFile << endl;
    cout << "[INFO] Labeled scene : " << configuration.labelledScene << endl;

    cout << "[INFO] Loaded labels: " << endl;

    map<string,TPoint3D>::iterator it;

    for ( it = m_labels.begin(); it != m_labels.end(); it++ )
        cout << " - " << it->first << ", with color " << it->second << endl;

    cout << "[INFO] Configuration successfully loaded." << endl;

}


//-----------------------------------------------------------
//                    loadLabelledScene
//-----------------------------------------------------------

void  loadLabelledScene()
{
    labelledScene = mrpt::opengl::COpenGLScene::Create();
    if ( labelledScene->loadFromFile( configuration.labelledScene ) )
    {
        // Load previously inserted boxes
        bool keepLoading = true;
        size_t boxes_inserted = 0;

        while ( keepLoading )
        {
            CBoxPtr box = labelledScene->getByClass<CBox>(boxes_inserted);

            if ( box.null() )
                keepLoading = false;
            else
            {
                TLabelledBox labelled_box;

                labelled_box.box = box;
                labelled_box.label = box->getName();

                TPose3D pose = box->getPose();

                TPoint3D c1,c2;
                box->getBoxCorners(c1,c2);

                TPoint3D C111 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c1.y,c1.z)) );
                TPoint3D C112 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c1.y,c2.z)) );
                TPoint3D C121 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c2.y,c1.z)) );
                TPoint3D C122 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c2.y,c2.z)) );
                TPoint3D C211 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c1.y,c1.z)) );
                TPoint3D C212 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c1.y,c2.z)) );
                TPoint3D C221 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c2.y,c1.z)) );
                TPoint3D C222 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c2.y,c2.z)) );

                pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud ( new pcl::PointCloud<pcl::PointXYZ>());
                pointCloud->push_back( pcl::PointXYZ( C111.x, C111.y, C111.z ));
                pointCloud->push_back( pcl::PointXYZ( C112.x, C112.y, C112.z ));
                pointCloud->push_back( pcl::PointXYZ( C121.x, C121.y, C121.z ));
                pointCloud->push_back( pcl::PointXYZ( C122.x, C122.y, C122.z ));
                pointCloud->push_back( pcl::PointXYZ( C211.x, C211.y, C211.z ));
                pointCloud->push_back( pcl::PointXYZ( C212.x, C212.y, C212.z ));
                pointCloud->push_back( pcl::PointXYZ( C221.x, C221.y, C221.z ));
                pointCloud->push_back( pcl::PointXYZ( C222.x, C222.y, C222.z ));

                pcl::ConvexHull<pcl::PointXYZ> convex_hull;
                convex_hull.setInputCloud(pointCloud);
                convex_hull.setDimension(3);
                convex_hull.reconstruct(*labelled_box.convexHullCloud,
                                        labelled_box.polygons);

                Eigen::Matrix4f transMat;

                transMat(0,0)=0;    transMat(0,1)=-1;     transMat(0,2)=0;    transMat(0,3)=0;
                transMat(1,0)=0;    transMat(1,1)=0;      transMat(1,2)=+1;   transMat(1,3)=0;
                transMat(2,0)=1;    transMat(2,1)=0;      transMat(2,2)=0;    transMat(2,3)=0;
                transMat(3,0)=0;    transMat(3,1)=0;      transMat(3,2)=0;    transMat(3,3)=1;

                pcl::transformPointCloud( *labelled_box.convexHullCloud,
                                          *labelled_box.convexHullCloud,
                                          transMat );

                v_labelled_boxes.push_back( labelled_box );

                if ( !m_labels.count(labelled_box.label) )
                    cout << "[CAUTION] label " << labelled_box.label << "does not appear in the label list." << endl;

            }

            boxes_inserted++;
        }

        cout << "[INFO] Label clusters on, " << v_labelled_boxes.size() <<  " labelled boxes loaded." << endl;

    }
    else
        cout << "[ERROR] While loading the labelled scene file." << endl;

}


void labelObs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    map<string,TPoint3D>::iterator it;

    size_t N_boxes = v_labelled_boxes.size();

    vector<vector<int > > v_indices(N_boxes);

    boost::shared_ptr<pcl::visualization::PCLVisualizer>	viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    for ( size_t box_index = 0; box_index < v_labelled_boxes.size(); box_index++ )
    {

        TLabelledBox &box = v_labelled_boxes[box_index];

        //cout << "Evaluating " << box.label;

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropHull<pcl::PointXYZ> cropHull;
        cropHull.setInputCloud( cloud );
        //cropHull.setIndices( boost::make_shared<const pcl::PointIndices> (indices) );
        cropHull.setHullIndices(box.polygons);
        cropHull.setHullCloud(box.convexHullCloud);
        cropHull.setDim(3);


//        viewer->removeAllPointClouds();
//        viewer->addPointCloud( cloud );
//        viewer->resetStoppedFlag();
//        while (!viewer->wasStopped())
//            viewer->spinOnce(100);

        cropHull.filter(v_indices[box_index]);
        cropHull.filter(*outputCloud);

        cout << "Size of indices: " << v_indices[box_index].size() << endl;

        // Give color to the point cloud excerpt
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredOutputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

//        size_t colorIndex = (v_labels[box.label]*5)%255;
//        uint8_t color_r = v_colors[colorIndex][0];
//        uint8_t color_g = v_colors[colorIndex][1];
//        uint8_t color_b = v_colors[colorIndex][2];

        TPoint3D color = m_labels[box.label];
        uint8_t color_r = color.x*255;
        uint8_t color_g = color.y*255;
        uint8_t color_b = color.z*255;

        for ( size_t point = 0; point < outputCloud->size(); point++ )
        {

            pcl::PointXYZRGB coloredPoint(color_r,color_g,color_b);
            coloredPoint.x = outputCloud->points[point].x;
            coloredPoint.y = outputCloud->points[point].y;
            coloredPoint.z = outputCloud->points[point].z;


            coloredOutputCloud->points.push_back(coloredPoint);
        }

        //viewer->removeAllPointClouds();
        //viewer->addPointCloud( box.convexHullCloud );
        stringstream ss;
        ss << "Outputcloud_" << box_index;
        viewer->addPointCloud( coloredOutputCloud,string(ss.str()) );

    }

    viewer->resetStoppedFlag();
    while (!viewer->wasStopped())
        viewer->spinOnce(100);
}

//-----------------------------------------------------------
//                          main
//-----------------------------------------------------------

int main(int argc, char* argv[])
{
    // Useful variables

    vector<string> sensors_to_use;
    CRawlog rawlog;
    bool stepByStepExecution = false;

    /*for ( size_t mult1 = 150; mult1 <= 255; mult1+= 20 )
        for ( size_t mult2 = 150; mult2 <= 255; mult2+= 20 )
            for ( size_t mult3 = 150; mult3 <= 255; mult3+= 20 )
            {
                vector<uint8_t> color;
                color.push_back(mult1);
                color.push_back(mult2);
                color.push_back(mult3);
                v_colors.push_back( color );
            }*/

    //
    // Set 3D window for visualization purposes
    //

    win3D.setWindowTitle("Rawlog labeling");

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

    win3D.unlockAccess3DScene();

    if ( argc > 1 )
    {
        // Get configuration file name

        cout << "1:" << argv[1] << endl;

        string configFile = argv[1];
        loadConfig(configFile);

        // Get optional paramteres
        if ( argc > 2 )
        {
            bool alreadyReset = false;

            size_t arg = 2;

            while ( arg < argc )
            {
                if ( !strcmp(argv[arg],"-i") )
                {
                    configuration.rawlogFile = argv[arg+1];
                    arg += 2;
                }
                if ( !strcmp(argv[arg],"-l") )
                {
                    configuration.labelledScene = argv[arg+1];
                    arg += 2;
                }
                else if ( !strcmp(argv[arg],"-sensor") )
                {
                    string sensor = argv[arg+1];
                    sensors_to_use.push_back(  sensor );
                    arg += 2;
                }
                else if ( !strcmp(argv[arg], "-step") )
                {
                    stepByStepExecution = true;
                    arg++;
                }
                else
                {
                    cout << "[Error] " << argv[arg] << " unknown paramter" << endl;
                    return -1;
                }

            }
        }
    }
    else
    {
        cout << "Usage information. At least one expected arguments: " << endl <<
                " \t <conf_fil>       : Configuration file." << endl;
        cout << "Then, optional parameters:" << endl <<
                " \t -i <rawlog_file> : Rawlog file to process." << endl <<
                " \t -sensor <sensor_label> : Use obs. from this sensor (none used by default)." << endl <<
                " \t -step                  : Enable step by step execution." << endl;

        return -1;
    }




    rawlog.loadFromRawLogFile( configuration.rawlogFile );

    cout << "Rawlog file   : " << configuration.rawlogFile << " " << rawlog.size() << " obs" << endl;
    cout << "Labeled scene : " << configuration.labelledScene << endl;
    loadLabelledScene();

    // Iterate over the obs into the rawlog and show them labeled in the 3D window

    size_t color_index = 0;

    for ( size_t obs_index = 0; obs_index < rawlog.size(); obs_index++ )
    {
        CObservationPtr obs = rawlog.getAsObservation(obs_index);

        // Check if the sensor is being used
        if ( find(sensors_to_use.begin(), sensors_to_use.end(),obs->sensorLabel) == sensors_to_use.end() )
            continue;

        // Get obs pose
        CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
        obs3D->load();

        CPose3D pose;
        obs3D->getSensorPose( pose );
        cout << "Pose [" << obs_index << "]: " << pose << endl;

        // Create per pixel labeling
        // Label size (0=8 bits, 1=16 bits, 2=32 bits, 3=32 bits, 8=64 bits
        const unsigned int LABEL_SIZE = 2; // 32 bits
        obs3D->pixelLabels =  CObservation3DRangeScan::TPixelLabelInfoPtr( new CObservation3DRangeScan::TPixelLabelInfo< LABEL_SIZE >() );

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud( new pcl::PointCloud<pcl::PointXYZ>() );

        obs3D->project3DPointsFromDepthImageInto( *pcl_cloud, true );

        Eigen::Matrix4f transMat;

        transMat(0,0)=0;    transMat(0,1)=-1;     transMat(0,2)=0;    transMat(0,3)=0;
        transMat(1,0)=0;    transMat(1,1)=0;      transMat(1,2)=+1;   transMat(1,3)=0;
        transMat(2,0)=1;    transMat(2,1)=0;      transMat(2,2)=0;    transMat(2,3)=0;
        transMat(3,0)=0;    transMat(3,1)=0;      transMat(3,2)=0;    transMat(3,3)=1;

        pcl::transformPointCloud( *pcl_cloud, *pcl_cloud, transMat );

        cout << "Number of points in point cloud: " << pcl_cloud->size() << endl;
        pcl_cloud->height = 240;
        pcl_cloud->width = 320;

        // Label observation

        cout << "[INFO] Labeling observation." << endl;

        labelObs( pcl_cloud );


        mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

        mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
        gl_points->setPointSize(6);

        CColouredPointsMap colouredMap;

        gl_points->loadFromPointsMap( &colouredMap );
        scene->insert( gl_points );
        win3D.unlockAccess3DScene();

        scene = win3D.get3DSceneAndLock();

        win3D.unlockAccess3DScene();
        win3D.repaint();

        if ( stepByStepExecution )
            win3D.waitForKey();
    }

    mrpt::system::pause();

    return 0;
}
