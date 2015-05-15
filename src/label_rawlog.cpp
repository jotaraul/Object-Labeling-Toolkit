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

struct TLabelledBox
{
    // Loaded from .scene file
    CBoxPtr box;   // object containing the corners of the object's bounding box
    string  label; // e.g. scourer, bowl, or scourer_1, bowl_3 if working with instances
    // Computed
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHullCloud;
    vector<pcl::Vertices>   polygons;

    TLabelledBox() : convexHullCloud(new pcl::PointCloud<pcl::PointXYZ>())
    {}
};

struct TConfiguration
{
    bool    visualizeLabels;// Enable visualization of individual observations
    string  rawlogFile;     // Rawlog file to be labeled
    string  labelledScene;  // Scene already labeled by "Label_scene"
    bool    instancesLabeled; // Are we working with instances? e.g. knife_1
};

//
//  Global variables
//

TConfiguration          configuration;

vector<TLabelledBox>    v_labelled_boxes;
map<string,TPoint3D>    m_consideredLabels; // Map <label,color>
vector<string>          v_appearingLabels;

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> pclWindow;

//-----------------------------------------------------------
//
//                      loadConfig
//
//-----------------------------------------------------------

void loadConfig( string const configFile )
{
    CConfigFile config( configFile );

    // Load general configuration

    configuration.visualizeLabels = config.read_bool("GENERAL","visualizeLabels",0,true);
    configuration.rawlogFile      = config.read_string("GENERAL","rawlogFile","",true);
    configuration.labelledScene   = config.read_string("GENERAL","labelledScene","",true);

    // Load object labels (classes) to be considered

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
        m_consideredLabels[v_labelNames[i_label]] = v_colors[i_label];

    cout << "[INFO] Loaded labels: " << endl;

    map<string,TPoint3D>::iterator it;

    for ( it = m_consideredLabels.begin(); it != m_consideredLabels.end(); it++ )
        cout << " - " << it->first << ", with color " << it->second << endl;

    cout << "[INFO] Configuration successfully loaded." << endl;

}


//-----------------------------------------------------------
//
//                    loadLabelledScene
//
//-----------------------------------------------------------

void  loadLabelledScene()
{
    mrpt::opengl::COpenGLScenePtr   labelledScene;

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

                if ( !m_consideredLabels.count(labelled_box.label) )
                    cout << "[CAUTION] label " << labelled_box.label << " does not appear in the label list." << endl;

                // Check if the label has been already inserted
                if ( find(v_appearingLabels.begin(),
                          v_appearingLabels.end(),
                          labelled_box.label) == v_appearingLabels.end() )
                    v_appearingLabels.push_back(labelled_box.label);
            }

            boxes_inserted++;
        }

        cout << "[INFO] Label clusters on, " << v_labelled_boxes.size() <<  " labelled boxes loaded." << endl;

    }
    else
        cout << "[ERROR] While loading the labelled scene file." << endl;

}

//-----------------------------------------------------------
//
//                    getInstanceLabel
//
//-----------------------------------------------------------

string getInstanceLabel(const string &instaceLabel )
{
    map<string,TPoint3D>::iterator it;

    for ( it = m_consideredLabels.begin(); it != m_consideredLabels.end(); it++ )
        if ( it->first.find(instaceLabel)!=std::string::npos )
            return it->first;

    return "";
}

//-----------------------------------------------------------
//
//                      labelObs
//
//-----------------------------------------------------------

void labelObs(CObservation3DRangeScanPtr obs,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    map<string,TPoint3D>::iterator it;

    size_t N_boxes = v_labelled_boxes.size();

    vector<vector<int > > v_indices(N_boxes);

    pclWindow viewer;

    if ( configuration.visualizeLabels )
    {
        viewer = pclWindow(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();
    }

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

        //cout << "Size of indices: " << v_indices[box_index].size() << endl;

        //
        // Give color to the point cloud excerpt
        //

        if ( configuration.visualizeLabels )
        {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredOutputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

            TPoint3D color;

            if ( configuration.instancesLabeled )
            {
                string label = getInstanceLabel(box.label);
                if ( label.size() )
                    color = m_consideredLabels[label];
                else
                    color = TPoint3D(1,1,1);
            }
            else
                color = m_consideredLabels[box.label];

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

    }

    // Visualize labeling results
    if ( configuration.visualizeLabels )
    {
        viewer->resetStoppedFlag();
        while (!viewer->wasStopped())
            viewer->spinOnce(100);
    }
}

//-----------------------------------------------------------
//
//                        main
//
//-----------------------------------------------------------

int main(int argc, char* argv[])
{
    // Useful variables

    vector<string> sensors_to_use;
    CRawlog rawlog;
    bool stepByStepExecution = false;

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

    cout << "[INFO] Rawlog file   : " << configuration.rawlogFile << " " << rawlog.size() << " obs" << endl;
    cout << "[INFO] Labeled scene : " << configuration.labelledScene << endl;
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

        //
        // Label observation
        //

        cout << "[INFO] Labeling observation." << endl;

        labelObs( obs3D, pcl_cloud );

    }

    mrpt::system::pause();

    return 0;
}
