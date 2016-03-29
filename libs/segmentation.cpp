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
//#include <mrpt/maps.h>
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
    bool doPlanarSegmentation;
    bool fusePlanes;
    bool doEuclideanSegmentation;
    bool showSegmentation;
    bool showColouredRegions;
    bool labelClusters;
    bool trackClusters;
    string rawlogFile;
    string labelledScene;
};

struct TPlanarSegmentationConfig
{
    size_t	minPlaneInliers;
    double	distThreshold;
    double	angleThreshold;
    double	maximumCurvature;

};

struct TEuclideanSegmentationConfig
{
    double clusterTolerance;
    size_t minClusterSize;
    size_t maxClusterSize;
};

struct TbilateralFilterConfig
{
    float	sigmaS;
    float	sigmaR;
    bool	earlyDivision;
};

struct TLabellingConfig
{
    float minPercentageToLabel;
};

struct TTrackingConfig
{
    float   minPercentageToAssingTrack;
    bool    decimatePointCloud;
    float   decimationLeafSize;
};

struct TFusionConfig
{
    float   minDistance;
    float   maxAngleDiff;
};

typedef Eigen::aligned_allocator< pcl::PlanarRegion<pcl::PointXYZ> > PlanarRegionAllocator;	//!< Typedef for a more readable code.

//
//  Global variables
//

mrpt::gui::CDisplayWindow3D     win3D;
mrpt::opengl::COpenGLScenePtr   labelledScene;

TbilateralFilterConfig          bilateralFilterConfig;
TEuclideanSegmentationConfig    euclideanSegmentationConfig;
TPlanarSegmentationConfig       planarSegmentationConfig;
TLabellingConfig                labellingConfig;
TTrackingConfig                 trackingConfig;
TFusionConfig                   fusionConfig;
TConfiguration                  configuration;

vector<vector<CPose3D> >        v_posesPerSensor;
vector<TLabelledBox>            v_labelled_boxes;
size_t                          trackID = 0;
vector<vector<vector<TSegmentedRegion> > >v_regionsPerSensorAndObs;

void  loadLabelledScene();

//-----------------------------------------------------------
//                        loadConfig
//-----------------------------------------------------------

void loadConfig( string const configFile )
{
    CConfigFile config( configFile );

    configuration.doPlanarSegmentation      = config.read_bool("GENERAL","doPlanarSegmentation",0,true);
    configuration.fusePlanes                = config.read_bool("GENERAL","fusePlanes",0,true);
    configuration.doEuclideanSegmentation   = config.read_bool("GENERAL","doEuclideanSegmentation",0,true);
    configuration.showSegmentation          = config.read_bool("GENERAL","showSegmentation",0,true);
    configuration.showColouredRegions       = config.read_bool("GENERAL","showColouredRegions",0,true);
    configuration.labelClusters             = config.read_bool("GENERAL","labelClusters",0,true);
    configuration.trackClusters             = config.read_bool("GENERAL","trackClusters",0,true);

    configuration.rawlogFile                = config.read_string("GENERAL","rawlogFile","",true);
    configuration.labelledScene             = config.read_string("GENERAL","labelledScene","",true);

    cout << "Rawlog file: " << configuration.rawlogFile << endl;

    // Load labelled boxes?
    if ( configuration.labelClusters )
        loadLabelledScene();

    TPlanarSegmentationConfig &p = planarSegmentationConfig;

    p.minPlaneInliers   = config.read_int("PLANAR_SEGMENTATION","minPlaneInliers",0,true);
    p.distThreshold     = config.read_double("PLANAR_SEGMENTATION","distThreshold",0,true);
    p.angleThreshold    = config.read_double("PLANAR_SEGMENTATION","angleThreshold",0,true);
    p.maximumCurvature  = config.read_double("PLANAR_SEGMENTATION","maximumCurvature",0,true);

    TEuclideanSegmentationConfig &e = euclideanSegmentationConfig;

    e.clusterTolerance  = config.read_double("EUCLIDEAN_SEGMENTATION","clusterTolerance",0,true);
    e.minClusterSize    = config.read_int("EUCLIDEAN_SEGMENTATION","minClusterSize",0,true);
    e.maxClusterSize    = config.read_int("EUCLIDEAN_SEGMENTATION","maxClusterSize",0,true);

    TbilateralFilterConfig &b = bilateralFilterConfig;

    b.sigmaS        = config.read_double("BILATERAL_FILTER","sigmaS",0,true);
    b.sigmaR        = config.read_double("BILATERAL_FILTER","sigmaR",0,true);
    b.earlyDivision = config.read_bool("BILATERAL_FILTER","earlyDivision",0,true);

    TLabellingConfig &l = labellingConfig;

    l.minPercentageToLabel = config.read_double("LABELLING","minPercentageToLabel",0,true);

    TTrackingConfig &t = trackingConfig;

    t.minPercentageToAssingTrack = config.read_double("TRACKING","minPercentageToAssingTrack",0,true);
    t.decimatePointCloud         = config.read_bool("TRACKING","decimatePointCloud",0,true);
    t.decimationLeafSize         = config.read_double("TRACKING","decimationLeafSize",0,true);

    TFusionConfig &f = fusionConfig;

    f.minDistance   = config.read_double("FUSION","minDistance",0,true);
    f.maxAngleDiff  = config.read_double("FUSION","maxAngleDiff",0,true);

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
            }

            boxes_inserted++;
        }

        cout << "[INFO] Label clusters on, " << v_labelled_boxes.size() <<  " labelled boxes loaded." << endl;
    }
    else
        cout << "[ERROR] While loading the labelled scene file." << endl;

}


//-----------------------------------------------------------
//                    poseDifferentEnough
//-----------------------------------------------------------

bool poseDifferentEnough( CPose3D &pose, size_t sensor_index)
{
    size_t N_poses = v_posesPerSensor[sensor_index].size();

    // First pose? If so, it's always okey
    if ( !N_poses )
    {
        v_posesPerSensor[sensor_index].push_back(pose);
        return true;
    }

    CPose3D &lastPose = v_posesPerSensor[sensor_index][N_poses-1];

    CVectorDouble pose_v;
    pose.getAsVector( pose_v );

    CVectorDouble lastPose_v;
    lastPose.getAsVector( lastPose_v );

    float dist = lastPose.distance3DTo(pose_v[0], pose_v[1], pose_v[2]);
    float angle = sqrt( pow(pose_v[3]-lastPose[3],2)
                        + pow(pose_v[4]-lastPose[4],2)
                        + pow(pose_v[5]-lastPose[5],2)
                        );

    if ( dist > 0.2 )
        return true;
    else if ( angle > DEG2RAD((float)10) )
        return true;
    else
        return false;
}


//-----------------------------------------------------------
//                       getTrack
//-----------------------------------------------------------

//int getTrack( const TSegmentedRegion region,
//              const vector<TSegmentedRegion> previous_regions,
//              TPoint3D &color )
int getTrack( const TSegmentedRegion region,
              const vector<vector<TSegmentedRegion> > previous_regions,
              TPoint3D &color )
{
    /*cout << "Regions from the previous obs: ";
    for ( size_t i = 0; i < previous_regions.size(); i++ )
        cout << "ID:" <<previous_regions[i].track_id << " color:" << previous_regions[i].color << " ";

    cout << endl;*/

    double bestPercentage = trackingConfig.minPercentageToAssingTrack;
    int indexBestPercentage = -1;
    int obsBestPercentage = -1;
    size_t N_points = region.indices.indices.size();

    //cout << "N_points " << N_points << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>());

    if ( trackingConfig.decimatePointCloud )
    {
        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (region.cloud);
        sor.setIndices( boost::make_shared<const pcl::PointIndices> (region.indices) );
        sor.setLeafSize (trackingConfig.decimationLeafSize,
                         trackingConfig.decimationLeafSize,
                         trackingConfig.decimationLeafSize);
        sor.filter (*downsampledCloud);

        N_points = downsampledCloud->size();
    }

    //cout << "N_points " << N_points << endl;

    for ( size_t obs_index = 0; obs_index < previous_regions.size(); obs_index ++ )
    {
    for ( size_t region_index = 0; region_index < previous_regions[obs_index].size(); region_index ++ )
    {
        //cout << "Checking with previous region " << region_index;
        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropHull<pcl::PointXYZ> cropHull;

        if ( trackingConfig.decimatePointCloud )
            cropHull.setInputCloud( downsampledCloud );
        else
        {
            cropHull.setInputCloud( region.cloud );
            cropHull.setIndices( boost::make_shared<const pcl::PointIndices> (region.indices) );
        }

        cropHull.setHullIndices(previous_regions[obs_index][region_index].polygons);
        cropHull.setHullCloud(previous_regions[obs_index][region_index].convexHullCloud);
        cropHull.setDim(3);

        std::vector<int> indices;
        cropHull.filter(indices);
        cropHull.filter(*outputCloud);

        double percentage = (float)outputCloud->size() / (float)N_points;
        //cout << " percentage: " << percentage << endl;

        if ( percentage > bestPercentage )
        {
            indexBestPercentage = region_index;
            obsBestPercentage = obs_index;
            bestPercentage = percentage;
        }
    }
    }

    if ( indexBestPercentage != -1 )
    {
        color = previous_regions[obsBestPercentage][indexBestPercentage].color;
        return previous_regions[obsBestPercentage][indexBestPercentage].track_id;
    }
    else
        return -1;
}

//-----------------------------------------------------------
//                      getLabel
//-----------------------------------------------------------

void getLabel( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
               pcl::PointIndices indices,
               string &label )
{
    double bestPercentage = labellingConfig.minPercentageToLabel;
    int bestBoxIndex = -1;
    size_t N_points = indices.indices.size();

    cout << "[INFO] Number of points in the cluster: " << N_points << endl;

    //    boost::shared_ptr<pcl::visualization::PCLVisualizer>	viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //    viewer->initCameraParameters ();

    //    viewer->addPointCloud( cloud );
    //    while (!viewer->wasStopped())
    //        viewer->spinOnce(100);

    for ( size_t box_index = 0; box_index < v_labelled_boxes.size(); box_index++ )
    {

        TLabelledBox &box = v_labelled_boxes[box_index];

        //cout << "Evaluating " << box.label;

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropHull<pcl::PointXYZ> cropHull;
        cropHull.setInputCloud( cloud );
        cropHull.setIndices( boost::make_shared<const pcl::PointIndices> (indices) );
        cropHull.setHullIndices(box.polygons);
        cropHull.setHullCloud(box.convexHullCloud);
        cropHull.setDim(3);

        //        // Extract the planar inliers from the input cloud
        //        pcl::ExtractIndices<pcl::PointXYZ> extract;
        //        extract.setInputCloud (cloud);
        //        extract.setIndices (boost::make_shared<const pcl::PointIndices> (indices) );
        //        extract.setNegative (false);

        //        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud ( new pcl::PointCloud<pcl::PointXYZ>() );
        //        extract.filter (*cluster_cloud);

        //        viewer->removeAllPointClouds();
        //        viewer->addPointCloud( cluster_cloud,  mrpt::utils::format( "cloud_%i",box_index) );
        //        viewer->resetStoppedFlag();
        //        while (!viewer->wasStopped())
        //            viewer->spinOnce(100);

        std::vector<int> indices;
        cropHull.filter(indices);
        cropHull.filter(*outputCloud);

        //        viewer->removeAllPointClouds();
        //        viewer->addPointCloud( box.convexHullCloud );
        //        viewer->addPointCloud( outputCloud,  mrpt::utils::format( "cloud_%i",box_index) );
        //        viewer->resetStoppedFlag();
        //        while (!viewer->wasStopped())
        //            viewer->spinOnce(100);

        double percentage = (float)outputCloud->size() / (float)N_points;
        //cout << " percentage: " << percentage << endl;

        if ( percentage > bestPercentage )
        {
            bestBoxIndex = box_index;
            label = box.label;
            bestPercentage = percentage;
        }

    }
}




//-----------------------------------------------------------
//                      segmentPlanes
//-----------------------------------------------------------

void applyBilateralFilter( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{

    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCopy ( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::copyPointCloud(*cloud,*cloudCopy);

    boost::shared_ptr<pcl::visualization::PCLVisualizer>	viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    viewer->addPointCloud( cloudCopy );

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }*/

    pcl::FastBilateralFilter<pcl::PointXYZ> m_bilateralFilter;

    double filter_start = pcl::getTime ();

    m_bilateralFilter.setInputCloud (cloud);

    m_bilateralFilter.setSigmaS (bilateralFilterConfig.sigmaS);
    m_bilateralFilter.setSigmaR (bilateralFilterConfig.sigmaR);
    //m_bilateralFilter.setEarlyDivision (m_bilateralFilterConf.earlyDivision);

    m_bilateralFilter.filter (*cloud);

    double filter_end = pcl::getTime ();

    cout << "[FILTER] Bilateral Filter took: " << double (filter_end - filter_start) << endl;

    /*boost::shared_ptr<pcl::visualization::PCLVisualizer>	viewer2(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer2->initCameraParameters ();
    viewer2->addPointCloud( cloud );

    while (!viewer2->wasStopped())
    {
        viewer2->spinOnce(100);
    }*/
}


//-----------------------------------------------------------
//                      segmentPlanes
//-----------------------------------------------------------

void segmentPlanes( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    std::vector<pcl::PointIndices> &inlierIndices,
                    vector<pcl::ModelCoefficients> &modelCoefficients,
                    vector<pcl::PointIndices>      &boundaryIndices)
{
    /*boost::shared_ptr<pcl::visualization::PCLVisualizer>	viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->addPointCloud( cloud );

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }*/

    cout << "[SEGMENT_PLANES] Segmenting planes..." << endl;

    // Ensure that the members vbles involved in the process are freshhhhh

    vector<pcl::PlanarRegion<pcl::PointXYZ>, PlanarRegionAllocator> regions;
    vector<pcl::PointIndices>       labelIndices;			//!< Points agrouped in each label.

    pcl::PointCloud<pcl::Normal>::Ptr   normalCloud;			//!< Normals computed of the input point cloud.
    pcl::PointCloud<pcl::Label>::Ptr    labels;				//!< Labels of each plane.

    labels		= pcl::PointCloud<pcl::Label>::Ptr( new pcl::PointCloud<pcl::Label>);
    normalCloud	= pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    cout << "[SEGMENT_PLANES] Computing normals." << endl;

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor (0.02f);
    ne.setNormalSmoothingSize (10.0f);

    double normal_start = pcl::getTime ();

    ne.setInputCloud (cloud);
    ne.compute (*normalCloud);

    double normal_end = pcl::getTime ();

    cout << "[SEGMENT_PLANES] Normal Estimation took: " << double (normal_end - normal_start) << endl;

    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers ( planarSegmentationConfig.minPlaneInliers );
    mps.setAngularThreshold ( planarSegmentationConfig.angleThreshold );
    mps.setDistanceThreshold ( planarSegmentationConfig.distThreshold );
    mps.setMaximumCurvature ( planarSegmentationConfig.maximumCurvature );


    double plane_extract_start = pcl::getTime ();

    mps.setInputCloud (cloud);
    mps.setInputNormals (normalCloud);

    mps.segmentAndRefine (regions, modelCoefficients, inlierIndices, labels,
                          labelIndices, boundaryIndices);


    double plane_extract_end = pcl::getTime ();

    cout << "[SEGMENT_PLANES] Plane extraction took " <<
            double (plane_extract_end - plane_extract_start) << endl;

}


//-----------------------------------------------------------
//                      segmentClusters
//-----------------------------------------------------------

void segmentClusters( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices &m_outlierIndices, std::vector<pcl::PointIndices> &cluster_indices )
{
    cout << "[SEGMENT_CLUSTERS] Segmenting clusters..." << endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);


    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (euclideanSegmentationConfig.clusterTolerance); // 2cm
    ec.setMinClusterSize (euclideanSegmentationConfig.minClusterSize);
    ec.setMaxClusterSize (euclideanSegmentationConfig.maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setIndices( boost::make_shared<const pcl::PointIndices> (m_outlierIndices) );
    ec.setInputCloud (cloud);

    double cluster_extract_start = pcl::getTime ();

    ec.extract (cluster_indices);

    double cluster_extract_end = pcl::getTime ();

    cout << "[SEGMENT_CLUSTERS] Cluster segmentation took " << double (cluster_extract_end - cluster_extract_start) << endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;


    }

}


//-----------------------------------------------------------
//                          planesFusion
//-----------------------------------------------------------

void planesFusion( pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                   std::vector<pcl::PointIndices> &inliers_indices,
                   vector<pcl::ModelCoefficients> &modelCoefficients,
                   vector<pcl::PointIndices>      &boundaryIndices)
{
    cout << "[PLANES_FUSION] Fusing planes..." << endl;

    bool fused = true;

    double fusion_start = pcl::getTime ();

    while ( fused )
    {
        size_t N_planes = inliers_indices.size();
        fused = false;

        // Do we have planes to work?
        if ( !N_planes )
            continue;

        size_t region_index1 = 0;


        while ( ( region_index1 < N_planes-1 ) && !fused )
        {
            size_t region_index2 = region_index1+1;

            while ( (region_index2 < N_planes) && !fused )
            {
                cout << "Checking fusion between " << region_index1 << " and " << region_index2 << endl;
                vector<int> &boundary1 = boundaryIndices[region_index1].indices;
                vector<int> &boundary2 = boundaryIndices[region_index2].indices;

                double minDist = std::numeric_limits<double>::max();

                for ( size_t b1 = 0; b1 < boundary1.size(); b1++ )
                    for ( size_t b2 = 0; b2 < boundary2.size(); b2++ )
                    {
                        double dist = sqrt( pow(pcl_cloud->points[boundary1[b1]].x
                                                - pcl_cloud->points[boundary2[b2]].x,2)
                                            + pow (pcl_cloud->points[boundary1[b1]].y
                                                   - pcl_cloud->points[boundary2[b2]].y,2)
                                            + pow( pcl_cloud->points[boundary1[b1]].z
                                                   - pcl_cloud->points[boundary2[b2]].z,2) );

                        minDist = ( minDist > dist ) ? dist : minDist;
                    }

                //cout << "Min dist: " << minDist << endl;
                if ( minDist < fusionConfig.minDistance )
                {
                    cout << "Model coeffs 1: " << modelCoefficients[region_index1] << endl;
                    cout << "Model coeffs 2: " << modelCoefficients[region_index2] << endl;

                    vector<float> &coeff1 = modelCoefficients[region_index1].values;
                    vector<float> &coeff2 = modelCoefficients[region_index2].values;

                    float scalarProduct = coeff1[0]*coeff2[0]
                                            + coeff1[1]*coeff2[1]
                                            + coeff1[2]*coeff2[2];

                    float angleDiff = acos( scalarProduct );

                    cout << "ANGLE DIF: " << RAD2DEG(angleDiff) << endl;

                    if ( ( angleDiff < DEG2RAD(fusionConfig.maxAngleDiff) ) ||
                           ( angleDiff > 180 - DEG2RAD(fusionConfig.maxAngleDiff) ) )
                    {
                        cout << "FUSE!";
                        vector<int> &indices1 = inliers_indices[region_index1].indices;
                        vector<int> &indices2 = inliers_indices[region_index2].indices;
                        indices1.insert( indices1.begin(), indices2.begin(), indices2.end() );

                        boundary1.insert( boundary1.begin(), boundary2.begin(), boundary2.end() );

                        inliers_indices.erase( inliers_indices.begin()+region_index2);
                        modelCoefficients.erase(modelCoefficients.begin()+region_index2);
                        boundaryIndices.erase(boundaryIndices.begin()+region_index2);

                        fused = true;
                    }

                }

                region_index2++;
            }
            region_index1++;
        }
    }

    double fusion_end = pcl::getTime ();
    cout << "[PLANES_FUSION] Cluster segmentation took " << double (fusion_end - fusion_start) << endl;

}


//-----------------------------------------------------------
//                          main
//-----------------------------------------------------------

int main(int argc, char* argv[])
{
    // Create vector of colors for visualization
    vector<TPoint3D> v_colors;

    for ( double mult1 = 0; mult1 <= 1; mult1+= 0.2 )
        for ( double mult2 = 0; mult2 <= 1; mult2+= 0.2 )
            for ( double mult3 = 0; mult3 <= 1; mult3+= 0.2 )
                v_colors.push_back( TPoint3D(1.0*mult1, 1.0*mult2, 1.0*mult3) );

    // Set 3D window

    vector<string> sensors_to_use;
    sensors_to_use.push_back("RGBD_1");
    sensors_to_use.push_back("RGBD_2");
    sensors_to_use.push_back("RGBD_3");
    sensors_to_use.push_back("RGBD_4");

    win3D.setWindowTitle("Segmentation");

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

    bool stepByStepExecution = false;

    CRawlog rawlog;

    if ( argc > 2 )
    {
        // Get rawlog file name

        cout << "1:" << argv[1] << endl;

        string configFile = argv[1];
        loadConfig(configFile);

        // Get optional paramteres
        if ( argc > 1 )
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
                else if ( !strcmp(argv[arg],"-sensor") )
                {
                    string sensor = argv[arg+1];
                    arg += 2;

                    if ( !alreadyReset )
                    {
                        sensors_to_use.clear();
                        alreadyReset = true;
                    }

                    sensors_to_use.push_back(  sensor );

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
        cout << "Usage information. At least two expected arguments: " << endl <<
                " \t <conf_fil>       : Configuration file." << endl <<
                " \t -i <rawlog_file> : Rawlog file to process." << endl;
        cout << "Then, optional parameters:" << endl <<
                " \t -sensor <sensor_label> : Use obs. from this sensor (all used by default)." << endl <<
                " \t -step                  : Enable step by step execution." << endl;

        return -1;
    }

    rawlog.loadFromRawLogFile( configuration.rawlogFile );

    // Set the number of sensors used into the vector to track the segmented regions
    v_regionsPerSensorAndObs.resize( sensors_to_use.size() );
    v_posesPerSensor.resize(sensors_to_use.size() );

    size_t N_segmented_point_clouds = 0;

    // Iterate over the obs into the rawlog and show them in the 3D window

    size_t color_index = 0;

    for ( size_t obs_index = 0; obs_index < rawlog.size(); obs_index++ )
    {
        CObservationPtr obs = rawlog.getAsObservation(obs_index);

        // Check if the sensor is being used
        if ( find(sensors_to_use.begin(), sensors_to_use.end(),obs->sensorLabel) == sensors_to_use.end() )
            continue;

        // Get sensor index
        size_t sensor_index = find(sensors_to_use.begin(),
                                   sensors_to_use.end(),obs->sensorLabel)
                - sensors_to_use.begin();

        // Get obs pose
        CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
        obs3D->load();

        CPose3D pose;
        obs3D->getSensorPose( pose );
        cout << "Pose [" << obs_index << "]: " << pose << endl;

        // Check if the new pose is different enough from the previous one
        if ( !poseDifferentEnough(pose,sensor_index) )
            continue;

        v_regionsPerSensorAndObs[sensor_index].push_back( vector< TSegmentedRegion >() );
        size_t obsRealIndex = v_regionsPerSensorAndObs[sensor_index].size()-1;

        mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

        mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
        gl_points->setPointSize(6);

        CColouredPointsMap colouredMap;
        colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
        colouredMap.loadFromRangeScan( *obs3D );

        gl_points->loadFromPointsMap( &colouredMap );
        //scene->insert( gl_points );
        win3D.unlockAccess3DScene();

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud( new pcl::PointCloud<pcl::PointXYZ>() );

        //
        // Do segmentation of planes or arbitrary regions?
        //

        if ( configuration.doEuclideanSegmentation ||
             configuration.doPlanarSegmentation )
        {

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

            applyBilateralFilter(pcl_cloud);

            std::vector<pcl::PointIndices> inliers_indices;
            vector<bool> v_indices_to_remove(obs3D->points3D_x.size(),false);

            //
            // Do segmentation of planes
            //

            if ( configuration.doPlanarSegmentation )
            {

                vector<pcl::ModelCoefficients>  modelCoefficients; // Planes Coefficients of
                vector<pcl::PointIndices>       boundaryIndices;   // Boundary indices of each plane.

                // Segment
                segmentPlanes(pcl_cloud,
                              inliers_indices,
                              modelCoefficients,
                              boundaryIndices);

                if ( configuration.fusePlanes )
                {
                    planesFusion(pcl_cloud,
                                 inliers_indices,
                                 modelCoefficients,
                                 boundaryIndices);

                }

                size_t N_planes = inliers_indices.size();
                cout << "[INFO] Num of segmented planes: " <<  N_planes << endl;

                for ( size_t region_index = 0; region_index < N_planes; region_index++ )
                {
                    vector<int> &v_indices = inliers_indices[region_index].indices;

                    // Keep track of segmented regions into the scene

                    TSegmentedRegion region;

                    region.cloud = pcl_cloud;
                    region.indices = inliers_indices[region_index];

                    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
                    convex_hull.setInputCloud( pcl_cloud );
                    convex_hull.setIndices( boost::make_shared<const pcl::PointIndices> (region.indices) );
                    convex_hull.setDimension(3);
                    convex_hull.reconstruct(*region.convexHullCloud,
                                            region.polygons);

                    Eigen::Vector4f v_min;
                    Eigen::Vector4f v_max;

                    pcl::getMinMax3D( *pcl_cloud, v_indices, v_min, v_max );

                    //cout << "V_min: " << v_min.transpose() << endl;
                    //cout << "V_max: " << v_max.transpose() << endl;

                    region.box->setBoxCorners( TPoint3D(v_min(2),-v_min(0),v_min(1)),
                                               TPoint3D(v_max(2),-v_max(0),v_max(1)));

                    region.box->setWireframe( true );
                    region.box->setColor( 1,0,0 );

                    if ( configuration.trackClusters )
                    {
                        TPoint3D color;
                        int tracked = ( !obsRealIndex ) ? -1 :
                                      getTrack(region,v_regionsPerSensorAndObs[sensor_index],color);
                                      //getTrack(region,v_regionsPerSensorAndObs[sensor_index][obsRealIndex-1],color);

                        // Already existing track
                        if ( tracked != -1 )
                        {
                            region.track_id = tracked;
                            region.color    =  color;

                        }
                        // Create new track
                        else
                        {
                            region.color = v_colors[color_index];

                            color_index+=1;
                            if ( color_index > v_colors.size()-1 )
                                color_index = 0;

                            region.track_id = trackID++;

                            cout << "New track!" << endl;
                        }

                        cout << "Previously tracked: " << tracked << endl;
                    }

                    v_regionsPerSensorAndObs[sensor_index][obsRealIndex].push_back(region);

                    // Show segmentation results
                    if ( configuration.showSegmentation)
                    {
                        size_t N_inliers = v_indices.size();

                        CColouredPointsMap region_pm;

                        for( size_t point_index = 0; point_index < N_inliers; point_index++ )
                        {
                            v_indices_to_remove[v_indices[point_index]] = true;

                            float x = obs3D->points3D_x[v_indices[point_index]];
                            float y = obs3D->points3D_y[v_indices[point_index]];
                            float z = obs3D->points3D_z[v_indices[point_index]];

                            CPose3D sensorPose;
                            obs3D->getSensorPose( sensorPose );

                            TPoint3D point( sensorPose + CPoint3D(x,y,z) );

                            if ( configuration.showColouredRegions )
                                if ( configuration.trackClusters )
                                    region_pm.insertPoint( point.x, point.y, point.z,
                                                           region.color.x,
                                                           region.color.y,
                                                           region.color.z);
                                else
                                    region_pm.insertPoint( point.x, point.y, point.z,
                                                           v_colors[color_index].x,
                                                           v_colors[color_index].y,
                                                           v_colors[color_index].z);

                            else
                                region_pm.insertPoint( point.x, point.y, point.z, 0, 1 , 0);

                        }

                        mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

                        scene->insert(region.box);

                        mrpt::opengl::CPointCloudColouredPtr gl_points_region = mrpt::opengl::CPointCloudColoured::Create();
                        gl_points_region->setPointSize(7);

                        gl_points_region->loadFromPointsMap( &region_pm );

                        scene->insert( gl_points_region );

                        if ( !configuration.trackClusters )
                        {
                            color_index+=2;
                            if ( color_index > v_colors.size()-1 )
                                color_index = 0;
                        }
                    }

                    if ( configuration.labelClusters )
                    {
                        cout << "[INFO] Getting clusters labels." << endl;

                        string label;
                        getLabel( pcl_cloud, inliers_indices[region_index], label);

                        cout << "Label: " << label << endl;
                    }

                    if ( configuration.showSegmentation )
                    {
                        win3D.unlockAccess3DScene();
                        win3D.repaint();

                        if ( stepByStepExecution )
                            win3D.waitForKey();
                    }
                }
            }


            //
            // Do segmentation of arbitrary regions
            //

            if ( configuration.doEuclideanSegmentation )
            {

                pcl::PointIndices v_outliers_indices;

                for ( size_t i = 0; i < obs3D->points3D_x.size(); i++ )
                    if ( !v_indices_to_remove[i] )
                        v_outliers_indices.indices.push_back(i);

                std::vector<pcl::PointIndices> cluster_indices;

                segmentClusters( pcl_cloud, v_outliers_indices, cluster_indices);

                size_t N_clusters = cluster_indices.size();

                if ( configuration.showSegmentation || configuration.labelClusters )
                {
                    for ( size_t region_index = 0; region_index < N_clusters; region_index++ )
                    {
                        vector<int> &v_indices = cluster_indices[region_index].indices;

                        if ( configuration.showSegmentation )
                        {
                            size_t N_inliers = v_indices.size();

                            CColouredPointsMap region;

                            for( size_t point_index = 0; point_index < N_inliers; point_index++ )
                            {
                                float x = obs3D->points3D_x[v_indices[point_index]];
                                float y = obs3D->points3D_y[v_indices[point_index]];
                                float z = obs3D->points3D_z[v_indices[point_index]];

                                CPose3D sensorPose;
                                obs3D->getSensorPose( sensorPose );

                                TPoint3D point( sensorPose + CPoint3D(x,y,z) );

                                if ( configuration.showColouredRegions )
                                    region.insertPoint( point.x, point.y, point.z,
                                                        v_colors[color_index].x,
                                                        v_colors[color_index].y,
                                                        v_colors[color_index].z);
                                else
                                    region.insertPoint( point.x, point.y, point.z, 0, 0 , 1);

                            }

                            mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();


                            Eigen::Vector4f v_min;
                            Eigen::Vector4f v_max;

                            pcl::getMinMax3D( *pcl_cloud, v_indices, v_min, v_max );

                            //cout << "V_min: " << v_min.transpose() << endl;
                            //cout << "V_max: " << v_max.transpose() << endl;

                            mrpt::opengl::CBoxPtr box = CBox::Create();
                            box->setBoxCorners( TPoint3D(v_min(2),-v_min(0),v_min(1)),
                                                TPoint3D(v_max(2),-v_max(0),v_max(1)));
                            box->setWireframe( true );
                            box->setColor( 1,1,0 );

                            scene->insert(box);


                            mrpt::opengl::CPointCloudColouredPtr gl_points_region = mrpt::opengl::CPointCloudColoured::Create();
                            gl_points_region->setPointSize(7);

                            gl_points_region->loadFromPointsMap( &region );

                            scene->insert( gl_points_region );

                            color_index+=2;
                            if ( color_index > v_colors.size()-1 )
                                color_index = 0;
                        }

                        if ( configuration.labelClusters )
                        {
                            cout << "[INFO] Getting clusters labels." << endl;

                            string label;
                            getLabel( pcl_cloud, cluster_indices[region_index], label );

                            cout << "Label: " << label << endl;
                        }

                        if ( configuration.showSegmentation )
                        {
                            win3D.unlockAccess3DScene();
                            win3D.repaint();

                            if ( stepByStepExecution )
                                win3D.waitForKey();

                        }
                    }
                }
            }
        }

        scene = win3D.get3DSceneAndLock();
        N_segmented_point_clouds++;

        win3D.unlockAccess3DScene();
        win3D.repaint();

        if ( stepByStepExecution )
            win3D.waitForKey();
    }

    cout << "[INFO] Number of points clouds segmented: " << N_segmented_point_clouds << endl;


    mrpt::system::pause();

    return 0;
}
