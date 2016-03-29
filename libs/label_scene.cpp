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
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/math.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/threads.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>

using namespace pcl;

using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt;
using namespace std;

//
// GLOBAL VBLES/CONSTANTS
//

struct TConfiguration
{
    double OFFSET;
    double OFFSET_ANGLES;
    string sceneFile;
    bool autoLabelInstances;

    bool showOnlyLabels;


    TConfiguration() : OFFSET(0.02), OFFSET_ANGLES(0.02), showOnlyLabels(false),
        autoLabelInstances(true)
    {}
};

TConfiguration configuration;

struct features
{
    float L,W,H; // Lenght, width, height
};

map<string,features> m_categoryFeats;

size_t visualization_mode = 0;      // Initial visualization mode
size_t N_visualization_modes = 7;   // Number of visualization modes

enum TState{ IDLE, EDITING } STATE; // State of the labeling

string sceneFileToSave; // Where to save the labeled sceneFile

map<string,TPoint3D> m_labelColors; // A map <label,color> to fill the boxes

vector<CBoxPtr> v_boxes;    // Keep track of the boxes within the scene
vector<CText3DPtr> v_labels;// Keep track of the labels of the boxes within the scene

size_t box_editing; // Box currently editing

mrpt::gui::CDisplayWindow3D  win3D; // Window to visually perform the labeling
mrpt::opengl::COpenGLScenePtr scene; // OpenGL scene showed in the window


//-----------------------------------------------------------
//
//                   showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "Usage information. At least one expected argument: " << endl <<
            " \t -conf <conf_fil>       : Configuration file." << endl;
    cout << "Then, optional parameters:" << endl <<
            " \t -h                     : This help." << endl <<
            " \t -scene <scene_file>    : Scene file to be labeled/edited." << endl <<
            " \t -showOnlyLabels        : Show only labels (boxes)." << endl;
}


//-----------------------------------------------------------
//
//                      loadConfig
//
//-----------------------------------------------------------

void loadConfig( string const configFile )
{
    CConfigFile config( configFile );

    // Load general configuration

    configuration.showOnlyLabels = config.read_bool("GENERAL","showOnlyLabels",0,true);
    configuration.sceneFile      = config.read_string("GENERAL","sceneFile","",true);

    configuration.OFFSET         = config.read_float("GENERAL","OFFSET",0,true);
    configuration.OFFSET_ANGLES  = config.read_float("GENERAL","OFFSET_ANGLES",0,true);


    // Load object labels (classes) to be considered

    vector<string> v_labelNames;
    config.read_vector("LABELS","labelNames",vector<string>(0),v_labelNames,true);

    size_t magicNumber = ceil(pow(v_labelNames.size(),1.0/3.0));

    vector<TPoint3D> v_colors;

    for ( double r = 0; r < magicNumber; r+= 1 )
        for ( double g = 0; g < magicNumber; g+= 1 )
            for ( double b = 0; b < magicNumber; b+= 1 )
                v_colors.push_back(TPoint3D(1.0*(1-r/(magicNumber-1)),
                                            1.0*(1-g/(magicNumber-1)),
                                            1.0*(1-b/(magicNumber-1))));


    for ( size_t i_label = 0; i_label < v_labelNames.size(); i_label++ )
        m_labelColors[v_labelNames[i_label]] = v_colors[i_label];

    cout << "  [INFO] " << m_labelColors.size() << " labels considered." << endl;

    //    cout << "  [INFO] Loaded labels: " << endl;
    //    map<string,TPoint3D>::iterator it;
    //    for ( it = m_labelColors.begin(); it != m_labelColors.end(); it++ )
    //        cout << " - " << it->first << ", with color " << it->second << endl;

    cout << "  [INFO] Configuration successfully loaded." << endl;

}


//-----------------------------------------------------------
//
//                     loadParameters
//
//-----------------------------------------------------------

int loadParameters(int argc, char* argv[])
{
    bool configurationLoaded = false;

    if ( argc > 1 )
    {
        size_t arg = 1;

        while ( arg < argc )
        {
            if ( !strcmp(argv[arg],"-h") )
            {
                showUsageInformation();
                return 0;
            }
            else if ( !strcmp(argv[arg],"-conf") )
            {
                string configFile = argv[arg+1];
                loadConfig(configFile);
                configurationLoaded = true;

                arg = arg+2;
            }
            else if ( !strcmp(argv[arg],"-showOnlyLabels") )
            {
                configuration.showOnlyLabels = true;
                arg++;
            }
            else if ( !strcmp(argv[arg],"-scene") )
            {
                configuration.sceneFile = argv[arg+1];
                arg = arg+2;
            }
            else
            {
                cout << "[Error] " << argv[arg] << " unknown paramter" << endl;
                return -1;
            }
        }
    }
    else
    {
        showUsageInformation();
        return -1;
    }

    if ( !configurationLoaded )
    {
        cout << "[Error] No configuration file specified. Use -conf <conf_file>." << endl;
        return -1;
    }

    return 0;

}


//-----------------------------------------------------------
//
//                   segmentPointCloud
//
//-----------------------------------------------------------

void segmentPointCloud()
{
    // Experimental method!

    size_t N_inliers;	// Number of inliers of the fitted plane in each iteration
    std::vector<int> indices;


    CPointCloudColouredPtr opengl_cloud = scene->getByClass<CPointCloudColoured>(0);

    size_t N_points = opengl_cloud->size();
    PointCloud<PointXYZ>::Ptr cloud   (new PointCloud<PointXYZ>());

    for (size_t i = 0; i < N_points; i++)
    {
        TPoint3Df point = opengl_cloud->getPointf(i);

        cloud->push_back(PointXYZ(point.x,point.y,point.z));
    }

    do {
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr				inliers ( new pcl::PointIndices() );
        pcl::ModelCoefficients::Ptr			coeffs( new pcl::ModelCoefficients() );

        // Optional
        //seg.setOptimizeCoefficients (true);

        // Mandatory
        seg.setInputCloud( cloud );

        if ( indices.size() )
        {
            vector<int>							indicesInverse;

            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            // Extract the inliers
            extract.setInputCloud ( cloud );
            extract.setIndices( pcl::IndicesPtr( new vector<int>(indices) ) );
            extract.setNegative ( true );
            extract.filter ( indicesInverse );

            seg.setIndices( pcl::IndicesPtr( new vector<int>(indicesInverse) ) );

            cout << "Num of indices to take into accout: " << indicesInverse.size() << endl;

        }

        seg.setMethodType ( pcl::SAC_RANSAC );
        seg.setModelType ( pcl::SACMODEL_PLANE );
        seg.setMaxIterations ( 1000 );
        seg.setDistanceThreshold ( 0.10 );
        seg.setEpsAngle ( 0.17 ); // ~10 degrees

        // Detect!
        seg.segment( *inliers, *coeffs );

        N_inliers = inliers->indices.size();

        //////////////////// VISUALIZATION ///////////////////////
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Detection step"));

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOriginal( new pcl::PointCloud<pcl::PointXYZRGBA>() );
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPlaneDetected( new pcl::PointCloud<pcl::PointXYZRGBA>() );

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalAux( new pcl::PointCloud<pcl::PointXYZ>() );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneDetectedAux( new pcl::PointCloud<pcl::PointXYZ>() );

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        /////////////////// END VISUALIZATION /////////////////////

        if ( N_inliers > 1000 )
        {
            //////////////////// VISUALIZATION ///////////////////////
            if ( indices.size() )
            {
                extract.setInputCloud ( cloud );
                extract.setIndices ( pcl::IndicesPtr( new vector<int>(indices) ) );
                extract.setNegative ( true );
                extract.filter ( *cloudOriginalAux );

                cout << "Size of original aux: " << cloudOriginalAux->size() << endl;

                pcl::copyPointCloud(*cloudOriginalAux,*cloudOriginal);
            }
            else
            {
                pcl::copyPointCloud(*cloud,*cloudOriginal);

                cout << "Size of original aux (cloud): " << cloud->size() << endl;
            }
            /////////////////// END VISUALIZATION /////////////////////

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneDetectedAux( new pcl::PointCloud<pcl::PointXYZ>() );

            //CPlane plane( inliers, coeffs, type );
            //cout << plane << endl;
            //planes.push_back( plane );
            indices.insert( indices.end(), inliers->indices.begin(), inliers->indices.end() );
        }

        // Extract the inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract2;
        extract2.setInputCloud ( cloud );
        extract2.setIndices ( inliers );
        extract2.setNegative ( false );
        extract2.filter ( *cloudPlaneDetectedAux );

        cout << "Size of plane detected aux: " << cloudPlaneDetectedAux->size() << endl;

        pcl::copyPointCloud(*cloudPlaneDetectedAux,*cloudPlaneDetected);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> blue(cloudPlaneDetected,0,0,255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red(cloudOriginal,255,0,0);

        viewer->addPointCloud<pcl::PointXYZRGBA> (cloudOriginal,red,"cloud1");
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloudPlaneDetected,blue,"cloud2");

        viewer->setBackgroundColor (0.5, 0.5, 0.5);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            mrpt::system::sleep(100);
        }

    }
    while( N_inliers > 1000 );
}


//-----------------------------------------------------------
//
//                     loadCategoryFeatures
//
//-----------------------------------------------------------

void loadCategoryFeat()
{
    ifstream file("label_scene-category_features.txt");

    if ( !file.is_open() )
        return;

    size_t N_labels;

    file >> N_labels;

    for ( size_t i_label = 0; i_label < N_labels; i_label++ )
    {
        string category;
        float L,W,H;

        file >> category >> L >> W >> H;

        features feat;
        feat.L = L;
        feat.W = W;
        feat.H = H;

        m_categoryFeats[category] = feat;
    }

    cout << "  [INFO] Features of objects' categories loaded (" << N_labels << ")" << endl;
}


//-----------------------------------------------------------
//
//                     saveCategoryFeat
//
//-----------------------------------------------------------

void saveCategoryFeat()
{
    ofstream file("label_scene-category_features.txt");

    if ( !file.is_open() )
        return;

    size_t N_labels = m_categoryFeats.size();

    file << N_labels << endl;


    for ( map<string,features>::iterator it = m_categoryFeats.begin();
          it != m_categoryFeats.end(); it++ )
    {
        file << it->first << " " << it->second.L << " " << it->second.W << " " << it->second.H << endl;
    }

    cout << "  [INFO] Features of objects' categories saved to file (" << N_labels << ")" << endl;
}


//-----------------------------------------------------------
//
//                     getCategoryFeat
//
//----------------------------------------------------------

size_t getCategoryFeat(const string &cat, features &feat)
{
    if ( m_categoryFeats.count(cat) )
    {
        feat = m_categoryFeats[cat];
        return 1;
    }
    else // Category doesn't exist
        return 0;
}


//-----------------------------------------------------------
//
//                    updateCategoryFeat
//
//----------------------------------------------------------

void updateCategoryFeat(string &cat, features &feat)
{
    if ( cat.empty() )
        return;

    if ( m_categoryFeats.count(cat) )
    {
        features new_feat;
        features old_feat = m_categoryFeats[cat];
        new_feat.L = old_feat.L*0.9 + feat.L*0.1;
        new_feat.W = old_feat.W*0.9 + feat.W*0.1;
        new_feat.H = old_feat.H*0.9 + feat.H*0.1;

        m_categoryFeats[cat] = new_feat;
    }
    else // Category doesn't exist
        m_categoryFeats[cat] = feat;

}

//-----------------------------------------------------------
//
//                        getColor
//
//-----------------------------------------------------------

TPoint3D getColor( const string label )
{
    map<string,TPoint3D>::iterator it;

    for ( it = m_labelColors.begin(); it != m_labelColors.end(); it++ )
        if ( label.find( it->first ) != string::npos )
            return TPoint3D(it->second.x, it->second.y, it->second.z );

    return TPoint3D(1.0,1.0,1.0);
}


//-----------------------------------------------------------
//
//                  changeBoxesVisualization
//
//-----------------------------------------------------------

void changeBoxesVisualization(bool keepVisualizationMode = false)
{
    if (!keepVisualizationMode)
        visualization_mode++;

    for ( size_t i = 0; i < v_boxes.size(); i++ )
    {
        if ( visualization_mode % N_visualization_modes == 0 )
        {
            v_boxes[i]->setWireframe(true);
            v_boxes[i]->enableBoxBorder(false);

            v_boxes[i]->setColor(1.0,1.0,1.0);
        }
        else if ( visualization_mode % N_visualization_modes == 1 )
        {
            v_boxes[i]->setWireframe(false);
            v_boxes[i]->enableBoxBorder(true);

            v_boxes[i]->setBoxBorderColor(TColor(0,0,0));
            TPoint3D color = getColor(v_boxes[i]->getName());
            v_labels[i]->setColor(color.x,color.y,color.z);
        }
        else if ( visualization_mode % N_visualization_modes == 2 )
        {
            v_boxes[i]->setWireframe(false);
            v_boxes[i]->enableBoxBorder(false);

            TPoint3D color = getColor(v_boxes[i]->getName());
            v_boxes[i]->setColor(color.x,color.y,color.z);
            v_labels[i]->setColor(1.0,1.0,1.0);
        }
        else if ( visualization_mode % N_visualization_modes == 3 )
        {
            v_boxes[i]->setWireframe(true);
            v_boxes[i]->enableBoxBorder(false);
        }
        else if ( visualization_mode % N_visualization_modes == 4 )
        {
            v_boxes[i]->setWireframe(false);
            v_boxes[i]->enableBoxBorder(true);

            TPoint3D color = getColor(v_boxes[i]->getName());
            v_boxes[i]->setColor(color.x,color.y,color.z);
        }
        else if ( visualization_mode % N_visualization_modes == 5 )
        {
            v_boxes[i]->setWireframe(false);
            v_boxes[i]->enableBoxBorder(true);

            TPoint3D color = getColor(v_boxes[i]->getName());
            v_boxes[i]->setColor(color.x,color.y,color.z,0.4);
        }
        else if ( visualization_mode % N_visualization_modes == 6 )
        {
            v_boxes[i]->setWireframe(false);
            v_boxes[i]->enableBoxBorder(true);

            v_boxes[i]->setBoxBorderColor(TColor(255,255,255));
        }

    }

    win3D.forceRepaint();

}


//-----------------------------------------------------------
//
//                      sortCorners
//
//-----------------------------------------------------------

void sortCorners( const vector<TPoint3D> &toSort, vector<TPoint3D> &sorted, const char axis )
{
    vector<bool> inserted(8,false);
    for ( size_t i = 0; i < 8; i++ )
    {
        double max = -std::numeric_limits<double>::max();
        size_t index = 0;

        for ( size_t j = 0; j < 8; j++ )
        {
            double value = ( axis == 'x' ) ? toSort[j].x :
                                             ( axis == 'y' ) ? toSort[j].y :
                                                               toSort[j].z;
            if ( value >= max && !inserted[j] )
            {
                max = value;
                index = j;
            }
        }

        sorted[i] = toSort[index];
        inserted[index] = true;
    }
}


//-----------------------------------------------------------
//
//                changeUnderlyingPointCloud
//
//-----------------------------------------------------------

void changeUnderlyingPointCloud()
{
    string &sceneFile = configuration.sceneFile;
    string newSceneFile;

    cout << "Insert the new scene file. It has to be one not previously labeled: ";

    cin.ignore();
    std::getline(std::cin, newSceneFile);

    // Check if the path is between comas
    if ( newSceneFile[0]== '\'' )
        newSceneFile = newSceneFile.substr(1);
    if ( newSceneFile[newSceneFile.size()-1] == '\'' )
        newSceneFile = newSceneFile.substr(0,newSceneFile.size()-1);

    cout << "  [INFO] Checking if '" << newSceneFile << "' is a valid scene file ..." ;

    mrpt::opengl::COpenGLScene sceneBis;

    if ( newSceneFile.compare(newSceneFile.size()-15,15,"_labelled.scene") )
    {
        if ( sceneBis.loadFromFile(newSceneFile) )
        {
            cout << " valid :)" << endl;
            sceneFile = newSceneFile;
            sceneFileToSave = sceneFile.substr(0,sceneFile.size()-6) + "_labelled.scene";
        }
        else
        {
            cout << " invalid :( unable to load the scene" << endl;
            return;
        }
    }
    else
    {
        cout << " invalid :( the scene must be not labelled yet!" << endl;
        return;
    }

    mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

    //
    // Load new scene!
    //

    scene->loadFromFile( sceneFile );

    for ( size_t box_index = 0;
          box_index < v_boxes.size();
          box_index++)
        scene->insert( v_boxes[box_index] );

    for ( size_t text_index = 0;
          text_index < v_labels.size();
          text_index++)
        scene->insert( v_labels[text_index] );


    win3D.unlockAccess3DScene();
    win3D.forceRepaint();

    cout << "  [INFO] New scene successfully loaded" << endl;

}


//-----------------------------------------------------------
//
//                      showIDLEMenu
//
//-----------------------------------------------------------

void showIDLEMenu()
{
    win3D.addTextMessage(0.02,0.06+0.03*1, "[Actions] 'l': show/hide list boxes 'n': new box 'e': edit box 'd': delete box", TColorf(1,1,0),10,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*0, "          'c': change underlying scene 's': save scene 'v': change visualization 'esc': exit", TColorf(1,1,0),11,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*2, "", TColorf(1,1,1),12,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*3, "", TColorf(1,1,1),13,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*4, "", TColorf(1,1,1),14,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
}


//-----------------------------------------------------------
//
//                      showEDITINGMenu
//
//-----------------------------------------------------------

void showEDITINGMenu()
{
    win3D.addTextMessage(0.02,0.06+0.03*0, "[Rot]   'Ins': +yaw 'Del': -yaw 'Home': +pitch 'End': -pitch 'Pag-up': +roll 'Pag-down': -roll", TColorf(1,1,0),10,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*1, "[Moves] 'up': +x 'down': -x 'left': +y 'right': -y '1': +z '0': -z", TColorf(1,1,0),11,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*2, "[Size]  '7': +x '4': -x '8': +y '5': -y '9': +z '6': -z", TColorf(1,1,0),12,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*3, "[Misc]  'l': label 'r': reset 'enter': finish editing", TColorf(1,1,0),13,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*4, "", TColorf(1,1,1),14,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
}


//-----------------------------------------------------------
//
//                      changeState
//
//-----------------------------------------------------------

void changeState(const TState newState)
{
    if ( newState == IDLE )
    {
        STATE = IDLE;
        showIDLEMenu();
    }
    else if ( newState == EDITING )
    {
        STATE = EDITING;
        showEDITINGMenu();
    }

    win3D.forceRepaint();

}


//-----------------------------------------------------------
//
//                 initializeVisualization
//
//-----------------------------------------------------------

void initializeVisualization()
{
    win3D.setWindowTitle("Scene labelling");

    win3D.resize(600,450);

    win3D.setCameraAzimuthDeg(140);
    win3D.setCameraElevationDeg(20);
    win3D.setCameraZoom(6.0);
    win3D.setCameraPointingToPoint(0,0,0);

    scene = win3D.get3DSceneAndLock();

    opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-7,7,-7,7,0,1);
    obj->setColor(0.7,0.7,0.7);
    obj->setLocation(0,0,0);
    scene->insert( obj );

    if (scene->loadFromFile(configuration.sceneFile))
        cout << "  [INFO] Scene " << configuration.sceneFile << " loaded" << endl;
    else
        cout << "  [INFO] Error while loading scene " << configuration.sceneFile << endl;

    win3D.unlockAccess3DScene();
    win3D.repaint();

    //
    // Already labelled scene?
    //

    if ( !configuration.sceneFile.compare(configuration.sceneFile.size()-15,15,"_labelled.scene") )
    {
        cout << "  [INFO] Scene previously labelled. Loading labels... ";

        // Load previously inserted boxes
        bool keepLoading = true;
        size_t boxes_inserted = 0;

        while ( keepLoading )
        {
            CBoxPtr box = scene->getByClass<CBox>(boxes_inserted);
            if ( box.null() )
                keepLoading = false;
            else
                v_boxes.push_back( box );

            CText3DPtr text = scene->getByClass<CText3D>(boxes_inserted);

            if ( !text.null() )
                v_labels.push_back(text);

            boxes_inserted++;
        }

        cout << v_boxes.size() <<  " boxes loaded " << endl;
        sceneFileToSave = configuration.sceneFile;
    }
    else
    {
        cout << "  [INFO] Scene not previously labelled. Press 'n' to start labelling." << endl;
        sceneFileToSave = configuration.sceneFile.substr(0,configuration.sceneFile.size()-6)
                + "_labelled.scene";
    }

    //
    // Check if the user wants to only visualize labels
    //

    if ( configuration.showOnlyLabels )
    {
        scene = win3D.get3DSceneAndLock();
        scene->clear();

        for ( size_t box_index = 0;
              box_index < v_boxes.size();
              box_index++)
            scene->insert( v_boxes[box_index] );

        for ( size_t text_index = 0;
              text_index < v_labels.size();
              text_index++)
            scene->insert( v_labels[text_index] );

        win3D.unlockAccess3DScene();
        win3D.repaint();
    }

    box_editing = 0;

    //
    // Load features from the object categories previously considered
    loadCategoryFeat();

    changeState(IDLE);
}


//-----------------------------------------------------------
//
//                     showStatusMessage
//
//-----------------------------------------------------------

void showStatusMessage(const string &text)
{
    win3D.addTextMessage(0.02,1-0.04, format("%s",text.c_str()), TColorf(1,0,1),20,MRPT_GLUT_BITMAP_TIMES_ROMAN_24 );
    win3D.forceRepaint();
}


//-----------------------------------------------------------
//
//                     readStringFromWindow
//
//-----------------------------------------------------------

string readStringFromWindow(const string &text, string initial_string="")
{
    string read = initial_string;
    bool done = false;

    while(!done)
    {
        win3D.addTextMessage(0.02,1-0.04, format("%s %s",text.c_str(),read.c_str()), TColorf(0,1,1),20,MRPT_GLUT_BITMAP_TIMES_ROMAN_24 );
        win3D.forceRepaint();
        cout << '\r' << text << read << "                 ";
        cout.flush();

        int key = win3D.waitForKey(false);

        switch (key)
        {
        case MRPTK_BACK:
            if ( read.size() )
                read.erase( read.end()-1, read.end());
            break;
        case MRPTK_RETURN:
            done = true;
            break;
        default:
            read.append(string(1,char(key)));
        }


        //cout << read << '\xd';
        cout.flush();
    }

    win3D.addTextMessage(0.02,1-0.06, "", TColorf(1,1,1),20,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );

    cout << endl;

    return read;
}


//-----------------------------------------------------------
//
//                    updateListOfBoxes
//
//-----------------------------------------------------------

void updateVisualListOfBoxes(bool changeState=true)
{
    static bool shownListOfBoxes = false;

    if ( changeState )
        shownListOfBoxes = !shownListOfBoxes;

    if ( !v_boxes.size() )
        cout << "    No boxes stored" << endl;

    for ( size_t box_index = 0; box_index < v_boxes.size(); box_index++ )
    {
        if ( shownListOfBoxes )
        {
            if ( changeState )
            {
                cout << "    Index " << box_index
                 << " name " << v_boxes[box_index]->getName()
                 << endl;
            }

            if ( box_index < 38 )
                win3D.addTextMessage(0.02,1-0.09-0.02*box_index,format("Index %lu %s",box_index,v_boxes[box_index]->getName().c_str()), TColorf(0,1,1),50+box_index,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
            else
                win3D.addTextMessage(1-0.12,1-0.09-0.02*(box_index-38),format("Index %lu %s",box_index,v_boxes[box_index]->getName().c_str()), TColorf(0,1,1),50+box_index,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
        }
        else
            win3D.addTextMessage(0.02,1-0.09-0.02*box_index,"", TColorf(0,1,1),50+box_index,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    }

    win3D.addTextMessage(0.02,1-0.09-0.02*v_boxes.size(),"", TColorf(0,1,1),50+v_boxes.size(),MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );

    if (changeState)
    {
        if ( shownListOfBoxes )
            cout << "  [INFO] Showing list of stored boxes" << endl;
        else
            cout << "  [INFO] Hiding list of stored boxes" << endl;
    }

    win3D.forceRepaint();
}


//-----------------------------------------------------------
//
//                         addNewBox
//
//-----------------------------------------------------------

void addNewBox(string &objectCategory)
{
    // Get category name
    string cat = readStringFromWindow("    Adding bounding box. Insert the category of the object (empty to skip): ");
    cout.flush();

    opengl::CBoxPtr box;
    opengl::CText3DPtr text = opengl::CText3D::Create();

    if ( !cat.empty() )
    {
        // New object category (label)?
        features feat;

        int res = getCategoryFeat(cat,feat);

        if ( res ) // Existing category
        {
            box = opengl::CBox::Create(TPoint3D(-feat.W/2,-feat.L/2,-feat.H/2),
                                       TPoint3D(feat.W/2,feat.L/2,feat.H/2) );
        }
        else// New category
        {
            cout << "  [INFO] New object category! '" << cat << "'" << endl;
            box = opengl::CBox::Create(TPoint3D(-0.05,-0.1,-0.1),TPoint3D(0.1,0.1,0.1) );
        }

        objectCategory = cat;

        //getInstanceLabel(cat);

        if ( configuration.autoLabelInstances )
        {
            size_t id = 0;
            bool alreadyExists = true;

            while ( alreadyExists )
            {
                alreadyExists = false;

                for ( size_t i_box = 0; i_box < v_boxes.size(); i_box++)
                {
                    if (!(format("%s_%lu",cat.c_str(),id).compare(v_boxes[i_box]->getName())))
                    {
                        alreadyExists = true;
                        id++;
                        break;
                    }
                }
            }
            text->setString( format("%s_%lu",cat.c_str(),id) );
            text->setScale(0.06);
            box->setName( format("%s_%lu",cat.c_str(),id) );
        }
    }
    else
    {
        //opengl::CBoxPtr box = opengl::CBox::Create(TPoint3D(-0.1,-0.2,-0.2),TPoint3D(0.2,0.2,0.2) );
        box = opengl::CBox::Create(TPoint3D(-0.05,-0.1,-0.1),TPoint3D(0.1,0.1,0.1) );        
        text->setString( box->getName() );
    }

    win3D.addTextMessage(0.02,0.06+0.03*2, "Adding bounding box. Move the mouse to the object centroid in the XY plane and press any key.", TColorf(1,1,1),12,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.forceRepaint();

    mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

    box->setLineWidth(5);

    scene->insert( box );
    scene->insert( text );

    win3D.unlockAccess3DScene();
    win3D.forceRepaint();

    // Move the mouse to the object centroid in the XY plane and push any key

    win3D.waitForKey();

    TPoint3D point;
    TLine3D ray;
    TObject3D obj;

    win3D.getLastMousePositionRay(ray);

    TPlane planeXY(TPoint3D(1,0,0),TPoint3D(0,1,0),TPoint3D(0,0,0));

    mrpt::math::intersect(planeXY,ray,obj);

    obj.getPoint(point);

    CPose3D newPose;
    newPose.x(point.x);
    newPose.y(point.y);

    box->setPose(newPose);

    win3D.forceRepaint();

    /*// Move the mouse to the object centroid in the XZ plane and push any key

    win3D.waitForKey();

    win3D.getLastMousePositionRay(ray);

    TPlane planeXZ(TPoint3D(1,0,0),TPoint3D(0,0,1),TPoint3D(0,0,0));

    intersect(planeXZ,ray,obj);
    obj.getPoint(point);

    newPose.z(point.z);

    win3D.clearKeyHitFlag();*/

    win3D.addTextMessage(0.02,0.06+0.03*2, "Adding bounding box. 'Now press the left mouse button 'a' (bottom) 's' (middle) or 'd' (top) to set the height.", TColorf(1,1,1),12,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );

    int key = -1;

    while ( ( key != 'a' ) && ( key != 's' )
            && ( key != 'd' ) )
        key = win3D.waitForKey();

    TPoint3D c1,c2;
    box->getBoxCorners(c1,c2);

    switch (key)
    {
    case 'a':
            newPose.z(c2.z);
            break;
    case 's':
            newPose.z(c2.z + 0.75);
            break;
    case 'd':
            newPose.z(c2.z + 1.2);
            break;
    }

    // Set the new box pose!

    box->setPose(newPose);
    win3D.forceRepaint();
    win3D.clearKeyHitFlag();

    v_boxes.push_back( box );
    v_labels.push_back( text );

    // Update visualization
    changeBoxesVisualization(true);

    changeState(EDITING);
    box_editing = v_boxes.size()-1;

    updateVisualListOfBoxes(false);
}


//-----------------------------------------------------------
//
//                       labelScene
//
//-----------------------------------------------------------

void labelScene()
{

    double &OFFSET = configuration.OFFSET;
    double &OFFSET_ANGLES = configuration.OFFSET_ANGLES;

    string objectCategory;

    bool end = false;

    while ( win3D.isOpen() && !end )
    {

        /*CSpherePtr sphere1 = CSphere::Create(0.025);
        CSpherePtr sphere2 = CSphere::Create(0.025);
        CSpherePtr sphere3 = CSphere::Create(0.025);
        CSpherePtr sphere4 = CSphere::Create(0.025);
        CSpherePtr sphere5 = CSphere::Create(0.05);
        CSpherePtr sphere6 = CSphere::Create(0.05);
        CSpherePtr sphere7 = CSphere::Create(0.05);
        CSpherePtr sphere8 = CSphere::Create(0.05);

        scene->insert( sphere1 );
        scene->insert( sphere2 );
        scene->insert( sphere3 );
        scene->insert( sphere4 );
        scene->insert( sphere5 );
        scene->insert( sphere6 );
        scene->insert( sphere7 );
        scene->insert( sphere8 );labelScene

        CSpherePtr sphereTest = CSphere::Create(0.05);
        sphereTest->setLocation(1,0,0);
        scene->insert( sphereTest );*/

        if ( win3D.keyHit() )
        {
            if ( STATE == IDLE )
            {

                int key = win3D.getPushedKey();

                switch ( key )
                {
                case ('n') : // New bounding box
                {
                    addNewBox(objectCategory);

                    cout << "  [INFO] Inserted new box to edit" << endl;

                    break;
                }
                case ('l') :
                {
                    updateVisualListOfBoxes();

                    break;
                }
                case ('e') :
                {
                    size_t box_index;

                    string boxToEdit = readStringFromWindow("    Insert the box index or label to edit: ");

                    // Check if it's a number
                    if ( strspn( boxToEdit.c_str(), "0123456789" ) == boxToEdit.size() )
                        box_index = atoi(boxToEdit.c_str());
                    else // It's the label of a box
                    {
                        bool found = false;
                        size_t index = 0;

                        while (!found && (index < v_boxes.size()) )
                        {
                            if ( boxToEdit == v_boxes[index]->getName() )
                            {
                                found = true;
                                box_index = index;
                            }
                            else
                                index++;
                        }

                        if ( !found )
                        {
                            cout << "    Invalid box label. Press 'b' to see a list of the stored boxes." << endl;
                            break;
                        }
                    }

                    if ( box_index >= 0 && box_index < v_boxes.size() )
                    {
                        box_editing = box_index;
                        changeState(EDITING);
                        cout << "  [INFO] editin box with index " << box_editing << endl;
                    }
                    else
                        cout << "    Invalid box index. Press 'b' to see a list of the stored boxes." << endl;

                    break;
                }
                case ('s') :
                {
                    cout << "  [INFO] Saving scene to " << sceneFileToSave << " ... ";
                    cout.flush();

                    showStatusMessage("Saving scene...");

                    if ( scene->saveToFile(sceneFileToSave) )
                    {
                        cout << "done" << endl;
                        showStatusMessage("Saving scene... DONE!");
                    }
                    else
                    {
                        cout << "error" << endl;
                        showStatusMessage("Saving scene... ERROR!");
                    }

                    saveCategoryFeat();

                    mrpt::system::sleep(2000);
                    showStatusMessage("");

                    break;
                }
                case ('v') :
                {
                    changeBoxesVisualization();

                    break;
                }
                case ('c') :
                {
                    changeUnderlyingPointCloud();

                    break;
                }
                case ('d'):
                {
                    size_t box_index;
                    cout << "Insert the index of the box to remove: ";
                    cin >> box_index;

                    if ( box_index >= 0 && box_index < v_boxes.size() )
                    {
                        mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

                        scene->removeObject(v_boxes[box_index]);
                        scene->removeObject(v_labels[box_index]);

                        v_boxes.erase(v_boxes.begin()+box_index);
                        v_labels.erase(v_labels.begin()+box_index);

                        win3D.unlockAccess3DScene();
                        win3D.forceRepaint();

                        cout << "  [INFO] Box deleted" << endl;
                    }
                    else
                        cout << "  [INFO] Error, invalid box index. Press 'b' to see the list of tracked boxes" << endl;

                    updateVisualListOfBoxes(false);

                    break;
                }
                case (MRPTK_ESCAPE) :
                {
                    string option = readStringFromWindow("    Exiting, are you sure? (y/n): ");

                    if ( option == "y" )
                    {
                        cout << endl;
                        end = true;
                    }

                    break;
                }
                default:
                    break;
                }
            }
            else if ( STATE == EDITING )
            {

                CBoxPtr box = v_boxes[box_editing];
                CText3DPtr label = v_labels[box_editing];

                int key = win3D.getPushedKey();
                TPose3D boxPose = box->getPose();

                TPoint3D c1,c2;

                box->getBoxCorners(c1,c2);

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
                    CPose3D cameraPose;
                    win3D.getDefaultViewport()->getCurrentCameraPose(cameraPose);
                    CPose3D move(-OFFSET,0,0,0,0);
                    CPose3D p1 = -cameraPose+boxPose;
                    box->setPose(cameraPose+(move+p1));

                    break;
                }
                case ( MRPTK_DOWN ) : // x axis down
                {
                    CPose3D cameraPose;
                    win3D.getDefaultViewport()->getCurrentCameraPose(cameraPose);
                    cameraPose.setYawPitchRoll(cameraPose.yaw(),cameraPose.pitch(),0);
                    CPose3D move(0,-OFFSET,0,0,0,0);
                    CPose3D p1 = -cameraPose+boxPose;
                    box->setPose(cameraPose+(move+p1));

                    break;
                }
                case ( MRPTK_RIGHT ) : // y axis right
                {
                    CPose3D cameraPose;
                    win3D.getDefaultViewport()->getCurrentCameraPose(cameraPose);
                    CPose3D move(OFFSET,0,0,0,0,0);
                    CPose3D p1 = -cameraPose+boxPose;
                    box->setPose(cameraPose+(move+p1));

                    break;
                }
                case ( MRPTK_UP ) : // x axis up
                {
                    CPose3D cameraPose;
                    win3D.getDefaultViewport()->getCurrentCameraPose(cameraPose);
                    cameraPose.setYawPitchRoll(cameraPose.yaw(),cameraPose.pitch(),0);
                    CPose3D move(0,OFFSET,0,0,0,0);
                    CPose3D p1 = -cameraPose+boxPose;
                    box->setPose(cameraPose+(move+p1));

                    break;
                }
                case ('1') : // z axis up
                {
                    CPose3D move(0,0,OFFSET,0,0,0);
                    box->setPose(move+boxPose);

                    break;
                }
                case ('0') : // z axis down255
                {
                    CPose3D move(0,0,-OFFSET,0,0,0);
                    box->setPose(move+boxPose);

                    break;
                }
                case ( MRPTK_INSERT ) : // yaw
                {
                    boxPose.getAsVector(v_pose);
                    CPose3D move(0,0,0,OFFSET_ANGLES,0,0);
                    CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                    CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                    box->setPose(poseFinal);

                    break;
                }
                case ( MRPTK_DELETE ) : // -yaw
                {
                    boxPose.getAsVector(v_pose);
                    CPose3D move(0,0,0,-OFFSET_ANGLES,0,0);
                    CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                    CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                    box->setPose(poseFinal);
                    break;
                }
                case ( MRPTK_HOME ) : // pitch
                {
                    CPose3D move(0,0,0,0,-OFFSET_ANGLES,0);
                    boxPose.getAsVector(v_pose);
                    CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                    CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                    box->setPose(poseFinal);

                    break;
                }
                case ( MRPTK_END ) : // -pitch
                {
                    CPose3D move(0,0,0,0,OFFSET_ANGLES,0);
                    boxPose.getAsVector(v_pose);
                    CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                    CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                    box->setPose(poseFinal);

                    break;
                }
                case ( MRPTK_PAGEUP ) : // roll
                {
                    CPose3D move(0,0,0,0,0,-OFFSET_ANGLES);
                    boxPose.getAsVector(v_pose);
                    CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                    CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                    box->setPose(poseFinal);

                    break;
                }
                case ( MRPTK_PAGEDOWN ) : // -roll
                {
                    CPose3D move(0,0,0,0,0,OFFSET_ANGLES);
                    boxPose.getAsVector(v_pose);
                    CPose3D( move + CPose3D(0,0,0,v_pose[3],v_pose[4], v_pose[5] ) ).getAsVector(v_pose2);
                    CPose3D poseFinal(v_pose[0], v_pose[1], v_pose[2],v_pose2[3],v_pose2[4],v_pose2[5]);

                    box->setPose(poseFinal);

                    break;
                }

                case ('R') : // RESET
                {
                    CPose3D move(0,0,0,0,0,0);
                    box->setPose(move);

                    break;
                }
                case ('7') : // Increase size in x
                {
                    box->setBoxCorners(c1-TPoint3D(OFFSET/2,0,0),c2+TPoint3D(OFFSET/2,0,0));

                    break;
                }
                case ('4') : // Decrease size in x
                {
                    box->setBoxCorners(c1+TPoint3D(OFFSET/2,0,0),c2-TPoint3D(OFFSET/2,0,0));

                    break;
                }
                case ('8') : // Increase size in y
                {
                    box->setBoxCorners(c1-TPoint3D(0,OFFSET/2,0),c2+TPoint3D(0,OFFSET/2,0));

                    break;
                }
                case ('5') : // Decrease size in y
                {
                    box->setBoxCorners(c1+TPoint3D(0,OFFSET/2,0),c2-TPoint3D(0,OFFSET/2,0));

                    break;
                }
                case ('9') : // Increase size in z
                {
                    box->setBoxCorners(c1,c2+TPoint3D(0,0,OFFSET/2));

                    break;
                }
                case ('6') : // Decrease size in z
                {
                    box->setBoxCorners(c1,c2-TPoint3D(0,0,OFFSET/2));

                    break;
                }
                case ('l') : // add label
                {
                    string boxLabel = readStringFromWindow("    Setting box_label: ", objectCategory);
                    cout.flush();

                    label->setString( boxLabel );
                    label->setScale(0.06);

                    box->setName(boxLabel);

                    updateVisualListOfBoxes(false);

                    break;
                }
                case ('v') :
                {
                    changeBoxesVisualization();

                    break;
                }
                case ( MRPTK_RETURN ): // Finished editing
                {
                    TPose3D pose = box->getPose();

                    box->getBoxCorners(c1,c2);

                    TPoint3D C111 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c1.y,c1.z)) );
                    TPoint3D C112 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c1.y,c2.z)) );
                    TPoint3D C121 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c2.y,c1.z)) );
                    TPoint3D C122 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c2.y,c2.z)) );
                    TPoint3D C211 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c1.y,c1.z)) );
                    TPoint3D C212 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c1.y,c2.z)) );
                    TPoint3D C221 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c2.y,c1.z)) );
                    TPoint3D C222 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c2.y,c2.z)) );

                    features feat;

                    feat.L = C111.distanceTo(C121);
                    feat.W = C111.distanceTo(C211);
                    feat.H = C111.distanceTo(C112);

                    updateCategoryFeat(objectCategory,feat);

                    objectCategory = "";

                    changeState(IDLE);

                    cout << "  [INFO] Finished editing box. IDLE state." << endl;
                    break;
                }
                default:
                    break;
                }

                mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

                TPose3D pose = box->getPose();

                box->getBoxCorners(c1,c2);

                TPoint3D C111 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c1.y,c1.z)) );
                TPoint3D C112 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c1.y,c2.z)) );
                TPoint3D C121 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c2.y,c1.z)) );
                TPoint3D C122 ( CPose3D(pose) + TPose3D(TPoint3D(c1.x,c2.y,c2.z)) );
                TPoint3D C211 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c1.y,c1.z)) );
                TPoint3D C212 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c1.y,c2.z)) );
                TPoint3D C221 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c2.y,c1.z)) );
                TPoint3D C222 ( CPose3D(pose) + TPose3D(TPoint3D(c2.x,c2.y,c2.z)) );

                /* vector<TPoint3D> toSort(8);
                    vector<TPoint3D> sorted_x(8);
                    vector<TPoint3D> sorted_y(8);
                    vector<TPoint3D> sorted_z(8);

                    toSort[0] = C111; toSort[1] = C112; toSort[2] = C121; toSort[3] = C122;
                    toSort[4] = C211; toSort[5] = C212; toSort[6] = C221; toSort[7] = C222;

                    sortCorners( toSort, sorted_x, 'x' );
                    sortCorners( toSort, sorted_y, 'y' );
                    sortCorners( toSort, sorted_z, 'z' );

                    cout << "Sorted x:" << endl;
                    for ( size_t i = 0; i < 8; i++ )
                        cout << sorted_x[i] << endl;

                    cout << "Sorted y:" << endl;
                    for ( size_t i = 0; i < 8; i++ )
                        cout << sorted_y[i] << endl;

                sphere1->setPose( TPose3D(C111) );
                sphere2->setPose( TPose3D(C112) );
                sphere3->setPose( TPose3D(C121) );
                sphere4->setPose( TPose3D(C122) );
                sphere5->setPose( TPose3D(C211) );
                sphere6->setPose( TPose3D(C212) );
                sphere7->setPose( TPose3D(C221) );
                sphere8->setPose( TPose3D(C222) );

                TPlane3D plane1(sorted_x[0],sorted_x[1],sorted_x[2]);
                TPlane3D plane2(sorted_x[5],sorted_x[6],sorted_x[7]);
                TPlane3D plane3(sorted_y[0],sorted_y[1],sorted_y[2]);
                TPlane3D plane4(sorted_y[5],sorted_y[6],sorted_y[7]);
                TPlane3D plane5(sorted_z[0],sorted_z[1],sorted_z[2]);
                TPlane3D plane6(sorted_z[5],sorted_z[6],sorted_z[7]);

                double v_normal1[3];
                double v_normal2[3];
                double v_normal3[3];
                double v_normal4[3];
                double v_normal5[3];
                double v_normal6[3];

                plane1.getNormalVector( v_normal1 );
                plane2.getNormalVector( v_normal2 );
                plane3.getNormalVector( v_normal3 );
                plane4.getNormalVector( v_normal4 );
                plane5.getNormalVector( v_normal5 );
                plane6.getNormalVector( v_normal6 );

                cout << "v_normal: " << v_normal1[0] << " " << v_normal1[1] << " " << v_normal1[2] << endl;
                cout << "v_normal: " << v_normal2[0] << " " << v_normal2[1] << " " << v_normal2[2] << endl;
                cout << "v_normal: " << v_normal3[0] << " " << v_normal3[1] << " " << v_normal3[2] << endl;
                cout << "v_normal: " << v_normal4[0] << " " << v_normal4[1] << " " << v_normal4[2] << endl;
                cout << "v_normal: " << v_normal5[0] << " " << v_normal5[1] << " " << v_normal5[2] << endl;
                cout << "v_normal: " << v_normal6[0] << " " << v_normal6[1] << " " << v_normal6[2] << endl;

                TPoint3D pointToTest(1,0,0);

                cout << "Evaluation plane1: " << plane1.evaluatePoint( pointToTest )*v_normal1[0] << endl;
                cout << "Evaluation plane2: " << plane2.evaluatePoint( pointToTest )*v_normal2[0] << endl;
                cout << "Evaluation plane3: " << plane3.evaluatePoint( pointToTest )*v_normal3[1] << endl;
                cout << "Evaluation plane4: " << plane4.evaluatePoint( pointToTest )*v_normal4[1] << endl;
                cout << "Evaluation plane5: " << plane5.evaluatePoint( pointToTest )*v_normal5[2] << endl;
                cout << "Evaluation plane6: " << plane6.evaluatePoint( pointToTest )*v_normal6[2] << endl;

                if ( ( plane1.evaluatePoint( pointToTest )*v_normal1[0] <= 0 ) &&
                     ( plane2.evaluatePoint( pointToTest )*v_normal2[0] >= 0 ) &&
                     ( plane3.evaluatePoint( pointToTest )*v_normal3[1] <= 0 ) &&
                     ( plane4.evaluatePoint( pointToTest )*v_normal4[1] >= 0 ) &&
                     ( plane5.evaluatePoint( pointToTest )*v_normal5[2] <= 0 ) &&
                     ( plane6.evaluatePoint( pointToTest )*v_normal6[2] >= 0 ) )

                    sphereTest->setColor(0,0,1);
                else
                    sphereTest->setColor(1,0,0);*/

                label->setPose(TPose3D(C112));

                win3D.addTextMessage(0.02,0.02, "Box pose:" + pose.asString(), TColorf(1,1,1),0,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );

                win3D.unlockAccess3DScene();
                win3D.repaint();
            }

            mrpt::system::sleep(10);
        }

        mrpt::system::sleep(10);
    }

}


//-----------------------------------------------------------
//
//                           main
//
//-----------------------------------------------------------

int main(int argc, char* argv[])
{
    cout << endl << "-----------------------------------------------------" << endl;
    cout <<         "                Label scene app.                     " << endl;
    cout <<         "            [Object Labeling Tookit]                 " << endl;
    cout <<         "-----------------------------------------------------" << endl << endl;

    try
    {
        //
        // Load parameters

        int res = loadParameters(argc,argv);

        if (res)
            return -1;

        //
        // Initialize visualization

        initializeVisualization();

        //
        // Segment point cloud (not ready yet!)

        //segmentPointCloud();

        //
        // Label scene!

        labelScene();

        // All done, see you! :)
        return 0;

    }
    catch (exception &e)
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
