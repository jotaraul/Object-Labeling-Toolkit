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

    bool showOnlyLabels;


    TConfiguration() : OFFSET(0.02), OFFSET_ANGLES(0.02), showOnlyLabels(false)
    {}
};

TConfiguration configuration;

size_t visualization_mode = 0;      // Initial visualization mode
size_t N_visualization_modes = 7;   // Number of visualization modes

enum TState{ IDLE, EDITING } STATE; // State of the labeling

string sceneFileToSave; // Where to save the labeled sceneFile

map<string,TPoint3D> m_labelColors; // A map <label,color> to fill the boxes

vector<CBoxPtr> v_boxes;    // Keep track of the boxes within the scene
vector<CText3DPtr> v_labels;// Keep track of the labels of the boxes within the scene

mrpt::gui::CDisplayWindow3D  win3D; // Window to visually perform the labeling

//
// AUXILIAR FUNCTIONS
//


//-----------------------------------------------------------
//
//                   showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "Usage information. At least one expected argument: " << endl <<
            " \t <conf_fil>       : Configuration file." << endl;
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

    cout << "[INFO] " << m_labelColors.size() << " labels considered." << endl;

//    cout << "[INFO] Loaded labels: " << endl;
//    map<string,TPoint3D>::iterator it;
//    for ( it = m_labelColors.begin(); it != m_labelColors.end(); it++ )
//        cout << " - " << it->first << ", with color " << it->second << endl;

    cout << "[INFO] Configuration successfully loaded." << endl;

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

void changeBoxesVisualization()
{
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

    cout << "[INFO] Checking if " << newSceneFile << " is a valid scene file ..." ;

    mrpt::opengl::COpenGLScene sceneBis;

    if ( ( newSceneFile.compare(newSceneFile.size()-15,15,"_labelled.scene") )
         && sceneBis.loadFromFile(newSceneFile) )
    {        
        cout << " valid :)" << endl;
        sceneFile = newSceneFile;
        sceneFileToSave = sceneFile.substr(0,sceneFile.size()-6) + "_labelled.scene";
    }
    else
    {
        cout << " invalid :(" << endl;
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

    cout << "[INFO] New scene successfully loaded" << endl;

}


//-----------------------------------------------------------
//
//                      showIDLEMenu
//
//-----------------------------------------------------------

void showIDLEMenu()
{
    win3D.addTextMessage(0.02,0.06+0.03*1, "[Actions] 'b': list boxes 'n': new box 'e': edit box 'd': delete box", TColorf(1,1,1),10,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*0, "          'c': change underlying scene 's': save scene 'v': change visualization 'esc': exit", TColorf(1,1,1),11,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*2, "", TColorf(1,1,1),12,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*3, "", TColorf(1,1,1),13,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
}


//-----------------------------------------------------------
//
//                      showEDITINGMenu
//
//-----------------------------------------------------------

void showEDITINGMenu()
{
    win3D.addTextMessage(0.02,0.06+0.03*0, "[Rot]   'Ins': +yaw 'Del': -yaw 'Home': +pitch 'End': -pitch 'Pag-up': +roll 'Pag-down': -roll", TColorf(1,1,1),10,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*1, "[Moves] 'up': +x 'down': -x 'left': +y 'right': -y '1': +z '0': -z", TColorf(1,1,1),11,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*2, "[Size]  '7': +x '4': -x '8': +y '5': -y '9': +z '6': -z", TColorf(1,1,1),12,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
    win3D.addTextMessage(0.02,0.06+0.03*3, "[Misc]  'l': label 'r': reset 'enter': finish editing", TColorf(1,1,1),13,MRPT_GLUT_BITMAP_TIMES_ROMAN_10 );
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
//                           main
//
//-----------------------------------------------------------

int main(int argc, char* argv[])
{
    double &OFFSET = configuration.OFFSET;
    double &OFFSET_ANGLES = configuration.OFFSET_ANGLES;

    // Load configuration

    if ( argc > 1 )
    {
        string configFile = argv[1];
        loadConfig(configFile);

        if ( argc > 2 )
        {
            size_t arg = 2;

            while ( arg < argc )
            {
                if ( !strcmp(argv[arg],"-h") )
                {
                    showUsageInformation();
                    return 0;
                }
                if ( !strcmp(argv[arg],"-showOnlyLabels") )
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
    }
    else
    {
        showUsageInformation();
        return -1;
    }

    //
    // Set 3D window
    //

    win3D.setWindowTitle("Scene labelling");

    win3D.resize(600,450);

    win3D.setCameraAzimuthDeg(140);
    win3D.setCameraElevationDeg(20);
    win3D.setCameraZoom(6.0);
    win3D.setCameraPointingToPoint(0,0,0);

    mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

    opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-7,7,-7,7,0,1);
    obj->setColor(0.7,0.7,0.7);
    obj->setLocation(0,0,0);
    scene->insert( obj );

    if (scene->loadFromFile(configuration.sceneFile))
        cout << "[INFO] Scene " << configuration.sceneFile << " loaded" << endl;
    else
        cout << "[INFO] Error while loading scene " << configuration.sceneFile << endl;

    win3D.unlockAccess3DScene();
    win3D.repaint();

    //
    // Already labelled scene?
    //

    if ( !configuration.sceneFile.compare(configuration.sceneFile.size()-15,15,"_labelled.scene") )
    {
        cout << "[INFO] Scene previously labelled. Loading labels... ";

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
        cout << "[INFO] Scene not previously labelled. Press 'n' to start labelling." << endl;
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

    size_t box_editing = 0;

    changeState(IDLE);

    //
    // MAIN CONTROL LOOP
    //

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
        scene->insert( sphere8 );

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
                    opengl::CBoxPtr box = opengl::CBox::Create(TPoint3D(-0.1,-0.2,-0.2),TPoint3D(0.2,0.2,0.2) );
                    box->setWireframe();
                    box->setLineWidth(5);

                    opengl::CText3DPtr text = opengl::CText3D::Create();
                    text->setString( box->getName() );

                    mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

                    scene->insert( box );

                    scene->insert( text );

                    win3D.unlockAccess3DScene();
                    win3D.forceRepaint();

                    v_boxes.push_back( box );
                    v_labels.push_back( text );

                    changeState(EDITING);
                    box_editing = v_boxes.size()-1;

                    cout << "[INFO] Inserted new box to edit" << endl;

                    break;
                }
                case ('b') :
                {
                    cout << "[INFO] Showing list of stored boxes" << endl;

                    if ( !v_boxes.size() )
                        cout << "No boxes stored" << endl;

                    for ( size_t box_index = 0; box_index < v_boxes.size(); box_index++ )
                        cout << "Index " << box_index
                             << " name " << v_boxes[box_index]->getName()
                             << endl;

                    break;
                }
                case ('e') :
                {
                    size_t box_index;
                    cout << "Insert the box index to edit ";
                    cin >> box_index;

                    if ( box_index >= 0 && box_index < v_boxes.size() )
                    {
                        box_editing = box_index;
                        changeState(EDITING);
                        cout << "[INFO] editin box with index " << box_editing << endl;
                    }
                    else
                        cout << "Invalid box index. Press 'b' to see a list of the stored boxes." << endl;

                    break;
                }
                case ('s') :
                {
                    cout << "[INFO] Saving scene to " << sceneFileToSave << " ... ";
                    mrpt::system::sleep(50);

                    if ( scene->saveToFile(sceneFileToSave) )
                        cout << "done" << endl;
                    else
                        cout << "error" << endl;
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

                        cout << "[INFO] Box deleted" << endl;
                    }
                    else
                        cout << "[INFO] Error, invalid box index. Press 'b' to see the list of tracked boxes" << endl;

                    break;
                }
                case (MRPTK_ESCAPE) :
                {
                    char option;
                    cout << "[INFO] Exiting, are you sure? (y/n): ";
                    cin >> option;

                    if ( option == 'y' )
                    {
                        cout << "Bye!" << endl;
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
                    CPose3D move(0,OFFSET,0,0,0,0);
                    box->setPose(move+boxPose);

                    break;
                }
                case ( MRPTK_DOWN ) : // x axis down
                {
                    CPose3D move(-OFFSET,0,0,0,0,0);
                    box->setPose(move+boxPose);

                    break;
                }
                case ( MRPTK_RIGHT ) : // y axis right
                {
                    CPose3D move(0,-OFFSET,0,0,0,0);
                    box->setPose(move+boxPose);

                    break;
                }
                case ( MRPTK_UP ) : // x axis up
                {
                    CPose3D move(OFFSET,0,0,0,0,0);
                    box->setPose(move+boxPose);

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
                    box->setBoxCorners(c1+TPoint3D(0,0,OFFSET/2),c2-TPoint3D(0,0,OFFSET/2));

                    break;
                }
                case ('l') : // add label
                {
                    string boxLabel;
                    cout << "box_label: ";
                    cin >> boxLabel;

                    label->setString( boxLabel );
                    label->setScale(0.06);

                    box->setName(boxLabel);

                    break;
                }
                case ('v') :
                {
                    changeBoxesVisualization();

                    break;
                }
                case ( MRPTK_RETURN ): // Finished editing
                {
                    changeState(IDLE);
                    cout << "[INFO] Finished editing box. IDLE state." << endl;
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


    return 0;
}
