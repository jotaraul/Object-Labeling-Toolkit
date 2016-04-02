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

#include "processing.hpp"

#include <mrpt/gui.h>

#include <mrpt/maps/CColouredPointsMap.h>

#include <mrpt/math.h>

#include <mrpt/opengl.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CAxis.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>

#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/poses/CPoint2D.h>

#include <mrpt/utils/CFileGZInputStream.h>

using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt;
using namespace std;

// Visualization vbles
mrpt::opengl::COpenGLScenePtr scene;
mrpt::gui::CDisplayWindow3DPtr  win3D;
gui::CDisplayWindowPlotsPtr	win;

// Configuration vbles
string             i_rawlogFileName; // Rawlog file name
string             i_sceneFilename; // Rawlog file name
CFileGZInputStream i_rawlog; // Rawlog file stream
bool stepByStepExecution = false; // Enables step by step execution
bool clearAfterStep = false;
bool showPoses = false;
double zUpperLimit = std::numeric_limits<double>::max();
vector<string>  sensors_to_use; // Visualize data from this sensors. Empty=all
bool setNumberOfObs = false;
size_t N_limitOfObs = std::numeric_limits<size_t>::max();
size_t N_lowerLimitOfObs = 0;
size_t decimate = 0;
float distBetweenPoints = 0.02;
float pointSize = 3;
bool equalizeRGBDHist = false;
bool visualize2Dposes = false;
bool saveAsPlainText = false;

//-----------------------------------------------------------
//
//                   showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "  Usage information. At least one expected argument: " << endl <<
            "    (1) Rawlog or scene file." << endl;

    cout << "  Then, optional parameters:" << endl <<
            "    -sensor <sensor_label> : Use obs. from this sensor (all used by default)." << endl <<
            "    -visualize2Dposes      : Visualize sensors' poses in a 2D plot." << endl <<
            "    -decimate <num>        : Visualize one of each <num> RGBD observations."  << endl <<
            "    -pointSize <num>       : Size of the points for visualization purposes (default 3)."  << endl <<
            "    -distBetweenPoints <num>: Min distance between two points to insert the second one." << endl <<
            "    -step                  : Enable step by step execution." << endl <<
            "    -equalizeRGBDHist      : Enable the equalization of the RGB images." << endl <<
            "    -clear                 : Clear the scene after a step." << endl <<
            "    -poses                 : Show spheres in the scene representing observation poses." << endl <<
            "    -zUpperLimit           : Remove points with a height higher than this paramter." << endl <<
            "    -limit                 : Sets a limit to the number of obs to process." << endl <<
            "    -lowerLimit            : Sets a lower limit to the number of obs to process." << endl <<
            "    -saveAsPlainText       : If a scene is loaded, save it as plain text." << endl << endl;

}


//-----------------------------------------------------------
//
//                    loadParameters
//
//-----------------------------------------------------------

int loadParameters(int argc, char* argv[])
{
    if ( argc > 1 )
    {
        // Get rawlog file name
        string fileName = argv[1];

        if ( !fileName.compare(fileName.size()-7,7,".rawlog") )
            i_rawlogFileName = fileName;
        else if ( !fileName.compare(fileName.size()-6,6,".scene") )
            i_sceneFilename = fileName;
        else
            return -1;

        // Get optional paramteres
        if ( argc > 2 )
        {
            size_t arg = 2;

            while ( arg < argc )
            {
                if ( !strcmp(argv[arg], "-h") )
                {
                    showUsageInformation();
                    arg++;
                }
                else if ( !strcmp(argv[arg],"-sensor") )
                {
                    string sensor = argv[arg+1];
                    arg += 2;

                    sensors_to_use.push_back( sensor );

                }
                else if ( !strcmp(argv[arg],"-saveAsPlainText") )
                {
                    saveAsPlainText = true;

                    cout << "  [INFO] Saving as plain text" << endl;

                    arg++;
                }
                else if ( !strcmp(argv[arg],"-decimate") )
                {
                    decimate = atoi(argv[arg+1]);
                    arg += 2;

                    cout << "  [INFO] Employing only 1 of each " << decimate << " obs" << endl;
                }
                else if ( !strcmp(argv[arg],"-equalizeRGBDHist") )
                {
                    equalizeRGBDHist = true;

                    cout << "  [INFO] Equalizing RGB images" << endl;
                    arg++;
                }
                else if ( !strcmp(argv[arg],"-visualize2Dposes") )
                {
                    visualize2Dposes = true;

                    cout << "  [INFO] Visualizing poses in a 2D plot." << endl;
                    arg++;
                }
                else if ( !strcmp(argv[arg],"-pointSize") )
                {
                    pointSize = atof(argv[arg+1]);
                    arg += 2;

                    cout << "  [INFO] Point size set to: " << pointSize << endl;
                }
                else if ( !strcmp(argv[arg],"-distBetweenPoints") )
                {
                    distBetweenPoints = atof(argv[arg+1]);
                    arg += 2;

                    cout << "  [INFO] Distance between points set to: " << distBetweenPoints << endl;
                }
                else if ( !strcmp(argv[arg], "-step") )
                {
                    stepByStepExecution = true;
                    arg++;
                }
                else if ( !strcmp(argv[arg], "-clear") )
                {
                    clearAfterStep = true;
                    arg++;

                    cout << "  [INFO] Clearing scene after each step" << endl;
                }
                else if ( !strcmp(argv[arg], "-poses") )
                {
                    showPoses = true;
                    arg++;
                }
                else if ( !strcmp(argv[arg],"-limit") )
                {
                    N_limitOfObs = atoi(argv[arg+1]);
                    setNumberOfObs = true;
                    arg += 2;
                }
                else if ( !strcmp(argv[arg],"-lowerLimit") )
                {
                    N_lowerLimitOfObs = atoi(argv[arg+1]);
                    arg += 2;
                }
                else if ( !strcmp(argv[arg],"-zUpperLimit") )
                {
                    zUpperLimit = atof(argv[arg+1]);
                    arg += 2;
                    cout << "  [INFO] Upper limit set to: " << zUpperLimit << endl;
                }
                else
                {
                    cout << "  [Error] " << argv[arg] << " unknown paramter." << endl;
                    showUsageInformation();
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
}


//-----------------------------------------------------------
//
//                     visualizeScene
//
//-----------------------------------------------------------

void visualizeScene()
{
    if (!mrpt::system::fileExists(i_sceneFilename))
    {
        cout << "  [ERROR] Couldn't open scene dataset file " <<
                i_sceneFilename << endl;
        return;
    }

    // Get scene name
    vector<string> tokens;
    mrpt::system::tokenize(i_sceneFilename,"/",tokens);

    win3D = gui::CDisplayWindow3DPtr( new gui::CDisplayWindow3D() );
    win3D->setWindowTitle(format("Visualizing %s",tokens[tokens.size()-1].c_str()));

    scene = win3D->get3DSceneAndLock();

    if (scene->loadFromFile(i_sceneFilename))
        cout << "  [INFO] Scene " << i_sceneFilename << " loaded" << endl;
    else
        cout << "  [INFO] Error while loading scene " << i_sceneFilename << endl;

    win3D->unlockAccess3DScene();
    win3D->forceRepaint();
}


//-----------------------------------------------------------
//
//                       buildScene
//
//-----------------------------------------------------------

void buildScene()
{
    //
    //  Check the input rawlog file

    if (!mrpt::system::fileExists(i_rawlogFileName))
    {
        cout << "  [ERROR] Couldn't open rawlog dataset file " <<
                i_rawlogFileName << endl;
        return;
    }

    i_rawlog.open(i_rawlogFileName);

    cout << "  [INFO] Working with " << i_rawlogFileName << endl;

    if ( sensors_to_use.empty() )
        cout << "  [INFO] Visualizing observations from any sensor." << endl;
    else
    {
        cout << "  [INFO] Visualizing observations from: ";
        for ( size_t i_sensor = 0; i_sensor < sensors_to_use.size(); i_sensor++ )
            cout << sensors_to_use[i_sensor] << " ";
        cout << endl;
    }

    //
    // Set 3D window and visualization objects

    // Get rawlog name
    vector<string> tokens;
    mrpt::system::tokenize(i_rawlogFileName,"/",tokens);

    win3D = gui::CDisplayWindow3DPtr( new gui::CDisplayWindow3D() );
    win3D->setWindowTitle(format("Building reconstruction visualization of %s",tokens[tokens.size()-1].c_str()));

    win3D->resize(400,300);

    win3D->setCameraAzimuthDeg(140);
    win3D->setCameraElevationDeg(20);
    win3D->setCameraZoom(6.0);
    win3D->setCameraPointingToPoint(2.5,0,0);

    scene = win3D->get3DSceneAndLock();

    opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-7,7,-7,7,0,1);
    obj->setColor(0.7,0.7,0.7);
    obj->setLocation(0,0,0);
    scene->insert( obj );

    mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
    gl_points->setPointSize(pointSize);
    CColouredPointsMap colouredMap;
    colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
    colouredMap.insertionOptions.minDistBetweenLaserPoints = distBetweenPoints;
    //colouredMap.insertionOptions.fuseWithExisting = true;

    scene->insert( gl_points );

    win3D->unlockAccess3DScene();

    //
    // Set 2D window

    if ( visualize2Dposes )
    {
        win = gui::CDisplayWindowPlotsPtr( new gui::CDisplayWindowPlots("2D poses localization") );
        win->hold_on();
    }

    //
    // Let's go!

    mrpt::system::sleep(3000);

    size_t N_inserted_point_clouds = 0;

    //
    // Iterate over the obs into the rawlog and show them in the 3D/2D windows

    cout << "  [INFO] Showing observations from " << N_lowerLimitOfObs << " up to ";

    if ( N_limitOfObs == std::numeric_limits<size_t>::max() )
        cout << "the end"<< endl;
    else
        cout << N_limitOfObs << endl;


    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = N_lowerLimitOfObs;

    while (( obsIndex < N_limitOfObs ) &&
           ( CRawlog::getActionObservationPairOrObservation(i_rawlog,action,
                                                            observations,obs,obsIndex) ))
    {
        // Check that it is a 3D observation
        if ( !IS_CLASS(obs, CObservation3DRangeScan) )
            continue;

        // Using information from this sensor?
        if ( !sensors_to_use.empty()
             && find(sensors_to_use.begin(), sensors_to_use.end(),obs->sensorLabel)
             == sensors_to_use.end() )
            continue;

        if ( decimate && ( obsIndex%decimate) )
            continue;

        CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
        obs3D->load();

        if ( !obs3D->hasPoints3D )
            obs3D->project3DPointsFromDepthImage();

        if (equalizeRGBDHist)
            obs3D->intensityImage.equalizeHistInPlace();

        CPose3D pose;
        obs3D->getSensorPose( pose );
        cout << "    Sensor " << obs3D->sensorLabel << " index "
             << obsIndex << " pose: " << pose << endl;

        // Clear previous point clouds?
        if ( clearAfterStep )
        {
            size_t index = std::distance(sensors_to_use.begin(),
                                         find(sensors_to_use.begin(),
                                              sensors_to_use.end(),obs->sensorLabel));
            if ( !index )
            {
                colouredMap.clear();
                colouredMap.loadFromRangeScan( *obs3D );
            }
        }
        else
        {
            CColouredPointsMap localColouredMap;
            localColouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
            localColouredMap.insertionOptions.minDistBetweenLaserPoints = distBetweenPoints;
            localColouredMap.loadFromRangeScan( *obs3D );

            colouredMap.addFrom( localColouredMap );
        }

        size_t N_points = colouredMap.size();
        cout << "    Points in the map: " << N_points << endl;

        CVectorDouble coords,x,y;
        pose.getAsVector( coords );
        x.push_back( coords[0] );
        y.push_back( coords[1] );
        CPoint3D point((double)coords[0], (double)coords[1], (double)coords[2]);

        if ( visualize2Dposes )
        {
            // Plot sensor pose into the 2D window
            win->plot(x,y,"b.4");
        }

        // Plot point cloud into the 3D window
        scene = win3D->get3DSceneAndLock();

        gl_points->loadFromPointsMap( &colouredMap );

        // Remove points with a z higher than a given one
        for ( size_t i = 0; i < N_points; i++ )
            if ( gl_points->getPointf(i).z > zUpperLimit )
                gl_points->setPoint_fast(i,0,0,0);

        // Show spheres representing the observation poses?
        if ( showPoses )
        {
            mrpt::opengl::CSpherePtr sphere = mrpt::opengl::CSphere::Create(0.02);
            sphere->setPose(point);
            scene->insert( sphere );
        }

        N_inserted_point_clouds++;

        win3D->unlockAccess3DScene();
        win3D->repaint();

        // Step by step execution?
        if ( stepByStepExecution )
            win3D->waitForKey();
    }

    cout << "  [INFO] Number of points clouds in the scene: " << N_inserted_point_clouds << endl;
}


//-----------------------------------------------------------
//
//                     saveSceneToFile
//
//-----------------------------------------------------------

void saveSceneToFile()
{
    string sceneFile;
    sceneFile.assign( i_rawlogFileName.begin(), i_rawlogFileName.end()-7 );
    sceneFile += ".scene";

    cout << "  [INFO] Saving to scene file " << sceneFile;
    cout.flush();

    scene->saveToFile( sceneFile );
    cout << " ... done" << endl;

}


//-----------------------------------------------------------
//
//                        main
//
//-----------------------------------------------------------

int main(int argc, char* argv[])
{
    cout << endl << "-----------------------------------------------------" << endl;
    cout <<         "          Visualize reconstruction app.                    " << endl;
    cout <<         "            [Object Labeling Tookit]                 " << endl;
    cout <<         "-----------------------------------------------------" << endl << endl;

    try
    {
        //
        // Load configuration params

        int res = loadParameters(argc,argv);

        if ( res < 0 )
            return -1;
        if ( !res )
            return 0;

        //
        // Visualize scene

        if ( !i_rawlogFileName.empty() )
        {
            buildScene();

            //
            // Decide if save the scene or not

            if ( !win3D.null() )
            {
                cout << "  [INFO] Press 's' to save the scene or other key to end the program." << endl;
                while ( win3D->isOpen() && !win3D->keyHit() )
                    mrpt::system::sleep(10);

                int key = win3D->getPushedKey();

                //
                // Save the built scene to file
                if (( key == 's' ) || ( key == 'S'))
                    saveSceneToFile();
            }
        }
        else if ( !i_sceneFilename.empty() )
        {

            if ( saveAsPlainText )
            {
                string withoutExtension = i_sceneFilename.substr(0,i_sceneFilename.size()-6);

                OLT::CSaveAsPlainText editor;

                editor.setOption("output_file",withoutExtension+"_scene.txt");

                editor.setInputScene(i_sceneFilename);
                editor.process();
            }
            else
            {
                visualizeScene();

                while ( win3D->isOpen() && !win3D->keyHit() )
                    mrpt::system::sleep(10);
            }
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
