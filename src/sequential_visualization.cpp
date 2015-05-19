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

#include <mrpt/poses/CPoint2D.h>

using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt;
using namespace std;

mrpt::gui::CDisplayWindow3D  win3D;
gui::CDisplayWindowPlots	win("Observations 2D pose");


//-----------------------------------------------------------
//
//                   showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "Usage information. At least one expected argument: " << endl <<
            " \t (1) Rawlog file." << endl;

    cout << "Then, optional parameters:" << endl <<
            " \t -sensor <sensor_label> : Use obs. from this sensor (all used by default)." << endl <<
            " \t -step                  : Enable step by step execution." << endl <<
            " \t -clear                 : Clear the scene after a step." << endl <<
            " \t -poses                 : Show spheres in the scene representing observation poses." << endl <<
            " \t -zUpperLimit           : Remove points with a height higher than this paramter." << endl <<
            " \t -limit                 : Sets a limit to the number of obs to process." <<
            " \t -lowerLimit            : Sets a lower limit to the number of obs to process." << endl;
}




//-----------------------------------------------------------
//
//                        main
//
//-----------------------------------------------------------

int main(int argc, char* argv[])
{
    try
    {
        //
        //  Useful variables
        //

        CRawlog         rawlog; // Input rawlog
        string          rawlogFile; // Rawlog name
        vector<string>  sensors_to_use; // Visualize data from this sensors. Empty=all

        bool stepByStepExecution = false;
        bool clearAfterStep = false;
        bool showPoses = false;
        double zUpperLimit = std::numeric_limits<double>::max();

        // Set 3D window

        win3D.setWindowTitle("Sequential visualization");

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

        bool setNumberOfObs = false;
        size_t N_limitOfObs;
        size_t N_lowerLimitOfObs = 0;

        win3D.unlockAccess3DScene();

        // Set 2D window

        win.hold_on();

        //
        // Load configuration params
        //

        if ( argc > 1 )
        {
            // Get rawlog file name
            rawlogFile = argv[1];

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
                    else if ( !strcmp(argv[arg], "-step") )
                    {
                        stepByStepExecution = true;
                        arg++;
                    }
                    else if ( !strcmp(argv[arg], "-clear") )
                    {
                        clearAfterStep = true;
                        arg++;
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
                        zUpperLimit = atoi(argv[arg+1]);
                        arg += 2;
                    }
                    else
                    {
                        cout << "[Error] " << argv[arg] << " unknown paramter." << endl;
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

        if ( sensors_to_use.empty() )
            cout << "[INFO] Considering observations from any sensor." << endl;
        else
        {
            cout << "[INFO] Considering observations from: ";
            for ( size_t i_sensor = 0; i_sensor < sensors_to_use.size(); i_sensor++ )
                cout << sensors_to_use[i_sensor] << " ";
            cout << endl;
        }

        //
        //  Load rawlog file
        //

        cout << "[INFO] Loading rawlog file: " << rawlogFile << endl;

        rawlog.loadFromRawLogFile( rawlogFile );

        string sceneFile;
        sceneFile.assign( rawlogFile.begin(), rawlogFile.end()-7 );
        sceneFile += ".scene";
        size_t N_inserted_point_clouds = 0;

        vector<CRenderizablePtr> v_obsInserted;

        if ( !setNumberOfObs )
            N_limitOfObs = rawlog.size();

        //
        // Iterate over the obs into the rawlog and show them in the 3D/2D windows
        //

        cout << "[INFO] Showing observations from " << N_lowerLimitOfObs;
        cout << " up to " << N_limitOfObs << endl;

        for ( size_t obs_index = N_lowerLimitOfObs; obs_index < N_limitOfObs; obs_index++ )
        {
            CObservationPtr obs = rawlog.getAsObservation(obs_index);

            // Check that it is a 3D observation
            if ( !IS_CLASS(obs, CObservation3DRangeScan) )
                continue;

            // Using information from this sensor?
            if ( !sensors_to_use.empty()
                 && find(sensors_to_use.begin(), sensors_to_use.end(),obs->sensorLabel)
                 == sensors_to_use.end() )
                continue;

            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            CPose3D pose;
            obs3D->getSensorPose( pose );
            cout << "Pose: " << obs_index << " " << pose << endl;

            // Clear previous point clouds?
            if ( clearAfterStep )
            {
                size_t index = std::distance(sensors_to_use.begin(),
                                             find(sensors_to_use.begin(),
                                                  sensors_to_use.end(),obs->sensorLabel));
                if ( !index )
                {
                    for ( size_t i = 0; i < v_obsInserted.size(); i++ )
                        scene->removeObject( v_obsInserted[i] );

                    v_obsInserted.clear();
                }

            }

            // Plot sensor pose into the 2D window
            CVectorDouble coords,x,y;
            pose.getAsVector( coords );
            x.push_back( coords[0] );
            y.push_back( coords[1] );
            CPoint3D point((double)coords[0], (double)coords[1], (double)coords[2]);
            win.plot(x,y,"b.4");

            // Plot point cloud into the 3D window
            mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

            mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
            gl_points->setPointSize(3);

            CColouredPointsMap colouredMap;
            colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
            colouredMap.insertionOptions.minDistBetweenLaserPoints = 0.01;
            colouredMap.loadFromRangeScan( *obs3D );
            size_t N_points = colouredMap.size();

            gl_points->loadFromPointsMap( &colouredMap );

            // Remove points with a z higher than a given one
            for ( size_t i = 0; i < N_points; i++ )
                if ( gl_points->getPointf(i).z > zUpperLimit )
                    gl_points->setPoint_fast(i,0,0,0);

            scene->insert( gl_points );

            // Show spheres representing the observation poses?
            if ( showPoses )
            {
                mrpt::opengl::CSpherePtr sphere = mrpt::opengl::CSphere::Create(0.02);
                sphere->setPose(point);
                scene->insert( sphere );
            }

            if ( clearAfterStep )
                v_obsInserted.push_back( gl_points );

            N_inserted_point_clouds++;

            win3D.unlockAccess3DScene();
            win3D.repaint();

            // Step by step execution?
            if ( stepByStepExecution )
                win3D.waitForKey();
        }

        cout << "[INFO] Number of points clouds in the scene: " << N_inserted_point_clouds << endl;

        //
        // Save the resultant scene to file
        //

        cout << "[INFO] Saving to scene file " << sceneFile;
        scene->saveToFile( sceneFile );
        cout << " ... done" << endl;

        mrpt::system::pause();

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
