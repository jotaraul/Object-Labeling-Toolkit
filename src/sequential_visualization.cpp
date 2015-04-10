/*---------------------------------------------------------------------------*
 |                             HOMe-toolkit                                  |
 |       A toolkit for working with the HOME Environment dataset (HOMe)      |
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
gui::CDisplayWindowPlots	win("ICP results");

int main(int argc, char* argv[])
{

    bool stepByStepExecution = false;
    bool clearAfterStep = false;

    // Set 3D window

    vector<string> sensors_to_use;
    sensors_to_use.push_back("RGBD_1");
    sensors_to_use.push_back("RGBD_2");
    sensors_to_use.push_back("RGBD_3");
    sensors_to_use.push_back("RGBD_4");

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

    win3D.unlockAccess3DScene();

    // Set 2D window

    win.hold_on();

    CRawlog rawlog;    
    string rawlogFile;

    if ( argc > 1 )
    {
        // Get rawlog file name
        rawlogFile = argv[1];        

        // Get optional paramteres
        if ( argc > 2 )
        {
            bool alreadyReset = false;

            size_t arg = 2;

            while ( arg < argc )
            {
                if ( !strcmp(argv[arg],"-sensor") )
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
                else if ( !strcmp(argv[arg], "-clear") )
                {
                    clearAfterStep = true;
                    arg++;
                }
                else
                {
                    cout << "[Error] " << argv[arg] << "unknown paramter" << endl;
                    return -1;
                }

            }
        }
    }
    else
    {
        cout << "Usage information. At least one expected argument: " << endl <<
                " \t (1) Rawlog file." << endl;
        cout << "Then, optional parameters:" << endl <<
                " \t -sensor <sensor_label> : Use obs. from this sensor (all used by default)." << endl <<
                " \t -step                  : Enable step by step execution." << endl <<
                " \t -clear                 : Clear the scene after a step." << endl;

        return -1;
    }

    rawlog.loadFromRawLogFile( rawlogFile );

    string sceneFile;
    sceneFile.assign( rawlogFile.begin(), rawlogFile.end()-7 );
    sceneFile += ".scene";
    size_t N_inserted_point_clouds = 0;

    vector<CRenderizablePtr> v_obsInserted;

    // Iterate over the obs into the rawlog and show them in the 3D/2D windows

    for ( size_t obs_index = 0; obs_index < rawlog.size(); obs_index++ )
    {
        CObservationPtr obs = rawlog.getAsObservation(obs_index);

        if ( find(sensors_to_use.begin(), sensors_to_use.end(),obs->sensorLabel) == sensors_to_use.end() )
            continue;

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

        CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
        obs3D->load();

        /*CMatrixFloat mat;
        obs3D->intensityImage.getAsMatrix(mat);

        cout << mat;*/

        //scene->removeObject()

        CPose3D pose;
        obs3D->getSensorPose( pose );
        cout << "Pose: " << obs_index << " " << pose << endl;

        CVectorDouble coords,x,y;
        pose.getAsVector( coords );
        x.push_back( coords[0] );
        y.push_back( coords[1] );
        CPoint3D point((double)coords[0], (double)coords[1], (double)coords[2]);
        win.plot(x,y,"b.4");

        mrpt::opengl::COpenGLScenePtr scene = win3D.get3DSceneAndLock();

        mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
        gl_points->setPointSize(3);

        CColouredPointsMap colouredMap;
        colouredMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
        colouredMap.loadFromRangeScan( *obs3D );
        size_t N_points = colouredMap.size();

        gl_points->loadFromPointsMap( &colouredMap );

        mrpt::opengl::CSpherePtr sphere = mrpt::opengl::CSphere::Create(0.02);
        sphere->setPose(point);

        for ( size_t i = 0; i < N_points; i++ )
            if ( gl_points->getPointf(i).z > 2 )
                gl_points->setPoint_fast(i,0,0,0);

        scene->insert( gl_points );
        scene->insert( sphere );

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

    cout << "[INFO] Saving to scene file " << sceneFile;
    scene->saveToFile( sceneFile );
    cout << " ... done" << endl;

    win.hold_off();
    win.waitForKey();

    mrpt::system::pause();

    return 0;
}
