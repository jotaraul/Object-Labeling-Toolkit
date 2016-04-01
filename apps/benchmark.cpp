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
#include <mrpt/system/filesystem.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils.h>
#include <mrpt/system/os.h>

#include <numeric>
#include <iostream>
#include <fstream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::obs;

using namespace std;

CFileGZInputStream gtRawlog;    // Rawlog with ground truth information
CFileGZInputStream labeledRawlog; // Labeled rawlog to be compared

string gtRawlogFilename;
string labeledRawlogFilename;

//-----------------------------------------------------------
//
//                      loadConfig
//
//-----------------------------------------------------------

void loadConfig()
{

}


//-----------------------------------------------------------
//
//                    showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "Usage information. At least two expected arguments: " << endl <<
            "    (1) Rawlog file with ground truth information." << endl <<
            "    (2) Rawlog file with annotated labels." << endl;
   cout << "Then, optional parameters:" << endl <<
            "    -h                         : Shows this help." << endl;

}


//-----------------------------------------------------------
//
//                     loadParameters
//
//-----------------------------------------------------------

int loadParameters(int argc, char* argv[])
{
    if ( argc >= 3 )
    {
        gtRawlogFilename = argv[1];
        labeledRawlogFilename = argv[2];

        for ( size_t arg = 3; arg < argc; arg++ )
        {
            if ( !strcmp(argv[arg],"-h") )
            {
                showUsageInformation();
                return -1;
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

        return -1;
    }

    return 0;
}


//-----------------------------------------------------------
//
//                        benchmark
//
//-----------------------------------------------------------

void benchmark()
{
    //
    // Check rawlogs
    //

    if (!mrpt::system::fileExists(gtRawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << gtRawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    gtRawlog.open(gtRawlogFilename);

    if (!mrpt::system::fileExists(labeledRawlogFilename))
    {
        cerr << "  [ERROR] A rawlog file with name " << labeledRawlogFilename;
        cerr << " doesn't exist." << endl;
        return;
    }

    labeledRawlog.open(labeledRawlogFilename);

    cout << "  [INFO] Working with ground truth rawlog " << gtRawlogFilename << endl;
    cout << "         and labeled rawlog " << labeledRawlogFilename << endl;

    // TODO: Add tests to ensure that they are the same rawlog sequence

    vector<double> v_success;

    // Get pairs of observations and compare their labels

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr gtObs,labeledObs;
    size_t gtObsIndex = 0, labeledObsIndex = 0;

    cout << "    Process: ";
    cout.flush();

    while ( ( CRawlog::getActionObservationPairOrObservation(gtRawlog,action,observations,gtObs,gtObsIndex) )&&
            ( CRawlog::getActionObservationPairOrObservation(labeledRawlog,action,observations,labeledObs,labeledObsIndex) ) )
    {
        // TODO: Check that the obss are 3D scans
        CObservation3DRangeScanPtr gt3DObs = CObservation3DRangeScanPtr(gtObs);
        CObservation3DRangeScanPtr labeled3DObs = CObservation3DRangeScanPtr(labeledObs);

        if ( !(gtObsIndex % 200) )
        {
            if ( !(gtObsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        // Check that both observations have labels
        if ( !gt3DObs->hasPixelLabels() || !labeled3DObs->hasPixelLabels() )
             continue;

        std::map<uint32_t,std::string>::iterator labelsIt;
        size_t labelsAppearing = 0;

        for ( labelsIt = gt3DObs->pixelLabels->pixelLabelNames.begin();
              labelsIt != gt3DObs->pixelLabels->pixelLabelNames.end();
              labelsIt++ )
        {
            // Get label name from the ground truth obs
            string label = labelsIt->second;

            // Check if it appears in the labeled obs
            if ( labeled3DObs->pixelLabels->checkLabelNameExistence(label) >= 0 )
                labelsAppearing++;
        }

        size_t N_labelsGt = gt3DObs->pixelLabels->pixelLabelNames.size();
        size_t N_labelsLabeled = labeled3DObs->pixelLabels->pixelLabelNames.size();

        size_t maxNumOfLabels = ( (N_labelsGt > N_labelsLabeled) ? N_labelsGt : N_labelsLabeled );

        if (!maxNumOfLabels) // Are there labels?
            continue;

        double success = labelsAppearing / (double)maxNumOfLabels;
        v_success.push_back(success);
    }

    double sumSuccess = std::accumulate(v_success.begin(), v_success.end(), 0.0);
    double meanSuccess = sumSuccess / (double)v_success.size();

    cout << endl;
    cout << "  [INFO] Results: " << endl;
    cout << "           - Mean success: " << meanSuccess*100 << "%";
    cout << endl << endl;
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

        cout << endl << "-----------------------------------------------------" << endl;
        cout <<         "                Benchmark app.                       " << endl;
        cout <<         "           [Object Labeling Tookit]                  " << endl;
        cout <<         "-----------------------------------------------------" << endl << endl;

        //
        // Load paramteres

        int res = loadParameters(argc,argv);

        if ( res < 0 )
            return -1;

        //
        // Compare datasets

        benchmark();


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



