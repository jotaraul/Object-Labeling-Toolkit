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

#include "CAnalyzer.hpp"
#include <cmath>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CText3D.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/utils.h>
#include <mrpt/poses/CPose3D.h>

#include <opencv2/opencv.hpp>


using namespace OLT;
using namespace std;

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::poses;


int CDepthInfoAnalyzer::process(vector<double> &results)
{
    if ( m_iRawlog.is_open() )
        return processRawlog(results);
    else
        return processScene(results);
}

int CDepthInfoAnalyzer::processRawlog(vector<double> &results)
{
    cout << "  [INFO] Analyzing depth information." << endl;

    double maxValue = 0;
    double minValue = std::numeric_limits<double>::max();
    double meanValue=0;
    double N_obs = 0;

    //
    // Process rawlog
    //

    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr obs;
    size_t obsIndex = 0;

    cout << "    Process: ";
    cout.flush();

    while ( CRawlog::getActionObservationPairOrObservation(m_iRawlog,action,observations,obs,obsIndex) )
    {
        // Show progress as dots

        if ( !(obsIndex % 200) )
        {
            if ( !(obsIndex % 1000) ) cout << "+ "; else cout << ". ";
            cout.flush();
        }

        if (IS_CLASS(obs, CObservation3DRangeScan))
        {
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();         

            if ( obs3D->hasRangeImage )
            {
                double max = obs3D->rangeImage.maxCoeff();
                double min = obs3D->rangeImage.minCoeff();
                double mean = obs3D->rangeImage.mean();

                if ( max > maxValue )
                    maxValue = max;
                if ( minValue < min )
                    min = minValue;

                meanValue += mean;
                N_obs++;
            }
        }
    }

    results.push_back( maxValue );
    results.push_back( minValue );
    results.push_back( meanValue / N_obs );

    cout << endl << "  [INFO] Done!" << endl << endl;

    return 1;
}

int CDepthInfoAnalyzer::processScene(vector<double> &results)
{
    // TODO (under request)
    return 1;
}

