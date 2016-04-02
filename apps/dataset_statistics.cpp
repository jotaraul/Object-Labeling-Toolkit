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

#include <mrpt/math.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils/CConfigFile.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt;
using namespace std;

//
//  Data types
//

vector<string> sensors_to_use;
CRawlog rawlog;
bool analyzeDepthInfo = false;

struct TConfiguration
{
    vector<string> labelsToConsider;    // Objects' labels to be considered
    vector<string> rawlogFiles;         // Rawlog files to be considered
    bool    instancesLabeled;           // Are we working with instances? e.g. knife_1
    bool    saveStatisticsToFile;       // Save compute statistics to a file
    string  statisticsOutputFileName;   // Where to store statistics
};

struct TStatistics
{
    map<string,size_t> labelOccurrences; // Map with <label,n_occurrences>
    map<string,size_t> labelNumOfPixels; // Map with <label,n_pixels>
    size_t             N_observations;   // Total number of observations processed

    TStatistics() : N_observations(0)
    {}
};

//
//  Global variables
//

TConfiguration  conf;
TStatistics     stats;

//-----------------------------------------------------------
//
//                  showUsageInformation
//
//-----------------------------------------------------------

void showUsageInformation()
{
    cout << "Usage information. At least one expected argument: " << endl <<
            " \t <conf_fil>       : Configuration file." << endl;
    cout << "Then, optional parameters:" << endl <<
            " \t -h      : This help." << endl <<
            " \t -sensor : Use obs from this sensor (all by default)." << endl <<
            " \t -analyzeDepthInfo : Analyze depth info." << endl << endl;

}


//-----------------------------------------------------------
//
//                    getInstanceLabel
//
//-----------------------------------------------------------

string getInstanceLabel(const string &instaceLabel )
{
    vector<string>::iterator it;

    for ( it = conf.labelsToConsider.begin();
          it != conf.labelsToConsider.end();
          it++ )
        if ( instaceLabel.find(*it)!=std::string::npos )
            return *it;

    string empty;
    return empty;
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

    config.read_vector("GENERAL","labelsToConsider",vector<string>(0),conf.labelsToConsider,true);
    conf.instancesLabeled = config.read_bool("GENERAL","instancesLabeled",0,true);
    conf.saveStatisticsToFile = config.read_bool("GENERAL","saveStatisticsToFile",0,true);
    conf.statisticsOutputFileName = config.read_string("GENERAL","statisticsOutputFileName","",conf.saveStatisticsToFile);

    vector<string> keys;
    config.getAllKeys("RAWLOGS",keys);

    for ( size_t i = 0; i < keys.size(); i++ )
    {
        string rawlogName = config.read_string("RAWLOGS",keys[i],"",true);
        conf.rawlogFiles.push_back(rawlogName);
    }

    cout << "[INFO] Configuration successfully loaded." << endl;

}


//-----------------------------------------------------------
//
//                     initializeStats
//
//-----------------------------------------------------------

void initializeStats()
{
    // Initialize stats
    vector<string>::iterator it;

    for ( it = conf.labelsToConsider.begin(); it != conf.labelsToConsider.end(); it++ )
    {
        stats.labelNumOfPixels[*it] = 0;
        stats.labelOccurrences[*it] = 0;
    }

    cout << "[INFO] Stats initialized." << endl;
}


//-----------------------------------------------------------
//
//                   updateStatsFromObs
//
//-----------------------------------------------------------

void updateStatsFromObs(CObservation3DRangeScanPtr obs,
                        size_t N_rows, size_t N_cols )
{
    // Increment the number of processed observations
    stats.N_observations++;

    std::map<uint32_t,std::string>::iterator it;

    for ( it = obs->pixelLabels->pixelLabelNames.begin();
          it != obs->pixelLabels->pixelLabelNames.end();
          it++ )
    {
        string label = ( conf.instancesLabeled ) ?
                    getInstanceLabel(it->second) :
                    label = it->second;

        //cout << "Cheking label " << label << endl;

        // Update occurrences
        if ( !label.empty() )
            stats.labelOccurrences[label] = stats.labelOccurrences[label] + 1;

        // Update num of pixels
        for ( size_t row = 0; row < N_rows; row++ )
            for ( size_t col = 0; col < N_cols; col++ )
                if ( obs->pixelLabels->checkLabel(row,col,it->first) )
                    stats.labelNumOfPixels[label] = stats.labelNumOfPixels[label] + 1;

    }

}

bool compareLabels( pair<string,size_t> l1, pair<string,size_t> l2 )
{
    return ( l1.second > l2.second );
}


//-----------------------------------------------------------
//
//                   saveStatsToFile
//
//-----------------------------------------------------------

void saveStatsToFile(ostream &statsFile, vector<string> &v_sensorsUsed )
{
    //
    // General information
    //

    statsFile << "Rawlogs processed: " << conf.rawlogFiles.size() << endl;
    for ( size_t rawlog_index = 0; rawlog_index < conf.rawlogFiles.size(); rawlog_index++ )
        statsFile << "- " << conf.rawlogFiles[rawlog_index] << endl;

    if ( v_sensorsUsed.empty() )
        statsFile << "Sensors used: all" << endl;
    else
    {
        statsFile << "Sensors used: " << v_sensorsUsed.size() << endl;
        for ( size_t sensor_index = 0; sensor_index < v_sensorsUsed.size(); sensor_index++ )
            statsFile << "- " << v_sensorsUsed[sensor_index] << endl;
    }

    statsFile << "Number of observations processed: ";
    statsFile << stats.N_observations << endl;

    //
    // Occurrences
    //

    std::vector<std::pair<string,size_t> > sorted_labels;

    //fill items
    map< string, size_t>::iterator it;

    for ( it = stats.labelOccurrences.begin();
          it != stats.labelOccurrences.end();
          it++ )
        sorted_labels.push_back( pair<string,size_t>(it->first,it->second) );

    //sort by value using std::sort
    std::sort(sorted_labels.begin(), sorted_labels.end(), compareLabels );

    statsFile << "Occurrence of object labels:" << endl;

    for ( size_t i = 0; i < sorted_labels.size(); i++ )
        statsFile << sorted_labels[i].first << " " << sorted_labels[i].second << endl;

    //
    // Pixels
    //

    sorted_labels.clear();

    for ( it = stats.labelNumOfPixels.begin();
          it != stats.labelNumOfPixels.end();
          it++ )
        sorted_labels.push_back( pair<string,size_t>(it->first,it->second) );

    //sort by value using std::sort
    std::sort(sorted_labels.begin(), sorted_labels.end(), compareLabels );

    statsFile << "Number of pixels per object labels:" << endl;

    for ( size_t i = 0; i < sorted_labels.size(); i++ )
        statsFile << sorted_labels[i].first << " " << sorted_labels[i].second << endl;

}

int loadParamters(int argc, char* argv[])
{
    if ( argc > 1 )
    {
        // Get configuration file name

        string configFile = argv[1];

        // Load configuration and initialize stats

        loadConfig(configFile);
        initializeStats();

        // Get optional paramteres
        if ( argc > 2 )
        {
            size_t arg = 2;

            while ( arg < argc )
            {
                if ( !strcmp(argv[arg],"-h") )
                {
                    showUsageInformation();
                    arg++;
                }
                else if ( !strcmp(argv[arg],"-sensor") )
                {
                    string sensor = argv[arg+1];
                    sensors_to_use.push_back(  sensor );
                    arg += 2;
                }
                else if ( !strcmp(argv[arg],"-analyzeDepthInfo") )
                {
                    analyzeDepthInfo = true;
                    arg ++;
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

}

void generateStats()
{
    const size_t N_rawlogs = conf.rawlogFiles.size();
    cout << "[INFO] a total of " << N_rawlogs << " to process." << endl;

    for ( size_t rawlog_index = 0; rawlog_index < N_rawlogs; rawlog_index++ )
    {
        rawlog.loadFromRawLogFile( conf.rawlogFiles[rawlog_index] );

        cout << "[INFO] Processing rawlog file : " << conf.rawlogFiles[rawlog_index];
        cout << " with " << rawlog.size() << " obs and index " << rawlog_index << endl;

        //
        // Iterate over the obs into the rawlog updating the stats
        //

        for ( size_t obs_index = 0; obs_index < rawlog.size(); obs_index++ )
        {
            CObservationPtr obs = rawlog.getAsObservation(obs_index);

            // Check that it is a 3D observation
            if ( !IS_CLASS(obs, CObservation3DRangeScan) )
                continue;

            // Check if the sensor is being used
            if ( !sensors_to_use.empty()
                 && find(sensors_to_use.begin(), sensors_to_use.end(),obs->sensorLabel)
                 == sensors_to_use.end() )
                continue;

            // Get obs pose
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load();

            size_t rows = obs3D->cameraParams.nrows;
            size_t cols = obs3D->cameraParams.ncols;

            // Update statistics with information from this observations
            updateStatsFromObs( obs3D, rows, cols );

        }
    }

    //
    //  Save stats to file
    //

    if ( conf.saveStatisticsToFile )
    {
        cout << "[INFO] Saving statistics to " << conf.statisticsOutputFileName << endl;
        ofstream statsFile(conf.statisticsOutputFileName.c_str(),ios::trunc);

        if ( statsFile.is_open() )
        {
            saveStatsToFile(statsFile,sensors_to_use);
            statsFile.close();
        }
        else
            cout << "[ERROR] Error when opening the file to save the statistics.";
    }

    saveStatsToFile(cout,sensors_to_use);

    cout << "[INFO] Done!" << endl;
}

void analyzeDepths()
{
    const size_t N_rawlogs = conf.rawlogFiles.size();
    cout << "[INFO] a total of " << N_rawlogs << " to process." << endl;

    double maxDepth = 0;
    double minDepth = std::numeric_limits<double>::max();
    double meanDepth = 0;

    for ( size_t rawlog_index = 0; rawlog_index < N_rawlogs; rawlog_index++ )
    {
        OLT::CDepthInfoAnalyzer analyzer;

        analyzer.setInputRawlog( conf.rawlogFiles[rawlog_index] );

        vector<double> results;

        analyzer.process(results);

        if ( results[0] > maxDepth )
            maxDepth = results[0];

        if ( results[1] < minDepth )
            minDepth = results[1];

        meanDepth = results[2];

    }

    cout << "  [INFO] Results: " << endl;
    cout << "         Max depth  = " << maxDepth << endl;
    cout << "         Min depth  = " << minDepth << endl;
    cout << "         Mean depth = " << meanDepth / static_cast<double>(N_rawlogs) << endl;

    cout << "[INFO] Done!" << endl;
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
        // Useful variables

        loadParamters(argc, argv);

        if ( sensors_to_use.empty() )
            cout << "[INFO] Considering observations from any sensor." << endl;
        else
        {
            cout << "[INFO] Considering observations from: ";
            for ( size_t i_sensor = 0; i_sensor < sensors_to_use.size(); i_sensor++ )
                cout << sensors_to_use[i_sensor] << " ";
            cout << endl;
        }

        if ( analyzeDepthInfo )
            analyzeDepths();
        else
            generateStats();

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
