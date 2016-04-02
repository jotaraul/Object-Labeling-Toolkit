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

#ifndef _OLT_ANALYZER_
#define _OLT_ANALYZER_

#include "core.hpp"

#include "map"
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/C3DSScene.h>


namespace OLT
{
    class CAnalyzer
    {

    protected:

        mrpt::utils::CFileGZInputStream   m_iRawlog;
        mrpt::opengl::COpenGLScene        m_scene;
        std::map<std::string,double>      m_optionsD;
        std::map<std::string,std::string> m_optionsS;

    public:

        int setInputRawlog(const std::string &i_rawlogName)
        {
            if (!mrpt::system::fileExists(i_rawlogName))
            {
                std::cerr << "  [ERROR] A rawlog file with name " << i_rawlogName;
                std::cerr << " doesn't exist." << std::endl;
                return 0;
            }
            else
                std::cout << "  [INFO] Processing rawlog " << i_rawlogName << std::endl;

           m_iRawlog.open(i_rawlogName);

           return 1;
        }

        int setInputScene(const std::string &i_sceneName)
        {
            if (!mrpt::system::fileExists(i_sceneName))
            {
                std::cerr << "  [ERROR] A scene file with name " << i_sceneName;
                std::cerr << " doesn't exist." << std::endl;
                return 0;
            }
            else
                std::cout << "  [INFO] Processing scene " << i_sceneName << std::endl;

            if (!m_scene.loadFromFile(i_sceneName))
            {
                std::cerr << "  [ERROR] Can't open scene file " << i_sceneName << std::endl;
                return 0;
            }

            return 1;
        }

        void setOption( const std::string option, const double &value){
            m_optionsD[option] = value;
        }

        void setOption( const std::string option, const std::string value){
            m_optionsS[option] = value;
        }

        virtual int process(std::vector<double> &results) = 0;
    };


    class CDepthInfoAnalyzer : public CAnalyzer
    {
    public:
        int process(std::vector<double> &results);
    private:
        int processRawlog(std::vector<double> &results);
        int processScene(std::vector<double> &results);
    };
}


#endif
