/**
 * Copyright (c) 2016, David Stutz
 * Contact: david.stutz@rwth-aachen.de, davidstutz.de
 *
 * Copyright (c) 2020, Marven von Domarus
 * Contact: marven.von.domarus@rwth-aachen.de
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/timer.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

#include "io_util.h"
#include "parameter_optimization_tool.h"

// Dirty but simple ...
std::string FAIR = "-f ";
std::string RELATIVE_PATH = ".";


////////////////////////////////////////////////////////////////////////////////
// ERGC
////////////////////////////////////////////////////////////////////////////////

/** \brief Connector for parameter optimization of ERGC.
 * 
 * \param[in] img_directory
 * \param[in] gt_directory
 * \param[in] base_directory
 * \param[in] superpixels
 */
void connector_ERGC(boost::filesystem::path img_directory, 
        boost::filesystem::path gt_directory, boost::filesystem::path base_directory,
        std::vector<int> superpixels, int dimensions) {
    
    for (unsigned int k = 0; k < superpixels.size(); k++) {
        ParameterOptimizationTool tool(img_directory, gt_directory,
                base_directory / boost::filesystem::path(std::to_string(superpixels[k])),
                RELATIVE_PATH + "/ergc_cli", FAIR);

        tool.addIntegerParameter("superpixels", "--superpixels", std::vector<int>{superpixels[k]});
        tool.addIntegerParameter("perturb-seeds", "--perturb-seeds", std::vector<int>{/*0, */1}); // 2
        tool.addIntegerParameter("color-space", "--color-space", std::vector<int>{/*0,*/ 1}); // 2
        tool.addIntegerParameter("compacity", "--compacity", std::vector<int>{/*0,*/ 1/*, 2, 5*/}); // 6
        tool.addIntegerParameter("dimensions", "--dimensions", std::vector<int>{dimensions});
        tool.addIntegerParameter("p-norm", "--p-norm", std::vector<int>{1, 2, 3, 4, 5, 7, 10, 20, 25});

        tool.optimize();
    }
}

////////////////////////////////////////////////////////////////////////////////
// ERS
////////////////////////////////////////////////////////////////////////////////

/** \brief Connector for parameter optimization of ERS.
 * 
 * \param[in] img_directory
 * \param[in] gt_directory
 * \param[in] base_directory
 * \param[in] superpixels
 */
void connector_ERS(boost::filesystem::path img_directory, 
        boost::filesystem::path gt_directory, boost::filesystem::path base_directory,
        std::vector<int> superpixels, int dimensions) {
    
    for (unsigned int k = 0; k < superpixels.size(); k++) {
        ParameterOptimizationTool tool(img_directory, gt_directory,
                base_directory / boost::filesystem::path(std::to_string(superpixels[k])),
                RELATIVE_PATH + "/ers_cli", "");

        tool.addIntegerParameter("superpixels", "--superpixels", std::vector<int>{superpixels[k]});
        tool.addFloatParameter("lambda", "--lambda", std::vector<float>{0.1f/*, 0.5f, 1.0f, 2.5f, 5.0f, 10.0f, 20.0f*/}); // 6
        tool.addFloatParameter("sigma", "--sigma", std::vector<float>{/*0.1f, 0.5f, 1.0f, 2.5f, 5.0f, */10.0f/*, 20.0f*/}); // 6
        tool.addIntegerParameter("dimensions", "--dimensions", std::vector<int>{dimensions});
        tool.addIntegerParameter("pnorm", "--pnorm", std::vector<int>{1, 2, 3, 4, 5/*, 7, 10, 20, 25*/});
        tool.optimize();
    }
}

////////////////////////////////////////////////////////////////////////////////
// ETPS
////////////////////////////////////////////////////////////////////////////////

/** \brief Connector for parameter optimization of ETPS.
 * 
 * \param[in] img_directory
 * \param[in] gt_directory
 * \param[in] base_directory
 * \param[in] superpixels
 */
void connector_ETPS(boost::filesystem::path img_directory, 
        boost::filesystem::path gt_directory, boost::filesystem::path base_directory,
        std::vector<int> superpixels, int dimensions) {
    
    for (unsigned int k = 0; k < superpixels.size(); k++) {
        ParameterOptimizationTool tool(img_directory, gt_directory,
                base_directory / boost::filesystem::path(std::to_string(superpixels[k])),
                RELATIVE_PATH + "/etps_cli", "");

        tool.addIntegerParameter("superpixels", "--superpixels", std::vector<int>{superpixels[k]});
        tool.addFloatParameter("regularization-weight", "--regularization-weight", std::vector<float>{/*0.01f, 0.05f, 0.1f, */0.2f/*, 0.5f, 5.f, 10.f*/}); // 7
        tool.addFloatParameter("length-weight", "--length-weight", std::vector<float>{/*0.1f, 1.f, 5.f,*/ 10.f/*,20.f*/}); // 3
        tool.addFloatParameter("size-weight", "--size-weight", std::vector<float>{1.f}); // 1
        tool.addIntegerParameter("iterations", "--iterations", std::vector<int>{/*1, 5, */10/*, 25*/}); // 4
        tool.addIntegerParameter("dimensions", "--dimensions", std::vector<int>{dimensions});
        tool.addIntegerParameter("p-norm", "--p-norm", std::vector<int>{1, 2, 3, 4, 5, 7, 10, 20, 25});

        tool.optimize();
    }
}


////////////////////////////////////////////////////////////////////////////////
// SLIC
////////////////////////////////////////////////////////////////////////////////

/** \brief Connector for parameter optimization of SLIC.
 *
 * \param[in] img_directory
 * \param[in] gt_directory
 * \param[in] base_directory
 * \param[in] superpixels
 */
void connector_SLIC(boost::filesystem::path img_directory, 
        boost::filesystem::path gt_directory, boost::filesystem::path base_directory,
        std::vector<int> superpixels, int dimensions) {
    
    for (unsigned int k = 0; k < superpixels.size(); k++) {
        ParameterOptimizationTool tool(img_directory, gt_directory,
                base_directory / boost::filesystem::path(std::to_string(superpixels[k])),
                RELATIVE_PATH + "/slic_cli", "");

        tool.addIntegerParameter("superpixels", "--superpixels", std::vector<int>{superpixels[k]});
        tool.addFloatParameter("compactness", "--compactness", std::vector<float>{/*1.0f, 5.0f, 10.0f, 20.0f, 40.0f, 80.0f, */160.0f/*, 200.0f, 240.0f*/}); // 9
        tool.addIntegerParameter("iterations", "--iterations", std::vector<int>{/*1, 5,*/ 5/*, 50*/}); // 5
        tool.addIntegerParameter("perturb-seeds", "--perturb-seeds", std::vector<int>{/*0, */1}); // 2
        tool.addIntegerParameter("color-space", "--color-space", std::vector<int>{0/*, 1*/}); // 2
        tool.addIntegerParameter("dimensions", "--dimensions", std::vector<int>{dimensions});
        tool.addIntegerParameter("p-norm", "--p-norm", std::vector<int>{/*1, 2, 3, 4, 5,*/ 8});

        tool.optimize();
    }
}
/** \brief Connector for running KMC.
 *
 * \param[in] img_directory
 * \param[in] gt_directory
 * \param[in] base_directory
 * \param[in] superpixels
 */
void connector_KMC(boost::filesystem::path img_directory,
        boost::filesystem::path gt_directory, boost::filesystem::path base_directory,
        std::vector<int> superpixels, int dimensions) {

    for (unsigned int k = 0; k < superpixels.size(); k++) {
        ParameterOptimizationTool tool(img_directory, gt_directory,
                base_directory / boost::filesystem::path(std::to_string(superpixels[k])),
                RELATIVE_PATH + "/kmc_cli", "");

        tool.addIntegerParameter("clusters", "--clusters", std::vector<int>{superpixels[k]});
        tool.addIntegerParameter("dimensions", "--dimensions", std::vector<int>{dimensions});
        tool.optimize();
    }
}

/** \brief Optimize parameters for the different algorithms. 
 * Usage:
 * \code{sh}
 *   $ ../bin/eval_parameter_optimization_cli --help
 *   Allowed options:
 *     --img-directory arg                   image directory
 *     --gt-directory arg                    ground truth directory
 *     --base-directory arg                  base directory
 *     --algorithm arg                       algorithm to optimize: reseeds, 
 *     -d [ --dimensions ] arg (=1)          image dimensions, 1 corresponds to traditional algorithm with one image ,  > 1 to the multi-dim version
 *     --depth-directory arg                 depth directory
 *     --intrinsics-directory arg            intrinsics directory
 *     --not-fair                            do not use fair parameters
 *     --help                                produce help message
 * \endcode
 * \author David Stutz
 */
int main(int argc, const char** argv) {
    
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("img-directory", boost::program_options::value<std::string>(), "image directory")
        ("gt-directory", boost::program_options::value<std::string>(), "ground truth directory")
        ("base-directory", boost::program_options::value<std::string>(), "base directory")
        ("algorithm", boost::program_options::value<std::string>(), "algorithm to optimize: reseeds, ")
        ("depth-directory", boost::program_options::value<std::string>()->default_value(""), "depth directory")
        ("intrinsics-directory", boost::program_options::value<std::string>()->default_value(""), "intrinsics directory")
        ("not-fair", "do not use fair parameters")
        ("help", "produce help message")
        ("dimensions,d", boost::program_options::value<int>()->default_value(1), "image dimensions. E.g. 3 would mean one image consists of 3 subimages");

    boost::program_options::positional_options_description positionals;
    positionals.add("algorithm", 1);
    positionals.add("img-directory", 1);
    positionals.add("gt-directory", 1);
    positionals.add("base-directory", 1);
    
    boost::program_options::variables_map parameters;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(positionals).run(), parameters);
    boost::program_options::notify(parameters);

    if (parameters.find("help") != parameters.end()) {
        std::cout << desc << std::endl;
        return 1;
    }

    int dimensions = parameters["dimensions"].as<int>();

    boost::filesystem::path img_directory(parameters["img-directory"].as<std::string>());
    if (!boost::filesystem::is_directory(img_directory)) {
        std::cout << "Image directory does not exist." << std::endl;
        return 1;
    }
    
    boost::filesystem::path gt_directory(parameters["gt-directory"].as<std::string>());
    if (!boost::filesystem::is_directory(gt_directory)) {
        std::cout << "Ground truth directory does not exist." << std::endl;
        return 1;
    }
    
    boost::filesystem::path base_directory(parameters["base-directory"].as<std::string>());
    if (!boost::filesystem::is_directory(base_directory)) {
        boost::filesystem::create_directories(base_directory);
    }
    
    boost::filesystem::path depth_directory(parameters["depth-directory"].as<std::string>());
    if (!depth_directory.empty()) {
        if (!boost::filesystem::is_directory(depth_directory)) {
            std::cout << "Depth directory does not exist." << std::endl;
            return 1;
        }
    }
    
    boost::filesystem::path intrinsics_directory(parameters["intrinsics-directory"].as<std::string>());
    if (!intrinsics_directory.empty()) {
        if (!boost::filesystem::is_directory(intrinsics_directory)) {
            std::cout << "Intrinsics directory does not exist." << std::endl;
            return 1;
        }
    }
    
    if (parameters.find("not-fair") != parameters.end()) {
        FAIR = "";
    }
        
    std::string algorithm = parameters["algorithm"].as<std::string>();
    std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(), 
            ::tolower);
    
    std::vector<int> superpixels = {/*25, 50, 75, */100/*, 200, 300, 400, 500, 600, 700*/};

    if (algorithm == "ergc") {
        connector_ERGC(img_directory, gt_directory, base_directory, superpixels, dimensions);
    }
    else if (algorithm == "ers") {
        connector_ERS(img_directory, gt_directory, base_directory, superpixels, dimensions);
    }
    else if (algorithm == "etps") {
        connector_ETPS(img_directory, gt_directory, base_directory, superpixels, dimensions);
    }
    else if (algorithm == "slic") {
        connector_SLIC(img_directory, gt_directory, base_directory, superpixels, dimensions);
    }
    else if (algorithm == "kmc") {
        connector_KMC(img_directory, gt_directory, base_directory, superpixels, dimensions);
    }
    else {
        std::cout << "Invalid algorithm." << std::endl;
        return 1;
    }
    
    return 0;
}
