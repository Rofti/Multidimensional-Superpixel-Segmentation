/**
 * Copyright (c) 2020, Marven von Domarus
 * Contact: marven.von.domarus@rwth-aachen.de
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

#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>
#include "io_util.h"
#include "superpixel_tools.h"
#include "visualization.h"
#include "kmc_lib.h"

/** \brief Command line tool for running KMC (K-Medoids Compression).
 * Usage:
 * \code{sh}
 *   $ ../bin/kmc_cli --help
 *   Allowed options:
 *     -h [ --help ]                   produce help message
 *     -d [--dimension]                number of subimages, must be greater than 1
 *     -i [ --input ] arg              the folder to process
 *     -s [ --superpixels ] arg (=400) number of superpixels
 *     -o [ --csv ] arg                save segmentation as CSV file
 *     -v [ --vis ] arg                visualize contours
 *     -b [ --vbase ] arg              base image for visualization of contours
 *     -x [ --prefix ] arg             output file prefix
 *     -w [ --wordy ]                  verbose/wordy/debug
 * \endcode
 * \author Marven von Domarus
 */
int main(int argc, const char** argv) {
    
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("dimensions,d", boost::program_options::value<int>()->default_value(1), "number of subimages, must be greater than 1")
        ("input,i", boost::program_options::value<std::string>(), "the folder to process")
        ("clusters,k", boost::program_options::value<int>()->default_value(400), "number of clusters/superpixels")
        ("csv,o", boost::program_options::value<std::string>()->default_value(""), "save segmentation as CSV file")
        ("vis,v", boost::program_options::value<std::string>()->default_value(""), "visualize contours")
        ("vbase,b", boost::program_options::value<std::string>()->default_value(""), "base image for visualization of contours")
        ("prefix,x", boost::program_options::value<std::string>()->default_value(""), "output file prefix")
        ("wordy,w", "verbose/wordy/debug");

    boost::program_options::positional_options_description positionals;
    positionals.add("input", 1);
    
    boost::program_options::variables_map parameters;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(positionals).run(), parameters);
    boost::program_options::notify(parameters);

    if (parameters.find("help") != parameters.end()) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    boost::filesystem::path output_dir(parameters["csv"].as<std::string>());
    if (!output_dir.empty()) {
        if (!boost::filesystem::is_directory(output_dir)) {
            boost::filesystem::create_directories(output_dir);
        }
    }
    
    boost::filesystem::path vis_dir(parameters["vis"].as<std::string>());
    if (!vis_dir.empty()) {
        if (!boost::filesystem::is_directory(vis_dir)) {
            boost::filesystem::create_directories(vis_dir);
        }
    }
    
    boost::filesystem::path input_dir(parameters["input"].as<std::string>());
    if (!boost::filesystem::is_directory(input_dir)) {
        std::cout << "Image directory not found ..." << std::endl;
        return 1;
    }
    
    std::string prefix = parameters["prefix"].as<std::string>();
    
    bool wordy = false;
    if (parameters.find("wordy") != parameters.end()) {
        wordy = true;
    }

    cv::Mat vis_base_img ;
    boost::filesystem::path imagePath(parameters["vbase"].as<std::string>());
    if (!vis_dir.empty() && !boost::filesystem::is_regular_file(imagePath)) {
        std::cout << "Could not find vbase image " << imagePath.string() << std::endl;
        return 1;
    }
    else{
         vis_base_img = cv::imread(imagePath.string());
    }


    int image_dimension= parameters["dimensions"].as<int>();
    if(image_dimension<=1){
        std::cout<< "Image dimension (number of segmentations to compress) must be greater than 1."<< std::endl;
        return 1;
    }
    int superpixels = parameters["clusters"].as<int>();

    std::multimap<std::string, boost::filesystem::path> segmentations_paths;
    std::vector<std::string> extensions;
    extensions.push_back(".csv");
    IOUtil::readDirectory(input_dir, extensions, segmentations_paths);

    std::vector<cv::Mat> segmentations;

    float total = 0;

    int rows= 0;
    int cols=0;

    for (std::multimap<std::string, boost::filesystem::path>::iterator it = segmentations_paths.begin();
         it != segmentations_paths.end(); ++it) {

        cv::Mat cur_seg;
        IOUtil::readMatCSVInt(it->first, cur_seg);
        segmentations.push_back(cur_seg);

        if(segmentations.size()%image_dimension!=0){

            if(segmentations.size()%image_dimension==1){

                rows=segmentations.back().rows;
                cols=segmentations.back().cols;

            }else if(rows!=segmentations.back().rows || cols!=segmentations.back().cols){
                std::cout << "Segmentation with differing rows/cols found ..." << std::endl;
                return 1;
            }

        }
        else{

            boost::timer::cpu_timer timer;
            cv::Mat labels;
            KMC kmc;
            kmc.compressSuperpixels(segmentations, superpixels, labels);
            float elapsed = boost::chrono::seconds(timer.elapsed().user).count();
            total += elapsed;

            int unconnected_components = SuperpixelTools::relabelConnectedSuperpixels(labels);

            if (wordy) {
                std::cout << SuperpixelTools::countSuperpixels(labels) << " superpixels for " << it->first
                          << " (" << unconnected_components << " not connected; "
                          << elapsed <<")." << std::endl;
            }

            if (!output_dir.empty()) {
                boost::filesystem::path csv_file(output_dir
                                                 / boost::filesystem::path(prefix + it->second.stem().string() + ".csv"));
                IOUtil::writeMatCSV<int>(csv_file, labels);
            }

            if (!vis_dir.empty()) {
                for(int d=0;d<image_dimension;d++){
                    boost::filesystem::path contours_file(vis_dir
                                                          / boost::filesystem::path(prefix + it->second.stem().string() +"_dim"+std::to_string(d)+ ".png"));
                    cv::Mat image_contours;
                    Visualization::drawContours(vis_base_img, labels, image_contours);
                    cv::imwrite(contours_file.string(), image_contours);
                }
            }

            segmentations.clear();
        }

    }

    if (wordy) {
        std::cout << "Average time: " << (double)total / (segmentations_paths.size()/(double)image_dimension) << "." << std::endl;
    }
    
    if (!output_dir.empty()) {
        std::ofstream runtime_file(output_dir.string() + "/" + prefix + "runtime.txt", 
                std::ofstream::out | std::ofstream::app);
        
        runtime_file << (double)total / (segmentations_paths.size()/(double)image_dimension)  << "\n";
        runtime_file.close();
    }
    
    return 0;
}
