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

#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>
#include "ers_opencv.h"
#include "io_util.h"
#include "superpixel_tools.h"
#include "visualization.h"

/** \brief Command line tool for running ERS.
 * Usage:
 * \code{sh}
 *   $ ../bin/ers_cli --help
 *   Allowed options:
 *     -h [ --help ]                   produce help message
 *     -d [ --dimensions ] arg (=1)    number of subimages, 1 for traditional ers ,  > 1 for multi-dim ers
 *     -i [ --input ] arg              the folder to process
 *     -l [ --lambda ] arg (=0.5)      lambda
 *     -g [ --sigma ] arg (=5)         sigma
 *     -f [ --eight-connected ]        use 8-connected
 *     -s [ --superpixels ] arg (=400) number of superpixels
 *     -o [ --csv ] arg                save segmentation as CSV file
 *     -v [ --vis ] arg                visualize contours
 *     -x [ --prefix ] arg             output file prefix
 *     -w [ --wordy ]                  verbose/wordy/debug
 * \endcode
 * \author David Stutz, Marven von Domarus
 */
int main(int argc, const char** argv) {
    
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("dimensions,d", boost::program_options::value<int>()->default_value(1), "number of subimages, 1 for traditional ers ,  > 1 for multi-dim ers")
        ("input,i", boost::program_options::value<std::string>(), "the folder to process")
        ("lambda,l", boost::program_options::value<double>()->default_value(0.5), "lambda")
        ("sigma,g", boost::program_options::value<double>()->default_value(5.0), "sigma")
        ("pnorm,p", boost::program_options::value<double>()->default_value(2.0), "pnorm")
        ("eight-connected,f", "use 8-connected")
        ("superpixels,s", boost::program_options::value<int>()->default_value(400), "number of superpixels")
        ("csv,o", boost::program_options::value<std::string>()->default_value(""), "save segmentation as CSV file")
        ("vis,v", boost::program_options::value<std::string>()->default_value(""), "visualize contours")
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
    
    int four_connected = 1;
    if (parameters.find("eight-connected") != parameters.end()) {
        four_connected = 0;
    }

    int dimensions= parameters["dimensions"].as<int>();
    int superpixels = parameters["superpixels"].as<int>();
    double lambda = parameters["lambda"].as<double>();
    double sigma = parameters["sigma"].as<double>();
    double pnorm = parameters["pnorm"].as<double>();
    

    std::multimap<std::string, boost::filesystem::path> images;
    std::vector<std::string> extensions;
    IOUtil::getImageExtensions(extensions);
    IOUtil::readDirectory(input_dir, extensions, images);

    float total = 0;

    std::vector<cv::Mat> cur_image; //current multi dimensional image
    std::string img_name ="";
    int countImg=0;
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = images.begin();
            it != images.end(); ++it) {


        cv::Mat subimage = cv::imread(it->first);
        cur_image.push_back(subimage);


        if(cur_image.size() % dimensions == 1 || dimensions==1){
            //first subimage. catch name
            img_name=it->second.stem().string();
        }

        if(cur_image.front().rows != cur_image.back().rows || cur_image.front().cols != cur_image.back().cols){
            std::cout << "Subimages have non matching rows and cols..." << std::endl;
            return 1;
        }

        if(cur_image.size() % dimensions == 0){

            boost::timer::cpu_timer timer;
            cv::Mat labels;

            ERS_OpenCV::computeSuperpixels(cur_image, superpixels, lambda, sigma, four_connected, pnorm,labels);

            auto seconds = boost::chrono::duration_cast<boost::chrono::seconds>( boost::chrono::nanoseconds(timer.elapsed().user + timer.elapsed().system));
            total += seconds.count();

            int unconnected_components = SuperpixelTools::relabelConnectedSuperpixels(labels);

            if (wordy) {
                std::cout << SuperpixelTools::countSuperpixels(labels) << " superpixels for " << img_name
                        << " (" << unconnected_components << " not connected; "
                        <<  seconds.count() <<")." << std::endl;
            }

            if (!output_dir.empty()) {
                boost::filesystem::path csv_file(output_dir
                        / boost::filesystem::path(prefix + img_name + ".csv"));
                IOUtil::writeMatCSV<int>(csv_file, labels);
            }


            cur_image.clear();

            countImg++;

            std::cout << "Images completed: " << countImg <<"/"<< (images.size() /(double)dimensions) << "." << std::endl;

            std::cout << "Average time: " << total / (double)countImg << "s." << std::endl;
        }
    }
    if(cur_image.size() % dimensions != 0){
        std::cout << "WARNING: Number of (sub)images in the input directory is not divisibly by the parameter dimensions. Remainder sub images: "<<cur_image.size()<<std::endl;
    }
    if (wordy) {
        std::cout << "Average time: " << total / (images.size() / (double)dimensions)<< "." << std::endl;
    }

    if (!output_dir.empty()) {
        std::ofstream runtime_file(output_dir.string() + "/" + prefix + "runtime.txt",
                std::ofstream::out | std::ofstream::app);

        runtime_file << total / (images.size() / (double)dimensions) << "\n";
        runtime_file.close();
    }

    return 0;
}
