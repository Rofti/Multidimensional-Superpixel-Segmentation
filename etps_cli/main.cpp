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
#include "etps_opencv.h"
#include "io_util.h"
#include "superpixel_tools.h"
#include "visualization.h"

/** \brief Command line tool for running ETPS.
 * Usage:
 * \code{sh}
 *   $ ../bin/etps_cli --help
 *   Allowed options:
 *     -h [ --help ]                         produce help message
 *     -i [ --input ] arg                    the folder to process
 *     -s [ --superpixels ] arg (=400)       number of superpixels
 *     -d [ --dimensions ] arg (=1)          number of subimages, 1 for traditional etps ,  > 1 for multi-dim etps
 *     -c [ --regularization-weight ] arg (=1)
 *                                           regularization weight
 *     -l [ --length-weight ] arg (=1)       length weight
 *     -n [ --size-weight ] arg (=1)         size weight
 *     -t [ --iterations ] arg (=1)          number of iterations
 *     -o [ --csv ] arg                      save segmentation as CSV file
 *     -v [ --vis ] arg                      visualize contours
 *     -x [ --prefix ] arg                   output file prefix
 *     -w [ --wordy ]                        verbose/wordy/debug
 * \endcode
 * \author David Stutz, Marven von Domarus
 */
int main(int argc, const char** argv) {
    
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("input,i", boost::program_options::value<std::string>(), "the folder to process")
        ("superpixels,s", boost::program_options::value<int>()->default_value(400), "number of superpixels")
        ("dimensions,d", boost::program_options::value<int>()->default_value(1), "number of subimages, 1 for traditional etps ,  > 1 for multi-dim etps")
        ("p-norm,p", boost::program_options::value<int>()->default_value(2), "p norm")
        ("regularization-weight,c", boost::program_options::value<double>()->default_value(1.0), "regularization weight")
        ("length-weight,l", boost::program_options::value<double>()->default_value(1.0), "length weight")
        ("size-weight,n", boost::program_options::value<double>()->default_value(1.0), "size weight")
        ("iterations,t", boost::program_options::value<int>()->default_value(1), "number of iterations")
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
    
    int superpixels = parameters["superpixels"].as<int>();
    int dimensions = parameters["dimensions"].as<int>();
    int p_norm = parameters["p-norm"].as<int>();
    double regularization_weight = parameters["regularization-weight"].as<double>();
    double length_weight = parameters["length-weight"].as<double>();
    double size_weight = parameters["size-weight"].as<double>();
    int iterations = parameters["iterations"].as<int>();
    
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

            // Same conversion for all algorithms.
            int region_size = SuperpixelTools::computeRegionSizeFromSuperpixels(cur_image.front(),
                    superpixels);

            boost::timer::cpu_timer timer;
            cv::Mat labels;
            ETPS_OpenCV::computeSuperpixels(cur_image, region_size, regularization_weight,
                    length_weight, size_weight, iterations, labels, dimensions, p_norm);

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

            for(int l = 0; l<cur_image.size(); ++l){
                if (!vis_dir.empty()) {
                    boost::filesystem::path contours_file(vis_dir
                                                          / boost::filesystem::path(prefix + img_name+"_"+std::to_string(l) + ".png"));
                    cv::Mat image_contours;
                    Visualization::drawContours(cur_image.at(l), labels, image_contours);
                    cv::imwrite(contours_file.string(), image_contours);
                }
                else if(!output_dir.empty()){
                    boost::filesystem::path contours_file(output_dir
                                                          / boost::filesystem::path(prefix +"vis"+  img_name + "_"+std::to_string(l) +".png"));
                    cv::Mat image_contours;
                    Visualization::drawContours(cur_image.at(l), labels, image_contours);
                    cv::imwrite(contours_file.string(), image_contours);
                }
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
