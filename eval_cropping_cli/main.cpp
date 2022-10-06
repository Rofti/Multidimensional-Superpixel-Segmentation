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

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/timer.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

#include "io_util.h"

/** \brief Divide images and corresponding CSV into similar sized subimages.
 * Usage:
 * \code{sh}
 *   $ ../bin/eval_cropping_cli --help
 *   Allowed options:
 *     -i --img-directory arg       image directory
 *     -g --gt-directory arg        ground truth directory
 *     -o --output-directory arg    output directory
 *     -d --dimensions arg (=1)     number of input (sub)images that correspond to one full image
 *     -t --divisor arg (=4)        number of output subimages to split an input image into
 *     -p --prefix arg              safe file prefix
 *     --help                       produce help message
 * \endcode
 * \author Marven von Domarus
 */
int main(int argc, const char** argv) {
    
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("img-directory,i", boost::program_options::value<std::string>(), "image directory")
            ("gt-directory,g", boost::program_options::value<std::string>(), "ground truth directory")
            ("output-directory,o", boost::program_options::value<std::string>()->default_value(""), "output directory")
            ("dimensions,d", boost::program_options::value<int>()->default_value(1), "number of input (sub)images that correspond to one full image")
            ("divisor,t", boost::program_options::value<int>()->default_value(4), "number of input (sub)images that correspond to one full image")
            ("csv-forallimages,c", "create seperate csv for all subimages")
            ("prefix,p", boost::program_options::value<std::string>(), "prefix string for naming newly created subimages")
            ("help", "produce help message");

    boost::program_options::positional_options_description positionals;
    positionals.add("img-directory", 1);
    positionals.add("gt-directory", 1);
    positionals.add("prefix", 1);
    
    boost::program_options::variables_map parameters;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(positionals).run(), parameters);
    boost::program_options::notify(parameters);

    if (parameters.find("help") != parameters.end()) {
        std::cout << desc << std::endl;
        return 1;
    }
    

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

    boost::filesystem::path output_dir(parameters["output-directory"].as<std::string>());
    if (!output_dir.empty()) {
        if (!boost::filesystem::is_directory(output_dir)) {
            boost::filesystem::create_directories(output_dir);
        }
    }


    bool csvForAll = false;
    if (parameters.find("csv-forallimages") != parameters.end()) {
        csvForAll = true;
    }

    int dimensions = parameters["dimensions"].as<int>();
    int divisor = parameters["divisor"].as<int>();
    std::string prefix = parameters["prefix"].as<std::string>();

    std::vector<std::string> extensions;
    IOUtil::getImageExtensions(extensions);
    std::multimap<std::string, boost::filesystem::path> images;
    IOUtil::readDirectory(img_directory, extensions, images);

    extensions.clear();
    IOUtil::getCSVExtensions(extensions);
    std::multimap<std::string, boost::filesystem::path> GTs;
    IOUtil::readDirectory(gt_directory, extensions, GTs);

    std::vector<cv::Mat> cur_image; //current multi dimensional image
    boost::filesystem::path first_img_path;
    std::vector<std::string> cur_simg_names;

    cv::Mat cur_gt_segmentation;
    boost::filesystem::path cur_gt_file;

    float countFullImg= 0;
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = images.begin();
         it != images.end(); ++it) {

        cv::Mat subimage = cv::imread(it->first);
        cur_image.push_back(subimage);
        cur_simg_names.push_back(it->second.stem().string());

        if(cur_image.size() % dimensions == 1 || dimensions==1){
            //first subimage. catch path
            first_img_path = it->second;

            //get the right ground truth
            boost::filesystem::path gtFile(gt_directory / boost::filesystem::path(it->second.stem().string() + ".csv"));
            std::multimap<std::string, boost::filesystem::path>::iterator foundGTs = GTs.find(gtFile.string());

            if(foundGTs == GTs.end()){//Try different csv extension if nothing came up
                gtFile = boost::filesystem::path (gt_directory / boost::filesystem::path(it->second.stem().string() + ".Csv"));
                foundGTs = GTs.find(gtFile.string());
            }
            if(foundGTs == GTs.end()){
                std::cout << "No fitting ground truth found for "<< it->second.stem().string()<< std::endl;
                std::cout<<"GTs size"<<GTs.size()<<std::endl;
                if(GTs.size()>0){
                    std::cout<<"Contains following CSVs:" <<std::endl;
                    for(std::pair<std::string, boost::filesystem::path> git : GTs)
                        std::cout<<git.first<<std::endl;;

                }

                return -1;
            }

            cur_gt_file = foundGTs->second;

            LOG_IF(FATAL, !boost::filesystem::is_regular_file(cur_gt_file))
                    << "Ground truth not found.("<<cur_gt_file.string()<<").";

            IOUtil::readMatCSVInt(cur_gt_file, cur_gt_segmentation);


            LOG_IF(FATAL, cur_gt_segmentation.rows != cur_image.front().rows
                    || cur_gt_segmentation.cols != cur_image.front().cols)
                    << "Ground truth does not match image size.";
        }

        if(cur_image.front().rows != cur_image.back().rows || cur_image.front().cols != cur_image.back().cols){
            std::cout << "Subimages have non matching rows and cols..." << std::endl;
            return 1;
        }
        if(cur_image.size() % dimensions == 0){

            int rows = cur_image.front().rows,        cols = cur_image.front().cols;
            int gridsizeY = static_cast<int>( rows/ (double)divisor );
            int gridsizeX = static_cast<int>( cols/ (double)divisor );
            int gridCount = 0;

            for (int y = 0; y < rows - (gridsizeY-1); y += gridsizeY) {
                for (int x = 0; x < cols - (gridsizeX-1); x += gridsizeX) {
                    for(int i = 0; i < dimensions; ++i){

                        if(csvForAll){
                            cv::Mat curGridGT = cur_gt_segmentation(cv::Rect(x,y,gridsizeX,gridsizeY));
                            boost::filesystem::path csv_file(output_dir
                                                             / boost::filesystem::path(prefix+"_"+ std::to_string(gridCount) +"_"+ cur_simg_names.at(i) + ".csv"));
                            IOUtil::writeMatCSV<int>(csv_file, curGridGT);
                        }
                        else if(i==0){
                            cv::Mat curGridGT = cur_gt_segmentation(cv::Rect(x,y,gridsizeX,gridsizeY));
                            boost::filesystem::path csv_file(output_dir
                                                             / boost::filesystem::path(prefix+"_"+ std::to_string(gridCount) +"_"+ cur_simg_names.at(0) + ".csv"));
                            IOUtil::writeMatCSV<int>(csv_file, curGridGT);
                        }

                        cv::Mat curGridImg = cur_image.at(i)(cv::Rect(x,y,gridsizeX,gridsizeY));
                        boost::filesystem::path img_file(output_dir
                                                         /boost::filesystem::path(prefix +"_"+std::to_string(gridCount)+"_"+ cur_simg_names.at(i) + ".png"));
                        cv::imwrite(img_file.string(),curGridImg);
                    }
                    ++gridCount;
                }
            }

            cur_image.clear();
            countFullImg++;
            std::cout << "Images completed: " << countFullImg / (images.size() /(double)dimensions) << "." << std::endl;

        }
    }
    return 0;
}
