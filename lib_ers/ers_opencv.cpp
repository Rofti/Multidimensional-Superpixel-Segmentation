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

#include "MERCLazyGreedy.h"
#include "MERCInputImage.h"
#include "MERCOutputImage.h"
#include "Image.h"
#include "ImageIO.h"
#include "ers_opencv.h"

void ERS_OpenCV::computeSuperpixels(const cv::Mat& image, int superpixels, 
        double lambda, double sigma, int four_connected, cv::Mat& labels) {
    
    int kernel = 0;
    MERCLazyGreedy merc;
    Image<RGBMap> input_image;
    MERCInputImage<RGBMap> input;

    input_image.Resize(image.cols, image.rows, false);

    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            RGBMap color((int) image.at<cv::Vec3b>(i, j)[2], 
                    (int) image.at<cv::Vec3b>(i, j)[1], 
                    (int) image.at<cv::Vec3b>(i, j)[0]);
            input_image.Access(j, i) = color;
        }
    }

    input.ReadImage(&input_image, 1 - four_connected);

    merc.ClusteringTreeIF(input.nNodes_, input, kernel, sigma*image.channels(), 
            lambda*1.0*superpixels, superpixels);

    vector<int> label = MERCOutputImage::DisjointSetToLabel(merc.disjointSet_);

    labels.create(image.rows, image.cols, CV_32SC1);
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            labels.at<int>(i, j) = label[j + i*image.cols];
        }
    }
}

void ERS_OpenCV::computeSuperpixels(const std::vector<cv::Mat>& image, int superpixels,
        double lambda, double sigma, int four_connected, int pnorm,  cv::Mat& labels) {

    int kernel = 0;
    MERCLazyGreedy merc;
    Image<RGBMapD> input_image;
    MERCInputImage<RGBMapD> input;


    input_image.Resize(image.front().cols, image.front().rows, false);


    for (int i = 0; i < image.front().rows; ++i) {
        for (int j = 0; j < image.front().cols; ++j) {
            uchar r[image.size()], g[image.size()], b[image.size()];
            for(int d = 0; d < image.size();++d){

                r[d]=(uchar) image.at(d).at<cv::Vec3b>(i, j)[2];
                g[d]=(uchar) image.at(d).at<cv::Vec3b>(i, j)[1];
                b[d]=(uchar) image.at(d).at<cv::Vec3b>(i, j)[0];
            }
            RGBMapD colors(image.size(),r,g,b,pnorm);

            input_image.Access(j, i) = colors;
        }
    }

    input.ReadImage(&input_image, 1 - four_connected);



    merc.ClusteringTreeIF(input.nNodes_, input, kernel, sigma*image.front().channels(),
            lambda*1.0*superpixels, superpixels);


    vector<int> label = MERCOutputImage::DisjointSetToLabel(merc.disjointSet_);


    labels.create(image.front().rows, image.front().cols, CV_32SC1);
    for (int i = 0; i < image.front().rows; ++i) {
        for (int j = 0; j < image.front().cols; ++j) {
            labels.at<int>(i, j) = label[j + i*image.front().cols];
        }
    }

}
