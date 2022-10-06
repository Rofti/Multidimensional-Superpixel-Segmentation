/**
 * Copyright (c) 2016, David Stutz
 * Contact: david.stutz@rwth-aachen.de, davidstutz.de
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

#include "SLIC.h"
#include "slic_opencv.h"
/*
void SLIC_OpenCV::computeSuperpixels(const cv::Mat &mat, int region_size,
        double compactness, int iterations, bool perturb_seeds,
        int color_space, cv::Mat &labels) {

    // Convert matrix to unsigned int array.
    unsigned int* image = new unsigned int[mat.rows*mat.cols];
    unsigned int value = 0x0000;

    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {

            int b = mat.at<cv::Vec3b>(i,j)[0];
            int g = mat.at<cv::Vec3b>(i,j)[1];
            int r = mat.at<cv::Vec3b>(i,j)[2];

            value = 0x0000;
            value |= (0xFF000000);
            value |= (0x00FF0000 & (r << 16));
            value |= (0x0000FF00 & (g << 8));
            value |= (0x000000FF & b);

            image[j + mat.cols*i] = value;
        }
    }

    SLIC slic;

    int* segmentation = new int[mat.rows*mat.cols];
    int number_of_labels = 0;

    slic.DoSuperpixelSegmentation_ForGivenSuperpixelStep(image, mat.cols,
            mat.rows, segmentation, number_of_labels, region_size,
            compactness, perturb_seeds, iterations, color_space);

    // Convert labels.
    labels.create(mat.rows, mat.cols, CV_32SC1);
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            labels.at<int>(i, j) = segmentation[j + i*mat.cols];
        }
    }
}
*/
void SLIC_OpenCV::computeSuperpixels(const std::vector<cv::Mat> &matVec, int region_size,
        double compactness, int iterations, bool perturb_seeds, 
        int color_space, int p_norm, cv::Mat &labels) {

/*
    // Convert vector of matrices to array of unsigned int arrays.
    std::vector<std::vector<unsigned int>> images= std::vector<std::vector<unsigned int>>(matVec.size(), std::vector<unsigned int>(matVec.front().rows*matVec.front().cols));
    unsigned int value = 0x0000;
    //std::vector<unsigned int*> imagesTest(matVec.size(), new unsigned int[matVec.front().rows*matVec.front().cols]);
    for(int dim=0; dim < matVec.size(); ++dim){
        //images[dim]=new unsigned int[matVec.at(dim).rows*matVec.at(dim).cols];


        for (int i = 0; i < matVec.at(dim).rows; ++i) {
            for (int j = 0; j < matVec.at(dim).cols; ++j) {

                int b = matVec.at(dim).at<cv::Vec3b>(i,j)[0];
                int g = matVec.at(dim).at<cv::Vec3b>(i,j)[1];
                int r = matVec.at(dim).at<cv::Vec3b>(i,j)[2];

                value = 0x0000;
                value |= (0xFF000000);
                value |= (0x00FF0000 & (r << 16));
                value |= (0x0000FF00 & (g << 8));
                value |= (0x000000FF & b);

                images[dim][j + matVec.at(dim).cols*i] = value;
                //imagesTest.at(dim)[j + matVec.at(dim).cols*i];
            }
        }
    }

*/


    SLIC slic;

    int* segmentation = new int[matVec.front().rows*matVec.front().cols];
    int number_of_labels = 0;

    slic.DoSuperpixelSegmentation_ForGivenSuperpixelStep_MultipleDimensions(matVec, matVec.front().cols,
            matVec.front().rows,matVec.size(), segmentation, number_of_labels, region_size,
            compactness, perturb_seeds, iterations, color_space, p_norm);

    // Convert labels .
    labels.create(matVec.front().rows, matVec.front().cols, CV_32SC1);
    for (int i = 0; i < matVec.front().rows; ++i) {
        for (int j = 0; j < matVec.front().cols; ++j) {
            labels.at<int>(i, j) = segmentation[j + i*matVec.front().cols];
        }
    }
}

void SLIC_OpenCV::computeSuperpixels_ubuff(const std::vector<cv::Mat> &matVec, int region_size,
        double compactness, int iterations, bool perturb_seeds,
        int color_space, int p_norm, cv::Mat &labels) {


    // Convert vector of matrices to array of unsigned int arrays.
    std::vector<std::vector<unsigned int>> images= std::vector<std::vector<unsigned int>>(matVec.size(), std::vector<unsigned int>(matVec.front().rows*matVec.front().cols));
    unsigned int value = 0x0000;
    //std::vector<unsigned int*> imagesTest(matVec.size(), new unsigned int[matVec.front().rows*matVec.front().cols]);
    for(int dim=0; dim < matVec.size(); ++dim){
        //images[dim]=new unsigned int[matVec.at(dim).rows*matVec.at(dim).cols];


        for (int i = 0; i < matVec.at(dim).rows; ++i) {
            for (int j = 0; j < matVec.at(dim).cols; ++j) {

                int b = matVec.at(dim).at<cv::Vec3b>(i,j)[0];
                int g = matVec.at(dim).at<cv::Vec3b>(i,j)[1];
                int r = matVec.at(dim).at<cv::Vec3b>(i,j)[2];

                value = 0x0000;
                value |= (0xFF000000);
                value |= (0x00FF0000 & (r << 16));
                value |= (0x0000FF00 & (g << 8));
                value |= (0x000000FF & b);

                images[dim][j + matVec.at(dim).cols*i] = value;
                //imagesTest.at(dim)[j + matVec.at(dim).cols*i];
            }
        }
    }



    SLIC slic;

    int* segmentation = new int[matVec.front().rows*matVec.front().cols];
    int number_of_labels = 0;

    slic.DoSuperpixelSegmentation_ForGivenSuperpixelStep_MultipleDimensions(images, matVec.front().cols,
            matVec.front().rows,matVec.size(), segmentation, number_of_labels, region_size,
            compactness, perturb_seeds, iterations, color_space, p_norm);

    // Convert labels.
    labels.create(matVec.front().rows, matVec.front().cols, CV_32SC1);
    for (int i = 0; i < matVec.front().rows; ++i) {
        for (int j = 0; j < matVec.front().cols; ++j) {
            labels.at<int>(i, j) = segmentation[j + i*matVec.front().cols];
        }
    }
}
