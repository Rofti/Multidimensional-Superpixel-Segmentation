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

#ifndef _kmc_h_
#define _kmc_h_
#include <opencv2/opencv.hpp>
#include "superpixel_tools.h"
#include <limits.h>
//k-medoids compression. Takes multiple segmentation and uses k-mediods to compress those segmentations into a single one.
//used on results of seperatly run superpixel algorithms on different image dimensions.
class KMC
{
public:
    void compressSuperpixels(std::vector<cv::Mat> segmentations, int k, cv::Mat& labels);
private:
    std::vector<cv::Mat> input_segmentations;
    int rows=0;
    int cols=0;
    struct Pixel{
        int x;
        int y;
    };
    std::vector<std::set<int>> cluster; //contains for each mediod all its member pixel ids
    std::vector<Pixel> medoid_pixel; //contains for every medoid the currently corresponding pixel

    //cv::Mat createDistanceMatrix(std::vector<cv::Mat> segmentations);
    double getDistance(Pixel px1, Pixel px2);
    void kmedoids(int k, cv::Mat& labels );
    void initSeeds( int k); //initializes seeds for kmedoids and initalizes px_clusterID ad medoid_pixel
    bool updateMedoids();//calculates for each cluster the new medoid depending on its members and updates medoid_pixel accordingly
    void updateClusterMemberships(); //checks for each pixel its cluster membership based on closest medoid
};
#endif
