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

#ifndef SLIC_OPENCV_H
#define	SLIC_OPENCV_H

#include <opencv2/opencv.hpp>

/** \brief Wrapper for running SLIC on OpenCV images.
 * \author David Stutz
 */
class SLIC_OpenCV {
public:
    /** \brief Compute superpixels using SLIC.
     * \param[in] image image to compute superpixels on
     * \param[in] region_size size between superpixels implicitly defining number of superpixels
     * \param[in] compactness compactness parameter
     * \param[in] iterations number of iterations
     * \param[in] perturb_seeds whether to perturb seeds for better performance
     * \param[in] color_space color space to use, > 0 for Lab, 0 for RGB
     * \param[out] labels superpixel labels
     */
    static void computeSuperpixels(const std::vector<cv::Mat> &image, int region_size,
            double compactness, int iterations, bool perturb_seeds,
            int color_space, int p_norm, cv::Mat &labels);
    static void computeSuperpixels_ubuff(const std::vector<cv::Mat> &image, int region_size,
            double compactness, int iterations, bool perturb_seeds,
            int color_space, int p_norm, cv::Mat &labels);
};

#endif	/* SLIC_OPENCV_H */

