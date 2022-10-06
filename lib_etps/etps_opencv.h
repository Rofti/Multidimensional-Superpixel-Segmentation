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

#ifndef ETPS_OPENCV_H
#define	ETPS_OPENCV_H

#include <opencv2/opencv.hpp>

/** \brief Wrapper for running ETPS on OpenCV images.
 * \author David Stutz, Marven von Domarus
 */
class ETPS_OpenCV {
public:
    /** \brief Compute superpixels using ETPS.
     * \param[in] image image to compute superpixels on
     * \param[in] region_size step size between superpixel centers, implicitly defining the number of superpixels
     * \param[in] regularization_weight compactness parameter,
     * \param[in] size_weight size weight
     * \param[in] iterations number of iterations
     * \param[out] labels superpixel labels
     */
    static void computeSuperpixels(const std::vector<cv::Mat> &image, int region_size,
            double regularization_weight, double length_weight, 
            double size_weight, int iterations, cv::Mat &labels, int image_dimensions, int p_norm);
};

#endif	/* ETPS_OPENCV_H */

