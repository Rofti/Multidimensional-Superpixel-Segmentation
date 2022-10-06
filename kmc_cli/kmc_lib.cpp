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

#include "kmc_lib.h"

void KMC::compressSuperpixels(std::vector<cv::Mat> segmentations, int k, cv::Mat& labels){
    rows=segmentations.front().rows;
    cols = segmentations.front().cols;
    input_segmentations = segmentations;
    kmedoids(k,labels);
}

double KMC::getDistance(Pixel px1, Pixel px2){
    double dist=0;
     for(int d=0; d < input_segmentations.size(); ++d){
         if(input_segmentations.at(d).at<int>(px1.y,px1.x) == input_segmentations.at(d).at<int>(px2.y,px2.x)){
             //pixel share same label in current segmentation/dimension
             dist++;
         }
     }
     dist= 1.0 - (dist/input_segmentations.size());
     return dist;
}

void KMC::kmedoids(int k, cv::Mat& labels ){
    initSeeds(k);
    bool changes=true;
    while(changes){
        changes = updateMedoids();
        updateClusterMemberships();
    }

   labels = cv::Mat::zeros(rows,cols,CV_32SC1);
   for(unsigned int i=0; i<cluster.size();++i){
       for(std::set<int>::iterator it = cluster.at(i).begin(); it!=cluster.at(i).end(); ++it){

           int it_x=(*it)%cols; //pixel id = x + (cols * y)  and x<cols
           int it_y=((*it)-it_x)/cols;
           labels.at<unsigned int>(it_y,it_x)=i;
       }
   }
}

void KMC::initSeeds(int k){

    int STEP = 0.5f + std::sqrt(rows*cols / (float)k);

    int numseeds=0;

    int xstrips = (0.5+float(cols)/float(STEP));
    int ystrips = (0.5+float(rows)/float(STEP));

    int xerr = cols  - STEP*xstrips;if(xerr < 0){xstrips--;xerr = cols - STEP*xstrips;}
    int yerr = rows - STEP*ystrips;if(yerr < 0){ystrips--;yerr = rows- STEP*ystrips;}

    float xerrperstrip = float(xerr)/float(xstrips);
    float yerrperstrip = float(yerr)/float(ystrips);

    int xoff = STEP/2;
    int yoff = STEP/2;
    //-------------------------
    numseeds = xstrips*ystrips;
    //-------------------------

    cluster = std::vector<std::set<int> >(numseeds,std::set<int>()); //contains for each mediod the IDs of all its current member pixel

    medoid_pixel = std::vector<Pixel>(); //contains for every medoid the currently corresponding pixel

    for( int y = 0; y < ystrips; y++ )
    {
        int ye = y*yerrperstrip;
        for( int x = 0; x < xstrips; x++ )
        {
            int xe = x*xerrperstrip;

            Pixel seed;
            seed.x=(x*STEP+xoff+xe);
            seed.y=(y*STEP+yoff+ye);

            medoid_pixel.push_back(seed); //contains for every medoid the currently corresponding pixel

        }
    }

    updateClusterMemberships();
}

bool KMC::updateMedoids(){

    std::vector<Pixel> old_medoid_pixel(medoid_pixel); //deep copy of medoid pixel

    for(unsigned int cluster_id=0; cluster_id < medoid_pixel.size(); ++cluster_id ){

        //find average pixel coordinates in cluster
        double avg_x=0, avg_y=0;

        for(std::set<int>::iterator it=cluster.at(cluster_id).begin(); it!=cluster.at(cluster_id).end() ; ++it){
             int it_x=(*it)%cols; //pixel id = x + (cols * y)  and x<cols
             int it_y=((*it)-it_x)/cols;

             avg_x+=it_x;
             avg_y+=it_y;
         }
         avg_x=avg_x/cluster.at(cluster_id).size();
         avg_y=avg_y/cluster.at(cluster_id).size();

         //find closest member pixel (new medoid)
         double minDist= std::numeric_limits<double>::max();

         for(std::set<int>::iterator it=cluster.at(cluster_id).begin(); it!=cluster.at(cluster_id).end() ; ++it){
              int it_x=(*it)%cols; //pixel id = x + (cols * y)  and x<cols
              int it_y=((*it)-it_x)/cols;

              double cur_dist =std::sqrt( std::pow((avg_x - it_x),2) + std::pow((avg_y - it_y),2) );

              if(minDist< cur_dist){
                  minDist = cur_dist;

                  //set new Medoid
                  medoid_pixel.at(cluster_id).x=it_x;
                  medoid_pixel.at(cluster_id).y=it_y;
              }
          }
    }

    //check for changes in medoids.
    bool res=false;
    for(int i=0; i<medoid_pixel.size(); ++i){
        if(medoid_pixel.at(i).x != old_medoid_pixel.at(i).x
                && medoid_pixel.at(i).y != old_medoid_pixel.at(i).y) {
            res=true;
        }
    }
    return res;
}

void KMC::updateClusterMemberships(){

    cluster = std::vector<std::set<int>>(cluster.size(),std::set<int>()); //reset cluster

    for(int x=0; x<cols; ++x){
       for(int y=0; y<rows; ++y){
           float minDist= std::numeric_limits<float>::max();
           int min_clusterID=-1;

           for(int cluster_id=0; cluster_id < medoid_pixel.size(); ++cluster_id ){
               Pixel cur{x,y};
               double curDist=getDistance(medoid_pixel.at(cluster_id), cur);
               if(minDist> curDist){
                   minDist=curDist;
                   min_clusterID=cluster_id;
               }
           }
           cluster.at(min_clusterID).insert(x + (cols*y));
       }
    }
}
