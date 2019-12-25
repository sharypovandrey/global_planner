/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Andrey Sharypov
 *********************************************************************/
#include <global_planner/expander.h>
#include <global_planner/a_super_star.h>
#include <global_planner/astar.h>
#include <costmap_2d/cost_values.h>
#include <ros/ros.h>
#include <math.h>
#include <algorithm>    // std::max


namespace global_planner {

ASuperStar::ASuperStar(PotentialCalculator* p_calc, int xs, int ys) :
    Expander(p_calc, xs, ys) {
        // the risk coefficient
        beta = 50;
        // the distance coefficient
        theta = 1;
        // kernel size is a range for obstacle cell where we calculate risk values
        kernel_size_ = 20;
}

bool ASuperStar::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential) {

    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    // let's prepare risks map
    calculateRisk(costs);

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

void ASuperStar::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y) {
    if (next_i < 0 || next_i >= ns_)
        return;

    if (potential[next_i] < POT_HIGH)
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    // update potential with risk value
    potential[next_i] = potential[next_i] + beta * risks_mat[x][y];
    float distance = abs(end_x - x) + abs(end_y - y);

    queue_.push_back(Index(next_i, potential[next_i] + theta * distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

void ASuperStar::calculateRisk(unsigned char* costs) {
    std::cout << nx_ << std::endl;
    std::cout << ny_ << std::endl;
    
    risks_mat = std::vector<std::vector<float> >(nx_, std::vector<float>(ny_, 0));
    std::vector<std::vector<int> > costs_mat (nx_, std::vector<int>(ny_, 0));
    for (int xi_=0; xi_ < nx_; xi_++) {
        for (int yi_=0; yi_ < ny_; yi_++) {
            costs_mat[xi_][yi_] = int(costs[toIndex(xi_, yi_)]);
        }
    }

    int cost_val;
    int offset_ = int(kernel_size_ / 2);
    for (int xi_ = 0; xi_ < costs_mat.size(); xi_++) {
        for (int yi_ = 0; yi_ < costs_mat[0].size(); yi_++) {

            if (costs_mat[xi_][yi_] > 0) {
                risks_mat[xi_][yi_] = 9.0;
            } else { 
                continue; 
            }

            for (int kx = -offset_; kx < kernel_size_ - offset_; kx++) {
                for (int ky = -offset_; ky < kernel_size_ - offset_; ky++) {
                    // check that index is not out of bounds
                    if (
                        xi_ + kx >= 0 && xi_ + kx < nx_ && 
                        yi_ + ky >= 0 && yi_ + ky < ny_ && 
                        (kx != 0 && ky != 0) 
                    )
                    {
                        float d = sqrt(pow(kx, 2.0) + pow(ky, 2.0));
                        float r = 1/d + 1;
                        risks_mat[xi_ + kx][yi_ + ky] = std::max(float(risks_mat[xi_ + kx][yi_ + ky]), r);
                        
                    }
                }
            }

        } 
    }


}

} //end namespace global_planner
