/**
 * \file	histogram.h
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	8/09/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_HISTOGRAM_H_
#define ATLAS_MATHS_HISTOGRAM_H_

#include <vector>

namespace atlas {

template <typename Data>
class Histogram {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Histogram(std::vector<Data> const &data);

  Histogram(std::vector<Data> const &data, double inter);

  Histogram(std::vector<Data> const &data, double max, double min);

  Histogram(std::vector<Data> const &data, double inter, double max,
            double min);

  ~Histogram();

  //============================================================================
  // P U B L I C  M E T H O D S

  std::vector<Data> *CreateHistogram();

  /**
 * To get the index of the maximum value.
 *
 * \return the index of the maximum value.
 */

  auto GetMaxIndex() -> double;

  /**
* To get the index of the minimum value.
*
* \return the index of the minimum value.
*/

  auto GetMinIndex() -> double;

  /**
* To get the maximum value of the histogram.
*
* \return the maximum value of the histogram in the same format of
*  the data vector.
*/

  Data GetMaxValue();

  /**
* To get the minimum value of the histogram.
*
* \return the minimum value of the histogram in the same format of
*  the data vector.
*/

  Data GetMinValue();

  /**
* To change the interval.
*
* If you want to change the interval of the histogram
*/

  void SetInter(double inter);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  int size_;
  double inter_;
  Data max_data_;
  Data min_data_;
  Data max_histogram_;
  Data min_histogram_;
  std::vector<Data> histogram_;
  std::vector<Data> data_;
  std::vector<Data> value_histogram_;
};
}

#include <lib_atlas/maths/histogram_inl.h>

#endif  // LIB_ATLAS_HISTOGRAM_H
