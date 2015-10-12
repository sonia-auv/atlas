/**
 * \file	histogram.h
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	8/09/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_HISTOGRAM_H_
#define LIB_ATLAS_MATHS_HISTOGRAM_H_

#include <vector>
#include <tuple>
#include <map>
#include <lib_atlas/macros.h>

namespace atlas {

template <typename Tp_>
class Histogram {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Histogram();

  Histogram(std::vector<Tp_> const &data, double inter);

  Histogram(std::vector<Tp_> const &data, unsigned int function);

  ~Histogram();

  //============================================================================
  // P U B L I C  O P E R A T O R S

  Histogram<Tp_> operator=(const Histogram<Tp_> &histo);

  //============================================================================
  // P U B L I C  M E T H O D S

  /**
   * To get the index of the maximum value.
   *
   * \return the index of the maximum value.
   */
  double GetMaxIndex() const ATLAS_NOEXCEPT;

  /**
   * To get the index of the minimum value.
   *
   * \return the index of the minimum value.
   */

  double GetMinIndex() const ATLAS_NOEXCEPT;

  /**
   * To get the maximum value of the histogram.
   *
   * \return the maximum value of the histogram in the same format of
   *  the data vector.
   */
  Tp_ GetMaxValue();

  /**
 * To get the minimum value of the histogram.
 *
 * \return the minimum value of the histogram in the same format of
 *  the data vector.
 */

  Tp_ GetMinValue();

  /**
   * To change the interval by passing a value.
   *
   * If you want to change the interval of the histogram.
   * It automatically recreate an histogram with the new interval.
   */
  void SetInterNumber(double inter);

  /**
   * To change the interval by passing a function.
   *
   * If you want to change the interval of the histogram.
   * It automatically recreate an histogram with the new interval.
   */
  void SetInterFunction(unsigned int function);

  /**
   * To add data in the histogram.
   *
   * Will add the data to the existing histogram.
   */
  void AddData(std::vector<Tp_> const &data);

  /**
   * To zoom on a certain region of the histogram.
   *
   * \return a new histogram
   */
  Histogram ZoomHistogram(Tp_ begin_zoom, Tp_ end_zoom);

  /**
   * To find the occurencie of a value
   *
   * \return the number of occurencie.
   */
  int FindOccurencie(Tp_ value);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  unsigned int (*pfunc_inter_)(unsigned int);

  Tp_ max_data_;

  Tp_ min_data_;

  std::map<Tp_, int> histogram_;

  bool inter_func_;

  double inter_;

  //============================================================================
  // P R I V A T E  M E T H O D S

  void CreateHistogram(std::vector<Tp_> const &data);

  /**
   * Creat a new histogram based on the new interval.
   *
   */
  void RefactorHistogram();
};
}

#include <lib_atlas/maths/histogram_inl.h>

#endif  // LIB_ATLAS_MATH_HISTOGRAM_H_
