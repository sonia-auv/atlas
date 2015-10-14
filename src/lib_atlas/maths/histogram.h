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
#include <memory>
#include <lib_atlas/macros.h>

namespace atlas {

template <typename Tp_>
class Histogram {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Histogram();

  explicit Histogram(const Histogram<Tp_> &rhs);

  explicit Histogram(Histogram<Tp_> &&rhs);

  explicit Histogram(std::vector<Tp_> const &data, double inter);

  explicit Histogram(std::vector<Tp_> const &data, unsigned int function);

  ~Histogram();

  //============================================================================
  // P U B L I C  O P E R A T O R S

  Histogram<Tp_> &operator=(const Histogram<Tp_> &rhs);

  Histogram<Tp_> &operator=(Histogram<Tp_> &&rhs);

  //============================================================================
  // P U B L I C  M E T H O D S

  /**
   * Return the index of the first occurence of the given value.
   *
   * If the value does not exists, this may throw a out of bound exception.
   *
   * \param value The value to look up in the histogram
   * \return The index of the first value occurence.
   */
  uint64_t Index(const Tp_ &value);

  /**
   * Return the value found on the given index.
   *
   * If the index is out of the histogram bound, this will throw an out
   * of bound exception.
   * Note that index "0" is the first element of the histogram.
   *
   * \param index The index to look up in the histogram
   * \return The value found at the index position.
   */
  Tp_ At(const uint64_t &index) const;

  /**
   * To get the maximum value of the histogram.
   *
   * \return the maximum value of the histogram in the same format of
   *  the data vector.
   */
  Tp_ Max() const ATLAS_NOEXCEPT;

  /**
   * To get the minimum value of the histogram.
   *
   * \return the minimum value of the histogram in the same format of
   *  the data vector.
   */
  Tp_ Min() const ATLAS_NOEXCEPT;

  /**
   * To add data in the histogram.
   *
   * Will add the data to the existing histogram.
   */
  void Add(const Tp_ &data) ATLAS_NOEXCEPT;

  /**
   * To zoom on a certain region of the histogram.
   *
   * \return a new histogram
   */
  std::shared_ptr<Histogram<Tp_>> ZoomOnValues(const Tp_ &begin,
                                               const Tp_ &end) ATLAS_NOEXCEPT;

  std::shared_ptr<Histogram<Tp_>> ZoomOnIndexes(
      const uint64_t &begin, const uint64_t &end) ATLAS_NOEXCEPT;

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
   * To find the occurencie of a value
   *
   * \return the number of occurencie.
   */
  uint64_t Count(const Tp_ &value) const ATLAS_NOEXCEPT;

 private:
  //============================================================================
  // P R I V A T E  M E T H O D S

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

  //============================================================================
  // P R I V A T E   M E M B E R S

  /**
   * TODO Antoine Dozois: Replace functions pointers by a delegated, for exemple
   * std::function.
   */
  unsigned int (*pfunc_inter_)(unsigned int);

  std::map<Tp_, uint64_t> histogram_;

  bool inter_func_;

  double inter_;
};

}  // namespace atlas

#include <lib_atlas/maths/histogram_inl.h>

#endif  // LIB_ATLAS_MATH_HISTOGRAM_H_
