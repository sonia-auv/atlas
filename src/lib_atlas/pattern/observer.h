/**
 * \file	observer.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIB_ATLAS_PATTERN_OBSERVER_H_
#define LIB_ATLAS_PATTERN_OBSERVER_H_

#include <type_traits>
#include <vector>
#include <functional>
#include <mutex>

#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/subject.h>

namespace atlas {

template <typename... Args_>
class Observer {
  // The callback on the Observer class are private by default.
  // We don't want other class than the Subject to be able to call the
  // OnSubjectXXX() method.
  friend class Subject<Args_...>;

 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Observer() ATLAS_NOEXCEPT;

  explicit Observer(Subject<Args_...> &subject) ATLAS_NOEXCEPT;

  /**
   * Copy ctor of an observer. This will attach this instance of an Observer
   * to all listened subject of the passed observer.
   *
   * \param rhs The Observer base used to create this instance.
   */
  explicit Observer(const Observer<Args_...> &rhs) ATLAS_NOEXCEPT;

  virtual ~Observer() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   O P E R A T O R S

  /**
   * Copy ctor of an observer. This will attach this instance of an Observer
   * to all listened subject of the passed observer.
   *
   * \param rhs The Observer base used to create this instance.
   */
  void operator=(const Observer<Args_...> &rhs) ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C  M E T H O D S

  void Observe(Subject<Args_...> &subject);

  bool IsAttached(const Subject<Args_...> &subject) const ATLAS_NOEXCEPT;

  void DetachFromAllSubject() ATLAS_NOEXCEPT;

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * This method is used if you want to use the observer without delegates.
   * If so, then the method will be called instead of the delegate.
   * If not, then simply override this and do nothing.
   */
  virtual void OnSubjectNotify(Subject<Args_...> &subject,
                               Args_... args) ATLAS_NOEXCEPT = 0;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  virtual void OnSubjectConnected(Subject<Args_...> &subject) ATLAS_NOEXCEPT;

  virtual void OnSubjectDisconnected(Subject<Args_...> &subject);

  //============================================================================
  // P R I V A T E   M E M B E R S

  std::vector<Subject<Args_...> *> subjects_;

  mutable std::mutex subjects_mutex_;
};

}  // namespace atlas

#include <lib_atlas/pattern/observer_inl.h>

#endif  // LIB_ATLAS_PATTERN_OBSERVER_H_
