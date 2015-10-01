/**
 * \file	observer_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#include "gtest/gtest.h"
#include <lib_atlas/pattern/observer.h>
#include <lib_atlas/pattern/subject.h>

class ConcreateObserver : public atlas::Observer<const std::string &, int> {
 public:
  int i_ = {0};
  std::string s_ = {""};

 protected:
  auto OnSubjectNotify(atlas::Subject<const std::string &, int> &subject,
                       const std::string &str,
                       int nb) ATLAS_NOEXCEPT -> void override {
    s_ = str;
    i_ = nb;
  }
};

class ConcreteSubject : public atlas::Subject<const std::string &, int> {
 public:
  void DoSomething(std::string s, int i) { Notify(s, i); }
};

TEST(Observer, attachToSubject) {
  ConcreteSubject subject = {};
  ConcreateObserver observer = {};
  ConcreateObserver observer_2 = {};

  observer.Observe(subject);
  ASSERT_EQ(subject.ObserverCount(), 1);
  ASSERT_TRUE(observer.IsAttached(subject));
  ASSERT_THROW(observer.Observe(subject), std::invalid_argument);

  subject.Attach(observer_2);
  ASSERT_EQ(subject.ObserverCount(), 2);
  ASSERT_TRUE(observer_2.IsAttached(subject));
  ASSERT_ANY_THROW(subject.Attach(observer_2));
}

TEST(Observer, detachFromSubject) {
  ConcreteSubject subject = {};
  ConcreateObserver observer = {};

  observer.Observe(subject);
  subject.Detach(observer);
  ASSERT_EQ(subject.ObserverCount(), 0);
  ASSERT_FALSE(observer.IsAttached(subject));

  observer.Observe(subject);
  observer.DetachFromAllSubject();
  ASSERT_EQ(subject.ObserverCount(), 0);
  ASSERT_FALSE(observer.IsAttached(subject));

  observer.Observe(subject);
  subject.DetachAll();
  ASSERT_EQ(subject.ObserverCount(), 0);
  ASSERT_FALSE(observer.IsAttached(subject));

  ConcreateObserver observer_2 = {};
  observer_2.Observe(subject);
  ASSERT_EQ(subject.ObserverCount(), 1);
}

TEST(Observer, isAttached) {
  ConcreteSubject subject = {};
  ConcreateObserver observer = {};

  observer.Observe(subject);
  ASSERT_TRUE(observer.IsAttached(subject));

  observer.DetachFromAllSubject();
  ASSERT_FALSE(observer.IsAttached(subject));
}

TEST(Observer, notify) {
  ConcreteSubject subject = {};
  ConcreateObserver observer = {};

  observer.Observe(subject);
  subject.DoSomething("Hello World !", 42);
  ASSERT_TRUE(observer.s_ == "Hello World !");
  ASSERT_TRUE(observer.i_ == 42);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}