#ifndef BASIC_TESTS_H
#define BASIC_TESTS_H

struct BasicTestSummary {
  int passed;
  int failed;
};

BasicTestSummary runBasicTests();

#endif  // BASIC_TESTS_H
