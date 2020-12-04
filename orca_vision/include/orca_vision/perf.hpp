#ifndef PERF_HPP
#define PERF_HPP

#define RUN_PERF
#ifdef RUN_PERF
#define START_PERF() \
  auto __start__ = std::chrono::high_resolution_clock::now();

#define STOP_PERF(msg) \
  auto __stop__ = std::chrono::high_resolution_clock::now(); \
  std::cout << msg << " " \
            << std::chrono::duration_cast<std::chrono::microseconds>(__stop__ - __start__).count() \
            << " microseconds" << std::endl;
#else
#define START_PERF()
#define STOP_PERF(msg)
#endif

#endif  // PERF_HPP