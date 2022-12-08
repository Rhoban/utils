#include "rhoban_utils/threading/multi_core.h"

#include <thread>
#include <stdexcept>

namespace rhoban_utils
{
MultiCore::Intervals MultiCore::buildIntervals(int nb_tasks, int nb_threads)
{
  /// Do not create more threads than tasks
  nb_threads = std::min(nb_tasks, nb_threads);
  /// There is two intervals possible, k and k+1
  int small_interval = nb_tasks / nb_threads;
  int big_interval = small_interval + 1;
  int nb_big_intervals = nb_tasks % nb_threads;
  /// Creating intervals
  int start = 0;
  Intervals result;
  for (int i = 0; i < nb_big_intervals; i++)
  {
    int end = start + big_interval;
    result.push_back(std::pair<int, int>(start, start + big_interval));
    start = end;
  }
  for (int i = 0; i < nb_threads - nb_big_intervals; i++)
  {
    int end = start + small_interval;
    result.push_back(std::pair<int, int>(start, start + small_interval));
    start = end;
  }
  return result;
}

void MultiCore::runParallelTask(Task t, int nb_tasks, int nb_threads)
{
  // Specific case for one thread, just use current thread
  if (nb_threads == 1)
  {
    t(0, nb_tasks);
    return;
  }
  MultiCore::Intervals intervals = buildIntervals(nb_tasks, nb_threads);
  std::vector<std::thread> threads;
  // Launch all threads
  for (size_t thread_no = 0; thread_no < intervals.size(); thread_no++)
  {
    int start = intervals[thread_no].first;
    int end = intervals[thread_no].second;
    threads.push_back(std::thread([t, start, end]() { t(start, end); }));
  }
  // Wait for all threads to finish
  for (size_t thread_no = 0; thread_no < intervals.size(); thread_no++)
  {
    threads[thread_no].join();
  }
}

void MultiCore::runParallelStochasticTask(StochasticTask st, int nb_tasks, int max_threads,
                                          std::default_random_engine* engine)
{
  // How much threads will be really used
  int nb_threads = max_threads;
  if (nb_tasks < max_threads)
  {
    nb_threads = nb_tasks;
  }
  // Initializing engines
  std::vector<std::default_random_engine> engines(nb_threads);
  unsigned int min = std::numeric_limits<unsigned int>::lowest();
  unsigned int max = std::numeric_limits<unsigned int>::max();
  std::uniform_int_distribution<unsigned int> seed_distrib(min, max);
  for (int i = 0; i < nb_threads; i++)
  {
    engines[i].seed(seed_distrib(*engine));
  }
  // Forwarding evaluation
  runParallelStochasticTask(st, nb_tasks, &engines);
}

void MultiCore::runParallelStochasticTask(StochasticTask st, int nb_tasks,
                                          std::vector<std::default_random_engine>* engines)
{
  if (engines == nullptr || engines->size() == 0)
  {
    throw std::runtime_error("MultiCore::runParallelStochasticTask with no engines");
  }
  int nb_threads = engines->size();
  // Specific case for one thread, just use current thread
  if (nb_threads == 1)
  {
    st(0, nb_tasks, &((*engines)[0]));
    return;
  }
  MultiCore::Intervals intervals = buildIntervals(nb_tasks, nb_threads);
  std::vector<std::thread> threads;
  // Launch all threads
  for (size_t thread_no = 0; thread_no < intervals.size(); thread_no++)
  {
    int start = intervals[thread_no].first;
    int end = intervals[thread_no].second;
    std::default_random_engine* thread_engine = &((*engines)[thread_no]);
    threads.push_back(std::thread([st, start, end, thread_engine]() { st(start, end, thread_engine); }));
  }
  // Wait for all threads to finish
  for (size_t thread_no = 0; thread_no < intervals.size(); thread_no++)
  {
    threads[thread_no].join();
  }
}

}  // namespace rhoban_utils
