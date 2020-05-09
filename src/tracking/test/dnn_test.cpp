#include <benchmark/benchmark.h>
#include <random>
#include <iostream>

std::mt19937 rng;

void SomeFunction_rand(std::vector<int>& arr, std::vector<int>& idx) {
    int s = 0;
    for(auto i : idx) {
        s += arr[i];
    }
    // std::cout << s << std::endl;
}

void SomeFunction(std::vector<int>& arr) {
    int s = 0;
    for(int i = 0; i < (int) arr.size(); i++) {
        s += arr[i];
    }
    // std::cout << s << std::endl;
}

static void BM_SomeFunction_rand(benchmark::State& state) {
  // Perform setup here
  rng.seed(12345);
  std::vector<int> arr((int)1e5, 1), idx((int)1e5);
  std::uniform_int_distribution<uint32_t> uint_dist(0, (int) arr.size() - 1);
  for(auto& x : idx) {
      x = uint_dist(rng);
  }
  for (auto _ : state) {
    // This code gets timed
    SomeFunction_rand(arr, idx);
  }
}

static void BM_SomeFunction(benchmark::State& state) {
  // Perform setup here
  std::vector<int> arr((int)1e5, 1);
  for (auto _ : state) {
    // This code gets timed
    SomeFunction(arr);
  }
}
// Register the function as a benchmark
BENCHMARK(BM_SomeFunction_rand);
BENCHMARK(BM_SomeFunction);
// Run the benchmark
BENCHMARK_MAIN();
