#ifndef _RING_BUFFER_IMU_H
#define _RING_BUFFER_IMU_H

#include <vector>
#include <cassert>
#include <numeric>
#include <mutex>
#include "svo/global.h"
#include "svo/imu_data.h"

namespace svo
{

class RingBufferImu
{
public:
  typedef std::unique_lock<std::mutex> lock_t;

  RingBufferImu(int size);
  void push_back(const ImuDataPtr& elem);
  bool empty() const;
  ImuDataPtr get(int i);
  list<ImuDataPtr> read(const double& t0, const double& t1);

  /// Performs binary search and finds idx that is immediately lesser than `t`
  int findIdxSmall(const double& t);

  /// Performs binary search and finds idx that is immediately greater than `t`
  int findIdxLarge(const double& t);

  /// Min. time from which data is available in buffer. **NOT THREADSAFE**
  inline double tMin() const { return arr_[begin_]->ts_; }

  /// Max. time upto which data is available in buffer. **NOT THREADSAFE**
  inline double tMax() const { return arr_[end_]->ts_;   }

  inline int size()     const { return num_elem_; }
  inline int readIdx()  const { return begin_; }
  inline int writeIdx() const { return end_; }

private:
  std::vector<ImuDataPtr> arr_;
  int begin_;
  int end_;
  int num_elem_;
  int arr_size_;

  // thread safety feature for read/write
  std::mutex arr_mut_;
};

} // namespace svo

#endif
