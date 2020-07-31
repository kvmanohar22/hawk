#include "svo/ring_buffer_imu.h"

namespace svo
{

RingBufferImu::RingBufferImu(int size) :
    arr_(size),
    begin_(0),
    end_(-1),
    num_elem_(0),
    arr_size_(size)
{}

bool RingBufferImu::empty() const
{
  return arr_.empty();
}

void RingBufferImu::push_back(const ImuDataPtr & elem)
{
  {
    lock_t lock(arr_mut_);

    if(num_elem_<arr_size_)
    {
      end_++;
      arr_[end_] = elem;
      num_elem_++;
    }
    else{
      end_ = (end_+1)%arr_size_;
      begin_ = (begin_+1)%arr_size_;
      arr_[end_] = elem;
    }
  }
}

ImuDataPtr RingBufferImu::get(int i)
{
  assert(i<num_elem_);
  return arr_[i % arr_size_];
}

int RingBufferImu::findIdxSmall(const double& t)
{
  int low, high;
  if(arr_[0]->ts_ < t)
  {
    low  = 0;
    high = writeIdx();
  } else {
    low  = readIdx();
    high = num_elem_ < arr_size_ ? num_elem_-1 : arr_size_-1;
  }

  while(true)
  {
    int mid = (low+high)/2;

    // cout.precision(std::numeric_limits<double>::max_digits10);
    // cout << "t = " << t
    //      << "\t low TS = " << arr_[low]->ts_
    //      << "\t mid TS = " << arr_[mid]->ts_
    //      << "\t high TS = " << arr_[high]->ts_
    //      << "\t low = " << low << "\t mid = " << mid << "\t high = " << high
    //      << endl;

    if(low == high)
    {
      double tlow = arr_[low]->ts_;
      int low_minus = low-1 < 0 ? arr_size_-1 : low-1;
      if(arr_[low]->ts_ <= t)
        return low;
      else if(arr_[low_minus]->ts_ <= t)
        return low_minus;
      else
        return -1;
    }


    if(t < arr_[mid]->ts_)
      high = mid;
    else
      low = mid+1;
  }
}

int RingBufferImu::findIdxLarge(const double& t)
{
  int low, high;
  if(arr_[0]->ts_ < t)
  {
    low  = 0;
    high = writeIdx();
  } else {
    low  = readIdx();
    high = num_elem_ < arr_size_ ? num_elem_-1 : arr_size_-1;
  }

  while(true)
  {
    if(low == high)
    {
      double tlow = arr_[low]->ts_;
      int high_plus = high+1 >= arr_size_ ? 0 : high+1;
      if(arr_[low]->ts_ >= t)
        return low;
      else if(arr_[high_plus]->ts_ >= t)
        return high_plus;
      else
        return -1;
    }

    int mid = (low+high)/2;
    if(t > arr_[mid]->ts_)
      low = mid+1;
    else
      high = mid;
  }
}

list<ImuDataPtr> RingBufferImu::read(const double& t0, const double& t1)
{
  list<ImuDataPtr> data;
  {
    lock_t lock(arr_mut_);

    const int small_idx = findIdxSmall(t0);
    const int large_idx = findIdxLarge(t1);
    const double tmin = tMin();
    const double tmax = tMax();
    // cout.precision(std::numeric_limits<double>::max_digits10);
    // cout << "tmin = " << tmin << "\t tmax = " << tmax << "\n"
    //      << "t0   = " << t0   << "\t t1   = " << t1
    //      << "\t small = " << small_idx << "\t large idx = " << large_idx << endl;      

    if(small_idx == -1 || large_idx == -1)
    {
      SVO_WARN_STREAM("Could not find required messages in Ring Buffer");
      return data;
    }

    for(int idx = small_idx; idx <= large_idx; ++idx)
    {
      ImuDataPtr packet = get(idx);
      data.push_back(packet);
    }
    SVO_DEBUG_STREAM("dt_min = " << std::abs(data.front()->ts_-t0) << "\t dt_max = " << std::abs(data.back()->ts_-t1));
  }
  return data;
}

} // namespace svo
