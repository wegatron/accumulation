#ifndef DATA_CACHE_H
#define DATA_CACHE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <iterator>
#include <cassert>
#include <functional>

/**
 * Use LRU policy to swap out data to disk to reduce memory usage.
 * Data element should be large enough, as we need extra data to save every element's state.
 * Notice: Not thread safe.
 */
template<typename T>
class data_cache
{
public:

	data_cache(int buffer_num, int total_size): 
		num_in_memory_(0), buffer_num_(buffer_num),
		check_id_(-1), data_index2lru_(total_size, lru_.end()),
		data_(total_size, nullptr), offset_(total_size, -1),
		element_write_out_func_([](std::fstream &fs, T *t_ptr){ fs.write(reinterpret_cast<char*>(t_ptr), sizeof(T)); }),
		element_load_func_([](std::fstream &fs, T *t_ptr){ fs.read(reinterpret_cast<char*>(t_ptr), sizeof(T)); })
	{
		fs_.open("cache.bin", std::ios::trunc | std::ios::out | std::ios::in | std::ios::binary);
		if(!fs_)
		{
			std::cerr << "unable to open cache file for write" << std::endl;
		}
	}

	/**
	 * get data if is null allocate it
	 */
	std::shared_ptr<T> & get(const int id)
	{
		// if the element visited change from null to not null(new data allocated)
		// then check if need swap out old element and make latest for check id
		if(check_id_ != -1) {
			if(data_[check_id_] != nullptr) {
				if(num_in_memory_ < buffer_num_) ++num_in_memory_;
				else swap_out_oldest();
				make_latest(check_id_);
			}
			check_id_ = -1;
		}

		if(id >= data_.size())
		{
			std::cerr << "index out of range!!!" << std::endl;
			throw std::overflow_error("index out of range");
		}

		if (data_[id] == nullptr) {
			if (offset_[id] == -1)
			{
				check_id_ = id; // to check
			} else {
				data_[id] = swap_out_oldest();
				fs_.seekg(offset_[id], fs_.beg);
				element_load_func_(fs_, data_[id].get());
			}
		}
		if(data_[id] != nullptr) make_latest(id);
		return data_[id];
	}

	std::shared_ptr<T> & operator[] (const int id)
	{
		return get(id);
	}

	// function for debug
	void print_lru()
	{
		std::copy(lru_.begin(), lru_.end(), std::ostream_iterator<int>(std::cout, " "));
		std::cout.flush();
	}

private:

	void make_latest(const int id)
	{
		// new element
		if(data_index2lru_[id] != lru_.end())
			lru_.erase(data_index2lru_[id]);
		lru_.push_front(id);
		data_index2lru_[id] = lru_.begin();
	}

	std::shared_ptr<T> swap_out_oldest()
	{
		assert(lru_.size() >= buffer_num_);
		int id = lru_.back();
		lru_.pop_back();
		// save to file system
		if(offset_[id] == -1) {
			fs_.seekp(0, fs_.end);
			offset_[id] = fs_.tellp();
		} else fs_.seekp(offset_[id], fs_.beg);
		element_write_out_func_(fs_, data_[id].get());
		std::shared_ptr<T> ret = data_[id];
		data_[id].reset();
		data_index2lru_[id] = lru_.end();
		return ret;
	}

	int num_in_memory_; //!< current element number in memory
	int buffer_num_; //!< buffer size 
	int check_id_; //!< the data index to check, if change from null to not null(filled with data)
	std::fstream fs_; //!< stream for write and read
	std::list<int> lru_; //!< lru list
	std::vector<std::list<int>::iterator> data_index2lru_; //!< map from data index to lru iterator
	std::vector<std::shared_ptr<T>> data_; //!< element vector
	std::vector<unsigned long long> offset_;//!< file offset of element write to
	std::function<void(std::fstream &fs, T *t)> element_write_out_func_;
	std::function<void(std::fstream &fs, T *t)> element_load_func_;
};

#endif //DATA_CACHE_H