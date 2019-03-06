#pragma once

#include <mutex>
#include <queue>
#include <condition_variable>

template <typename T>
class sync_queue
{
public:
	sync_queue() {}
	~sync_queue() {}

	void push(const T& a) {
		std::unique_lock<std::mutex> lock(m_);
		q_.push(a);
		lock.unlock();
		cv_.notify_one();
	}

	void pop(T& a) {
		std::unique_lock<std::mutex> lock(m_);
		while (q_.empty()) {
			cv_.wait(lock);
		}
		a = q_.front();
		q_.pop();
	}

	int size() {
		return q_.size();
	}

	bool empty() {
		return q_.empty();
	}

private:
	std::queue<T> q_;
	std::mutex m_;
	std::condition_variable cv_;
};

