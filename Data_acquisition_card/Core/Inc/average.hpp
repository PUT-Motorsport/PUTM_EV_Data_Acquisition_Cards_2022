#pragma once

#include <type_traits>

namespace {
constexpr std::size_t MAX_SAMPLES{10};
}

template<typename T>
requires (std::is_arithmetic<T>::value)
struct Average {
public:
	auto addSample(T sample) {
		if (samplesCount < MAX_SAMPLES) {
			++samplesCount;
			samples[samplesCount] = sample;
		}
		return;
	}
	auto reset() {
		samplesCount = 0;
		return;
	}
	[[nodiscard]]
	 operator T() {	//lazy evaluate the average
		T sum{};

		if (samplesCount == 0) {
			Device::setState(State::IOSPIError);
		}

		for (std::size_t iter = 0; iter < samplesCount; ++iter) {
			sum += samples[iter];
		}
		reset();
		return sum / samplesCount;
	}
private:
	T samples[MAX_SAMPLES];
	std::size_t samplesCount;
};
