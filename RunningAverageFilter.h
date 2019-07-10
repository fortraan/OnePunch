#ifndef RUNNING_AVERAGE_FILTER_H
#define RUNNING_AVERAGE_FILTER_H

template <typename T>
class RunningAverageFilter {
public:
  RunningAverageFilter(T startingValue) : RunningAverageFilter(startingValue, (uint8_t) 5) {}
  
  RunningAverageFilter(T startingValue, uint8_t maxSamples) :
    samples((T*) malloc(maxSamples * sizeof(T))),
    numSamples(1),
    maxSamples(maxSamples) {
    samples[0] = startingValue;
  }

  ~RunningAverageFilter() {
    free(samples);
  }

  T update(T newValue) {
    if (numSamples < maxSamples) numSamples++;
    memcpy(samples + 1, samples, sizeof(T) * maxSamples - 1);
    samples[0] = newValue;
    current = 0;
    for (uint8_t i = 0; i < numSamples; i++) {
      current += samples[i];
    }
    current /= numSamples;
    return current;
  }
  
  const T read() {
    return current;
  }
private:
  T current;
  T* samples;
  uint8_t numSamples;
  uint8_t maxSamples;
};

#endif
