#ifndef LOGGER_H
#define LOGGER_H

#include "../Processing/MotionEngine.h"

namespace Data {

class Logger {
public:
    Logger();
    
    // Prints a CSV formatted line of features and current system state
    void logFeatures(const MotionFeatures& f, MotionState state);
};

} // namespace Data

#endif // LOGGER_H
