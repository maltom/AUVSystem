#include <variant>

#include "external/jsonxx/jsonxx.h"

using UnprocessedFrame = jsonxx::Object;

struct ProcessedFrame
{
    double timeElapsed{0.0};
    double vx{0.0};
    
};

struct Frame
{
std::variant<UnprocessedFrame,ProcessedFrame> content;

bool isProcessed {false};
};