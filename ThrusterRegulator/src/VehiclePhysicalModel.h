#pragma once

#include <fstream>
#include <memory>

#include "jsonxx/jsonxx.h"


class VehiclePhysicalModel final
{
public:
    VehiclePhysicalModel(std::fstream& config)
    {
        loadParameters();
    }
private:
void loadParameters();
};