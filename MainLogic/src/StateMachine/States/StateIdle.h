#pragma once

#include "StateBase.h"

class StateIdle : public StateBase
{
    private:

    protected:

    public:
    StateIdle()=default;
    void process() override;
};