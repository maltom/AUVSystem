#include <memory>

#include "StateMachine.h"

class Vehicle final
{

    Vehicle();
    ~Vehicle() = default;
private:
    std::unique_ptr< StateMachine > statemachine;
protected:

public:
void initialize();

};