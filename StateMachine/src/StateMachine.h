#include <array>
#include <fstream>
#include <memory>

#include "jsonxx/jsonxx.h"
#include <ros/ros.h>

class StateMachine final
{
public:
    StateMachine(std::shared_ptr<ros::NodeHandle> &node, std::fstream &config):rosNode(node), configFile(config)
    {
        file.parse(configFile);

    }
    ~StateMachine()
    {}

private:
    std::shared_ptr<ros::NodeHandle> &rosNode;
    std::fstream &configFile;
    jsonxx::Object file;
    

    void subscribeTopics() const;
    void advertiseTopics() const;
    void connectServices() const;
};