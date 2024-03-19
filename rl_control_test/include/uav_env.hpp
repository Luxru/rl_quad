#pragma once
#include "ros/ros.h"
#include "type.hpp"
#include <sys/types.h>

namespace sim {
    using namespace  flightlib;
    class Env{
    public:
        static const u_int8_t action_dim = 4, state_dim = 6, observation_dim = 6;
        Env(ros::NodeHandle& nh);
        void run();
        void reset();
        void step(const Vector<action_dim> &action, Vector<state_dim> &state, Vector<observation_dim> &observation, Scalar &reward, bool &terminal, bool &turncate);

    private:

    };

};