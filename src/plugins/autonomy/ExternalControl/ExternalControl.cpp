/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/DelayedTask.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControl.h>
#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControlClient.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/motion/MotionModel.h>
#include <scrimmage/math/State.h>
#include <scrimmage/sensor/Sensor.h>

#include <grpc++/grpc++.h>

#include <limits>

#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>

#include <chrono> // NOLINT
#include <thread> // NOLINT

namespace sp = scrimmage_proto;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ExternalControl, ExternalControl_plugin)

namespace scrimmage {
namespace autonomy {

ExternalControl::ExternalControl() :
    external_control_client_(std::make_shared<ExternalControlClient>()),
    delayed_task_(std::make_shared<DelayedTask>()) {}

void ExternalControl::init(std::map<std::string, std::string> &params) {
    server_address_ = get("server_address", params, std::string("localhost:50051"));
    delayed_task_->delay = get("timestep", params, 0.0);
    num_attempts_ = get("num_attempts", params, 5);
    use_trained_model_ = get("use_trained_model", params, false);
    if (use_trained_model_) {
        std::string xml_file = params.at("XML_FILENAME");
        std::string actor = params.at("actor_name");

        std::vector<std::string> address_parts;
        // boost::split(address_parts, params.at("server_address"), [](char c){return c == ':';});

        split(address_parts, params.at("server_address"), ":");
        std::string port_num = address_parts.at(1);
        python_cmd_ = std::string("external_control.py")
            + " --port-num " + port_num
            + " --actor " + actor
            + " --xml " + xml_file
            + " &";

        int result = system(python_cmd_.c_str());
        if (result) {
            std::cout << "Error occurred in running python script." << std::endl;
            std::cout <<"Make sure SCRIMMAGE python bindings are up to date" << std::endl;
        }
        // Give python time to startup GRPC Server before continuing
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

bool ExternalControl::step_autonomy(double t, double dt) {
    boost::optional<scrimmage_proto::Action> action = boost::none;
    if (!env_sent_) {
        *external_control_client_ =
            ExternalControlClient(grpc::CreateChannel(
                server_address_, grpc::InsecureChannelCredentials()));
        env_sent_ = true;
        int attempts = 0;
        bool done = true;
        while (attempts++ < num_attempts_ && done) {
            done = !send_env();
        }
        // openai does not send a reward when reset so just send the state
        sp::ActionResult action_result = get_observation(t);
        action = send_action_result(action_result, 0, done);
    }

    bool done;
    double reward;
    std::tie(done, reward) = calc_reward(t);
    curr_reward += reward;

    bool update_action = delayed_task_->update(t).first;
    if (done || update_action || action) {
        if (!action) {
            sp::ActionResult action_result = get_observation(t);
            action = send_action_result(action_result, curr_reward, done);
        }
        curr_reward = 0;
        if (!action) {
            // std::cout << "did not receive external action. exiting." << std::endl;
            return false;
        } else {
            return handle_action(t, dt, *action);
        }
    } else {
        return true;
    }
}

std::pair<bool, double> ExternalControl::calc_reward(double t) {
    return {false, 0.0};
}

bool ExternalControl::handle_action(
        double t, double dt, const scrimmage_proto::Action &action) {
    if (!check_action(action, 0, 9)) {
        return false;
    }
    desired_state_->pos() <<
        action.continuous(0), action.continuous(1), action.continuous(2);
    desired_state_->vel() <<
        action.continuous(3), action.continuous(4), action.continuous(5);
    desired_state_->quat().set(
        action.continuous(6), action.continuous(7), action.continuous(8));
    return true;
}

scrimmage_proto::ActionResult ExternalControl::get_observation(double t) {
    sp::ActionResult action_result;
    sp::SpaceSample *obs = action_result.mutable_observations();

    for (auto &kv : parent_->sensors()) {
        auto msg = kv.second->sensor_msg_flat(t);
        if (msg) {
            sp::SpaceSample &sample = msg->data;
            for (int i = 0; i < sample.value_size(); i++) {
                obs->add_value(sample.value(i));
            }
        }
    }

    return action_result;
}

boost::optional<scrimmage_proto::Action>
ExternalControl::send_action_result(
        scrimmage_proto::ActionResult &action_result,
        double reward, bool done) {
    action_result.set_reward(reward);
    action_result.set_done(done);
    return external_control_client_->send_action_result(action_result);
}

bool ExternalControl::check_action(
        const scrimmage_proto::Action &action,
        uint64_t discrete_action_size,
        uint64_t continuous_action_size) {
    if (action.done()) {
        std::cout << "received done signal from external controller" << std::endl;
        return false;
    } else if (static_cast<uint64_t>(action.discrete_size()) != discrete_action_size) {
        std::cout << "received discrete external action of length "
            << action.discrete_size() << " (need length "
            << discrete_action_size << ")" << std::endl;
    } else if (static_cast<uint64_t>(action.continuous_size()) != continuous_action_size) {
        std::cout << "received continuous external action of length "
            << action.continuous_size() << " (need length "
            << continuous_action_size << ")" << std::endl;
        return false;
    }
    return true;
}

bool ExternalControl::send_env() {
    sp::Environment env;

    *env.mutable_action_spaces() = action_space_params();

    sp::SpaceParams *obs_space = env.mutable_observation_spaces();

    for (auto &kv : parent_->sensors()) {
        auto obs_space_params = kv.second->observation_space_params();
        for (const sp::SingleSpaceParams params : obs_space_params.params()) {
            *obs_space->add_params() = params;
        }
    }

    env.set_min_reward(min_reward_);
    env.set_max_reward(max_reward_);

    return external_control_client_->send_environment(env);
}

scrimmage_proto::SpaceParams ExternalControl::action_space_params() {
    sp::SpaceParams space_params;
    sp::SingleSpaceParams *single_space_params = space_params.add_params();
    single_space_params->set_num_dims(9);
    single_space_params->set_discrete(false);

    for (int i = 0; i < 6; i++) {
        // we have to use non-infinite bounds for openai's tuple_space to give
        // non-nan values when the sample method on tuple_space is called
        const double inf = std::numeric_limits<float>::max();
        single_space_params->add_minimum(-inf);
        single_space_params->add_maximum(inf);
    }

    // roll
    single_space_params->add_minimum(-M_PI);
    single_space_params->add_maximum(M_PI);

    // pitch
    single_space_params->add_minimum(-M_PI / 2);
    single_space_params->add_maximum(M_PI / 2);

    // yaw
    single_space_params->add_minimum(-M_PI / 2);
    single_space_params->add_maximum(M_PI / 2);

    return space_params;
}

void ExternalControl::close(double t) {
    sp::ActionResult action_result = get_observation(t);
    const double reward = 0.0;
    const bool done = true;
    send_action_result(action_result, reward, done);
}

} // namespace autonomy
} // namespace scrimmage
