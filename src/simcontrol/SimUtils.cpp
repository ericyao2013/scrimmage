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

#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/plugin_manager/PluginManager.h>
#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/simcontrol/SimUtils.h>

#include <iostream>

namespace scrimmage {

bool create_ent_inters(MissionParsePtr mp,
                       PluginManagerPtr plugin_manager,
                       FileSearchPtr file_search,
                       RandomPtr random,
                       PubSubPtr pubsub,
                       TimePtr time,
                       std::shared_ptr<std::unordered_map<int, int>> id_to_team_map,
                       std::shared_ptr<std::unordered_map<int, EntityPtr>> id_to_ent_map,
                       std::list<scrimmage_proto::ShapePtr> &shapes,
                       std::list<EntityInteractionPtr> &ent_inters) {

    for (std::string ent_inter_name : mp->entity_interactions()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            mp->attributes()[ent_inter_name];
        EntityInteractionPtr ent_inter =
            std::dynamic_pointer_cast<EntityInteraction>(
                plugin_manager->make_plugin("scrimmage::EntityInteraction",
                                            ent_inter_name, *file_search,
                                            config_parse, overrides));

        if (ent_inter == nullptr) {
            std::cout << "Failed to load entity interaction plugin: "
                 << ent_inter_name << std::endl;
            return false;
        }

        ent_inter->set_random(random);
        ent_inter->set_mission_parse(mp);
        ent_inter->set_projection(mp->projection());
        ent_inter->set_pubsub(pubsub);
        ent_inter->set_time(time);
        ent_inter->set_id_to_team_map(id_to_team_map);
        ent_inter->set_id_to_ent_map(id_to_ent_map);
        ent_inter->set_name(ent_inter_name);
        ent_inter->init(mp->params(), config_parse.params());

        // Get shapes from plugin
        shapes.insert(
            shapes.end(), ent_inter->shapes().begin(), ent_inter->shapes().end());
        ent_inter->shapes().clear();

        ent_inters.push_back(ent_inter);
    }

    return true;
}

bool create_metrics(MissionParsePtr mp,
                    PluginManagerPtr plugin_manager,
                    FileSearchPtr file_search,
                    PubSubPtr pubsub,
                    TimePtr time,
                    std::shared_ptr<std::unordered_map<int, int>> id_to_team_map,
                    std::shared_ptr<std::unordered_map<int, EntityPtr>> id_to_ent_map,
                    std::list<MetricsPtr> &metrics_list) {

    for (std::string metrics_name : mp->metrics()) {
        ConfigParse config_parse;
        std::map<std::string, std::string> &overrides =
            mp->attributes()[metrics_name];
        MetricsPtr metrics =
            std::dynamic_pointer_cast<Metrics>(
                plugin_manager->make_plugin(
                    "scrimmage::Metrics", metrics_name,
                    *file_search, config_parse, overrides));

        if (metrics == nullptr) {
            std::cout << "Failed to load metrics: " << metrics_name << std::endl;
            return false;
        }

        metrics->set_id_to_team_map(id_to_team_map);
        metrics->set_id_to_ent_map(id_to_ent_map);
        metrics->set_pubsub(pubsub);
        metrics->set_time(time);
        metrics->set_name(metrics_name);
        metrics->init(config_parse.params());
        metrics_list.push_back(metrics);
    }

    return true;
}

void run_callbacks(PluginPtr plugin) {
    for (auto &sub : plugin->subs()) {
        for (auto msg : sub->pop_msgs<MessageBase>()) {
            sub->accept(msg);
        }
    }
}
} // namespace scrimmage
