import sys
import os
import yaml

import xml.etree.ElementTree as ET

from .behavior_file import\
    BehaviorFile, BehaviorState, StateTransition, GoalSet
from .scene_file import\
    SceneFile, ObstacleSet, AgentProfile, AgentGroup
from .plugin_file import\
    Plugin
from .util import write_xml_file, write_xml_to_complete_file_path

from building_crowdsim.building_yaml_parse import\
    BuildingYamlParse, LevelWithHumanLanes
from building_map.level import Level


class ConfigFileGenerator:
    def __init__(self, building_yaml_parse):
        assert(isinstance(building_yaml_parse, BuildingYamlParse))
        # only level L1
        self.level_yaml = building_yaml_parse.yaml_node['levels']
        self.level = {}
        L1 = next(iter(self.level_yaml), -1)
        self.enable_crowdsim = 0
        if L1 != -1 and len(building_yaml_parse.crowd_sim_config) > 0:
            self.level[L1] = Level(self.level_yaml[L1], L1)
            self.level[L1].calculate_scale_using_measurements()
            # assume only level 1 for now
            self.crowd_sim_yaml = building_yaml_parse.crowd_sim_config[0]
            # assume only level 1 for now
            self.crowd_sim_human_yaml = building_yaml_parse.crowd_sim_human[0]
            if 'enable' not in self.crowd_sim_yaml:
                raise ValueError(
                    "Missing 'enable' tag for crowdsim configuration.")
            self.enable_crowdsim = int(self.crowd_sim_yaml['enable']) == 1
            self.human_goals = building_yaml_parse.get_human_goals()
            self.behavior_file = BehaviorFile()
            self.scene_file = SceneFile()
            self.plugin_file = Plugin()

    def generate_behavior_file(self, output_dir=""):
        # must follow the sequence:
        # 'states', 'transitions', 'goal_sets'
        if 'states' in self.crowd_sim_yaml:
            for state in self.crowd_sim_yaml['states']:
                cur_state = BehaviorState()
                cur_state.load_from_yaml(state)
                self.behavior_file.sub_elements.append(cur_state)
        if 'transitions' in self.crowd_sim_yaml:
            for transition in self.crowd_sim_yaml['transitions']:
                cur_transition = StateTransition()
                cur_transition.load_from_yaml(transition)
                self.behavior_file.sub_elements.append(cur_transition)
        if 'goal_sets' in self.crowd_sim_yaml:
            for goal_set in self.crowd_sim_yaml['goal_sets']:
                cur_goal_set = GoalSet()
                cur_goal_set.load_from_yaml(goal_set, self.human_goals)
                self.behavior_file.sub_elements.append(cur_goal_set)

        write_xml_file(
            self.behavior_file.output_xml_element(),
            output_dir=output_dir,
            file_name='behavior_file.xml')

    def transform_point_modify_yaml(self, yaml_node):
        x = yaml_node['x']
        y = yaml_node['y']
        yaml_node['x'], yaml_node['y'] = \
            self.level['L1'].transform.transform_point((x, -y))

    def generate_scene_file(self, output_dir):
        # add default configuration
        self.scene_file.add_spatial_query()
        self.scene_file.add_common()

        # must follow the sequence:
        # 'obstacle_set','agent_profiles','agent_groups'
        if 'obstacle_set' in self.crowd_sim_yaml:
            obstacle_set = ObstacleSet()
            obstacle_set.load_from_yaml(self.crowd_sim_yaml['obstacle_set'])
            self.scene_file.sub_elements.append(obstacle_set)
        if 'agent_profiles' in self.crowd_sim_yaml:
            for item in self.crowd_sim_yaml['agent_profiles']:
                cur_profile = AgentProfile()
                cur_profile.load_from_yaml(item)
                self.scene_file.sub_elements.append(cur_profile)
        if 'external_agent_groups' in self.crowd_sim_yaml:
            for item in self.crowd_sim_yaml['external_agent_groups']:
                cur_group = AgentGroup()
                cur_group.load_from_yaml(item)
                self.scene_file.sub_elements.append(cur_group)

        agent_groups = {}
        for item in self.crowd_sim_human_yaml:
            group_id = item['agent_group_id']
            if item['agent_group_id'] in agent_groups:
                self.transform_point_modify_yaml(item)
                agent_groups[group_id].load_an_agent_from_yaml(item)
            else:
                cur_group = AgentGroup()
                self.transform_point_modify_yaml(item)
                cur_group.load_an_agent_from_yaml(item)
                agent_groups[group_id] = cur_group

        for group_id, agent_group in agent_groups.items():
            self.scene_file.sub_elements.append(agent_group)

        write_xml_file(
            self.scene_file.output_xml_element(),
            output_dir=output_dir,
            file_name='scene_file.xml')

    def generate_plugin_file(self):
        self.plugin_file.load_from_yaml(self.crowd_sim_yaml,
                                        self.crowd_sim_human_yaml)

    def insert_plugin_into_world_file(self, world_file_to_be_inserted):
        self.generate_plugin_file()
        if not world_file_to_be_inserted:
            print("No world_file provided")
            return
        if not self.enable_crowdsim:
            print("Crowd Simulation is disabled. No plugin will be inserted.")
            return
        tmp = ET.parse(world_file_to_be_inserted)
        file_root = tmp.getroot()
        world_root = None
        for root_child in file_root:
            if root_child.tag != "world":
                continue
            world_root = root_child
        if not world_root:
            raise ValueError(
                "Invalid world file! please check your world file: ",
                world_file_to_be_inserted)

        plugin_already_inserted = []
        for world_child in world_root:
            # remove the possible crowd_simulation plugin previously inserted
            if world_child.tag == "plugin" and\
               world_child.attrib['name'] == "crowd_simulation":
                plugin_already_inserted.append(world_child)
        for world_child in plugin_already_inserted:
            world_root.remove(world_child)

        world_root.append(self.plugin_file.output_xml_element())
        write_xml_to_complete_file_path(
            file_root,
            file_name=world_file_to_be_inserted)
        print(
            "Insert <plugin> tag into ",
            world_file_to_be_inserted)


def configfile_main(map_file, output_dir, world_file_to_be_inserted):
    if not os.path.exists(map_file):
        raise ValueError('Map path not exist!: ' + map_file)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print("Create output dir for:", output_dir)

    if not os.path.exists(world_file_to_be_inserted):
        world_file_to_be_inserted = ""

    yaml_parse = BuildingYamlParse(map_file)
    configfile_generator = ConfigFileGenerator(yaml_parse)
    if configfile_generator.enable_crowdsim == 1:
        L1 = next(iter(configfile_generator.level_yaml))
        output_dir = output_dir + L1
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        configfile_generator.generate_behavior_file(output_dir)
        configfile_generator.generate_scene_file(output_dir)
        configfile_generator.insert_plugin_into_world_file(
            world_file_to_be_inserted)
