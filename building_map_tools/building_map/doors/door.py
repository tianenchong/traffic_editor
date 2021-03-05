import math
from xml.etree.ElementTree import Element, SubElement
from..utils import door_material, box_link, joint, lidar_sensor


class Door:
    def __init__(self, door_edge, level_elevation=0.0):
        self.name = door_edge.params['name'].value
        self.type = door_edge.params['type'].value
        if 'plugin' in door_edge.params:
            self.plugin = door_edge.params['plugin'].value
        else:
            self.plugin = 'normal'

        self.length = door_edge.length
        self.cx = door_edge.x
        self.cy = door_edge.y
        self.cz = level_elevation
        self.yaw = door_edge.yaw
        self.height = 2.2  # parameterize someday?
        self.thickness = 0.03  # parameterize someday?
        print(f'Door({self.name})')

        self.model_ele = Element('model')
        self.model_ele.set('name', self.name)
        pose_ele = SubElement(self.model_ele, 'pose')
        pose_ele.text = f'{self.cx} {self.cy} {self.cz} 0 0 {self.yaw}'

        if self.plugin == 'none':
            static_ele = SubElement(self.model_ele, 'static')
            static_ele.text = 'true'

    def generate_section(self, name, width, x_offset, options):
        pose_ele = Element('pose')
        pose_ele.text = f'{x_offset} 0 {self.height/2+0.01} 0 0 0'
        size = [width, self.thickness, self.height]
        link_ele = box_link(name,
                            size,
                            pose_ele,
                            material=door_material(options),
                            bitmask='0x02')

        mass = 50.0
        inertial_ele = SubElement(link_ele, 'inertial')
        mass_ele = SubElement(inertial_ele, 'mass')
        mass_ele.text = str(mass)
        inertia_ele = SubElement(inertial_ele, 'inertia')
        SubElement(inertia_ele, 'ixx').text = \
            str(mass/12.0*(self.thickness**2 + self.height**2))
        SubElement(inertia_ele, 'iyy').text = \
            str(mass/12.0*(width**2 + self.height**2))
        SubElement(inertia_ele, 'izz').text = \
            str(mass/12.0*(self.thickness**2 + width**2))

        self.model_ele.append(link_ele)
        return link_ele

    def generate_sliding_section(self, name, width, x_offset, bounds, options):
        self.generate_section(name, width, x_offset, options)

        self.model_ele.append(joint(f'{name}_joint',
                                    'prismatic',
                                    'world',
                                    name,
                                    joint_axis='x',
                                    lower_limit=bounds[0],
                                    upper_limit=bounds[1],
                                    max_effort=500.0))

        if self.plugin == 'automatic':
            z = 2.5
            # ray pitch angle
            vertical = math.pi/2  # 90 degree clockwise
            slanted = 1.3
            # ray yaw orientation angle
            door_yaw = -(1/2)*math.pi
            pose_ele1 = Element('pose')
            pose_ele1.text = f'-0.03 0 {z} 0 {slanted} {-door_yaw}'
            pose_ele2 = Element('pose')
            pose_ele2.text = f'0.03 0 {z} 0 {slanted} {door_yaw}'
            self.model_ele.append(lidar_sensor(
                self.name, num=1, pose=pose_ele1))
            self.model_ele.append(lidar_sensor(
                self.name, num=2, pose=pose_ele2))

    '''Generate a single swing section/panel of a door.

    name = name of the door section
    width = width of the door section
    x_offset = offset of the center of the door section from the center
               of the entire door (this will be non-zero for double doors)
    bounds = bounds for the range of motion of this section, in radians
    axis = pose of the joint axis, in the door *section* frame
    '''

    def generate_swing_section(
        self,
        name,
        width,
        x_offset,
        bounds,
        axis,
        options
    ):
        self.generate_section(name, width, x_offset, options)

        pose_ele = Element('pose')
        pose_ele.text = f'{axis[0]} {axis[1]} {axis[2]} 0 0 0'
        self.model_ele.append(joint(f'{name}_joint',
                                    'revolute',
                                    'world',
                                    name,
                                    joint_axis='z',
                                    lower_limit=bounds[0],
                                    upper_limit=bounds[1],
                                    pose=pose_ele))
        if self.plugin == 'automatic':
            z = 2.5
            # ray pitch angle
            vertical = math.pi/2  # 90 degree clockwise
            slanted = 1.3
            # ray yaw orientation angle
            door_yaw = -(1/2)*math.pi
            pose_ele1 = Element('pose')
            if name == 'left':
                pose_ele1.text = f'-0.2 0.7 {z} 0 {vertical} {(bounds[1]+bounds[0])/2}'
            else:
                pose_ele1.text = f'-0.03 0 {z} 0 {slanted} {-door_yaw}'
            pose_ele2 = Element('pose')
            if name == 'left':
                pose_ele2.text = f'-0.03 0 {z} 0 {slanted} {door_yaw}'
            else:
                pose_ele2.text = f'-0.2 -0.7 {z} 0 {vertical} {(bounds[1]+bounds[0])/2}'
            self.model_ele.append(lidar_sensor(
                self.name, num=1, pose=pose_ele1))
            self.model_ele.append(lidar_sensor(
                self.name, num=2, pose=pose_ele2))
