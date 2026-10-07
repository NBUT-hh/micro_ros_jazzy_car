from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xuegecar_voice_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huanghao',
    maintainer_email='huanghao@todo.todo',
    description='语音/指令导航：地点名 -> Nav2 目标',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'goto_node = xuegecar_voice_nav.goto_node:main',
            'voice_node = xuegecar_voice_nav.voice_node:main',
            'voice_word_node = xuegecar_voice_nav.voice_word_node:main',
            'vision_node = xuegecar_voice_nav.vision_node:main',
        ],
    },
)
