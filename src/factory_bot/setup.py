import os
from glob import glob
from setuptools import setup

package_name = "factory_bot"


def files(pattern):
    """All files (not folders) matching a pattern, e.g. 'config/*.yaml'."""
    return [f for f in glob(pattern) if os.path.isfile(f)]


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", files("launch/*.py")),
        ("share/" + package_name + "/config", files("config/*.yaml")),
        ("share/" + package_name + "/config/scenarios", files("config/scenarios/*.yaml")),
        ("share/" + package_name + "/urdf", files("urdf/*")),
        ("share/" + package_name + "/worlds", files("worlds/*")),
        ("share/" + package_name + "/maps", files("maps/*")),
        ("share/" + package_name + "/rviz", files("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="Two-robot factory material delivery fleet",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_mover = factory_bot.obstacle_mover:main",
            "mapping_driver = factory_bot.mapping_driver:main",
            "map_viewer = factory_bot.map_viewer:main",
            "ogm_mapper = factory_bot.ogm_mapper:main",
            "mapping_check = factory_bot.mapping_check:main",
            "compare_maps = factory_bot.compare_maps:main",
            "odom_test = factory_bot.odom_test:main",
            "icp_localizer = factory_bot.icp_localizer:main",
            "loc_check = factory_bot.loc_check:main",
            "show_grid = factory_bot.show_grid:main",
            "plan_benchmark = factory_bot.plan_benchmark:main",
            "track_benchmark = factory_bot.track_benchmark:main",
            "navigator = factory_bot.navigator:main",
            "nav_test = factory_bot.nav_test:main",
            "fleet_manager = factory_bot.fleet_manager:main",
            "send_job = factory_bot.send_job:main",
            "call_emergency = factory_bot.call_emergency:main",
        ],
    },
)
