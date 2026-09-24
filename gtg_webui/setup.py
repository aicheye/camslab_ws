from setuptools import setup

package_name = "gtg_webui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "aiohttp"],
    zip_safe=True,
    maintainer="Sean Yang",
    maintainer_email="seanxxyang@gmail.com",
    description="Web UI shared by the go-to-goal Python sim and the ROS 2 nodes.",
    license="Proprietary",
    entry_points={
        "console_scripts": ["webui_bridge_node = gtg_webui.webui_bridge_node:main"],
    },
)
