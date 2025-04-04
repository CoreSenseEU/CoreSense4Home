# Copyright (c) 2025 TODO. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ament_index_python as aip
import xml.etree.ElementTree as ET
import json
import yaml


def get_manifest(pkg_name):
    """
    Return the manifest of a package, if it exports a skill, task or mission.

    :param pkg_name: the name of the package
    :returns: a tuple (type of component, manifest dictionary)
    """
    path = aip.get_package_share_path(pkg_name)

    package_xml = path / 'package.xml'
    if not package_xml.exists():
        return None, None

    tree = ET.parse(package_xml)
    root = tree.getroot()
    export = root.find('export')
    if export is not None:
        for type in ["skill", "task", "mission", "app"]:
            res = export.find(type)
            if res is not None:
                content_type = res.attrib.get('content-type', 'json')
                content = res.text
                if not content.strip():
                    print(f"Error while reading the manifest of {type} {pkg_name} "
                          f"({package_xml}): empty manifest! Skipping.")
                    continue
                if content_type == 'json':
                    try:
                        manifest = json.loads(content)
                        return type, manifest
                    except json.decoder.JSONDecodeError as jde:
                        print(f"Error while reading the manifest of {type} {pkg_name} "
                              f"(from {package_xml}): {jde}. Skipping.")
                        continue
                elif content_type == 'yaml':
                    manifest = yaml.safe_load(content)
                    return type, manifest
    return None, None


def get_component_dependencies(pkg_name):
    """
    Return the list of tasks/skills that this application depends on.

    Process:

    1. get the package.xml of the package
    2. inside the <export> tag, check if the package is a <skill>, <task> or
    <mission> by checking for the presence of any of these tags.
    3. get the content of the tag (ie, the manifest of the package), and parse
    it as a json or yaml content, depending on the value of the content-type
    attribute.
    4. retrieve the 'dependencies' key of the resulting dictionary. This
    dictionary is of the form {"skills": ["skill1", "skill2", ...], "tasks":
    ["task1", "task2",...], "mission": [...]}
    5. using ament_index, get the list of all installed packages
    6. parse each of their package.xml
    7. check if they export a <mission>, <skill> or <task>, and find the ones
    matching the required dependencies
    8. return the list of package dependencies, with their manifests.

    :param pkg_name: the name of the package
    """
    type, manifest = get_manifest(pkg_name)

    deps = {}
    if manifest and "dependencies" in manifest:
        deps = manifest["dependencies"]

    if not deps:
        return {}

    components = {}
    for pkg, _ in aip.get_resources("packages").items():
        type, manifest = get_manifest(pkg)
        if type:
            components.setdefault(type, []).append((pkg, manifest))

    result = {}
    for type, dep_list in deps.items():
        for dep in dep_list:
            if type in components:
                found = False
                for c in components[type]:
                    pkg, manifest = c
                    if manifest["name"] == dep:
                        result.setdefault(type, []).append(c)
                        found = True
                if not found:
                    print(f"Missing dependency of <{pkg_name}>: {type} <{dep}>")
            else:
                print(f"Missing dependency of <{pkg_name}>: {type} <{dep}>")

    return result


if __name__ == "__main__":
    dependencies = get_component_dependencies("carry")

    print("This component has the following dependencies:")
    for type, component in dependencies.items():
        pkg, manifest = component
        print(f"- {type} {manifest['name']}, implemented by package {pkg}")
