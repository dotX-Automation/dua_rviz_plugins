# dua_rviz_plugins

Collection of C++ plugins for RViz visualization integrated with the DUA framework.

## Usage

### `TextSubPanel`

This plugin displays a text message in a panel, subscribing to a given topic of type `std_msgs/msg/String`.

The text can be colored depending on the string being displayed. The colors can be specified in a YAML file whose absolute path can be set in the `DUA_RVIZ_PLUGINS_TEXTSUB_COLORS_YAML_PATH` environment variable. A default one is provided in [`config/colors.yaml`](config/colors.yaml).

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
