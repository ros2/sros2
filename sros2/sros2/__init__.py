# Copyright 2019 Apex.AI, Inc.
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

import os
import urllib.parse
import urllib.request

import ament_index_python


_xml_cache_path = urllib.parse.urljoin(
    'file:',
    urllib.request.pathname2url(
        os.path.join(
            str(ament_index_python.get_package_share_directory('sros2')),
            'xml_cache',
            'xhtml-cache.xml'
        )
    )
)


if 'XML_CATALOG_FILES' not in os.environ:
    os.environ['XML_CATALOG_FILES'] = _xml_cache_path
elif _xml_cache_path not in os.environ['XML_CATALOG_FILES']:
    os.environ['XML_CATALOG_FILES'] += ' ' + _xml_cache_path
