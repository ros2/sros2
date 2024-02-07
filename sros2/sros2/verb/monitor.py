# Copyright 2022 Alias Robotics
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

import sys, signal
from sros2.verb import VerbExtension

# Avoid 'GDK_IS_DISPLAY (display)' failed
# see https://github.com/secdev/scapy/issues/2666
import matplotlib

matplotlib.use("Agg")

from scapy.all import *
from scapy.layers.inet import UDP, TCP, IP
from scapy.contrib.rtps import RTPS
from scapy.contrib.rtps.common_types import _rtps_vendor_ids

bind_layers(UDP, RTPS)
conf.verb = 0

from sros2.verb.introspection import _unique_dds_endpoints, _package_version_str

# A dict mapping for each DDS vendor, RTPS version -> ROS 2 distros
#  as shipped in .deb files
_rtpsversion_ros2distro_map = {
    b"\x01\x0F": {  # "eProsima - Fast-RTPS"
        "2.1": ["Bouncy Bolson", "Ardent Apalone"],
        "2.2": [
            "Crystal Clemmys",
            "Dashing Diademata",
            "Eloquent Elusor",
            "Foxy Fitzroy",
            "Galactic Geochelone",
        ],
        "2.3": [  # with HAVE_SECURITY
            "Crystal Clemmys",
            "Dashing Diademata",
            "Eloquent Elusor",
            "Foxy Fitzroy",
            "Galactic Geochelone",
        ],
    },
    b"\x01\x10": {  # "ADLINK - Cyclone DDS",
        "2.1": [
            "Dashing Diademata",
            "Eloquent Elusor",
            "Foxy Fitzroy",
            "Galactic Geochelone",
        ],
    },
}

# A dict mapping for each DDS vendor, ROS 2 distros -> DDS release version
#  as shipped in .deb files
_ros2distro_ddsversion_map = {
    b"\x01\x0F": {  # "eProsima - Fast-RTPS"
        "Bouncy Bolson": ["1.6.0"],
        "Crystal Clemmys": ["1.6.0", "1.7.0", "1.7.2"],
        "Dashing Diademata": [
            "1.7.2",
            "1.8.0",
            "1.8.1",
            "1.8.2",
            "1.8.4",
        ],
        "Eloquent Elusor": ["1.9.0", "1.9.2", "1.9.3"],
        "Foxy Fitzroy": ["2.0.0", "2.0.1", "2.0.2", "2.1.1"],
        "Galactic Geochelone": ["2.3.1", "2.3.4"],
    },
    b"\x01\x10": {  # "ADLINK - Cyclone DDS",
        "Dashing Diademata": ["0.1.0", "0.5.1", "0.7.0"],
        "Eloquent Elusor": ["0.1.0", "0.5.1", "0.7.0"],
        "Foxy Fitzroy": ["0.6.0", "0.7.0"],
        "Galactic Geochelone": ["0.8.0"],
    },
}

# A dict mapping for each DDS vendor publicly disclosed DDS vulnerabilities -> DDS release version
#  as shipped in .deb files
_ddsvulns_ddsversion_map = {
    b"\x01\x0F": {  # "eProsima - Fast-RTPS"
        "CVE-2021-38425": [
            "1.6.0",
            "1.7.0",
            "1.7.2",
            "1.8.0",
            "1.8.1",
            "1.8.2",
            "1.8.4",
            "1.9.0",
            "1.9.2",
            "1.9.3",
            "2.0.0",
            "2.0.1",
            "2.0.2",
            "2.1.1",
            "2.3.1",
            "2.3.4",
        ],
    },
    b"\x01\x10": {  # "ADLINK - Cyclone DDS",
        "CVE-2021-38441": ["0.1.0", "0.5.1", "0.6.0", "0.7.0"],
        "CVE-2021-38443": ["0.1.0", "0.5.1", "0.6.0", "0.7.0"],
    },
}


class SecurityMonitorDDS(VerbExtension):
    """Monitor DDS endpoints in search for known-to-be vulnerable versions."""

    def add_arguments(self, parser, cli_name) -> None:
        parser.add_argument(
            "iface",
            type=str,
            nargs="?",
            help="Network interface whereto introspect (e.g. lo).",
        )

    def signal_handler(self, signal, frame):
        print("\nexiting")
        sys.exit(0)

    def main(self, *, args) -> int:
        print("sniffing the DDS network...")

        signal.signal(signal.SIGINT, self.signal_handler)
        unique_endpoints = []  # keys of unique endpoints

        while True:
            if args.iface:
                packages = sniff(iface=args.iface[0], timeout=0.5)
            else:
                packages = sniff(timeout=0.5)
            # get unique DDS endpoints
            dict_dds_endpoints = _unique_dds_endpoints(packages)

            for key in dict_dds_endpoints.keys():
                hostId, appId, instanceId = key
                p = dict_dds_endpoints[key]

                if key in unique_endpoints:
                    continue

                # add to unique_endpoints
                unique_endpoints.append(key)

                # figure out if vulnerable and if so, report it
                vulnerabilities = []
                version_candidates = []
                rtps_version = _package_version_str(p)
                vendor_id = p[RTPS].vendorId.vendor_id

                if vendor_id in _rtpsversion_ros2distro_map:
                    if rtps_version in _rtpsversion_ros2distro_map[vendor_id]:
                        # print(_rtpsversion_ros2distro_map[vendor_id][rtps_version])  # debug

                        # find out DDS release version candidates
                        for distro in _rtpsversion_ros2distro_map[vendor_id][
                            rtps_version
                        ]:
                            version_candidates += _ros2distro_ddsversion_map[vendor_id][
                                distro
                            ]
                        # figure out vulnerabilities affecting these candidates
                        for vuln in _ddsvulns_ddsversion_map[vendor_id].keys():
                            for version in version_candidates:
                                if version in _ddsvulns_ddsversion_map[vendor_id][vuln]:
                                    cve_str = vuln + " ({} version {})".format(
                                        _rtps_vendor_ids[vendor_id], version
                                    )
                                    if not cve_str in vulnerabilities:
                                        vulnerabilities.append(cve_str)

                if len(vulnerabilities) > 0:
                    print(
                        "Vulnerable DDS endpoint found (hostId={}, appId={}, instanceId={})".format(
                            hostId, appId, instanceId
                        )
                    )
                    if vendor_id in _rtps_vendor_ids:
                        print("\t- vendorId: " + str(_rtps_vendor_ids[vendor_id]))
                    else:
                        print("\t- vendorId: Unknown")
                    print(
                        "\t- DDS implementation version candidates: {}".format(
                            version_candidates
                        )
                    )
                    print("\t- CVE IDs:")
                    for vuln in vulnerabilities:
                        print("\t\t* {}".format(vuln))

        return 0
