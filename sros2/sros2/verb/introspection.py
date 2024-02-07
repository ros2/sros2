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


def _unique_dds_endpoints(packages):
    """
    Returns a dict with unique DDS endpoints.

    Processes list of packages and returns a dict
    of RTPS packages wherein each keys corresponds
    with a unique guidPrefix and the content with
    the package itself.
    """
    # filter RTPS packages
    rtps_packages = [p for p in packages if RTPS in p]

    # keep only new endpoints (unique guidPrefix)
    dict_dds_endpoints = {}
    for p in rtps_packages:
        hostId = p[RTPS].guidPrefix.hostId
        appId = p[RTPS].guidPrefix.appId
        instanceId = p[RTPS].guidPrefix.instanceId

        key = (hostId, appId, instanceId)
        if key in dict_dds_endpoints:
            continue
        dict_dds_endpoints[key] = p
    return dict_dds_endpoints


def _package_version_str(package):
    """
    Returns the RTPS version of a package as an string
    """
    return (
        str(package[RTPS].protocolVersion.major)
        + "."
        + str(package[RTPS].protocolVersion.minor)
    )


class IntrospectDDSEnpoints(VerbExtension):
    """Introspect DDS endpoints."""

    def add_arguments(self, parser, cli_name) -> None:
        parser.add_argument(
            "iface",
            type=str,
            nargs=1,
            help="Network interface whereto introspect (e.g. lo).",
        )
        parser.add_argument(
            "timeout", type=int, nargs="?", help="Seconds capturing traffic.", default=1
        )

    def main(self, *, args) -> int:
        print(
            "introspecting "
            + args.iface[0]
            + " for "
            + str(args.timeout)
            + " seconds ..."
        )
        packages = sniff(iface=args.iface[0], timeout=args.timeout)
        # get unique DDS endpoints
        dict_dds_endpoints = _unique_dds_endpoints(packages)

        for key in dict_dds_endpoints.keys():
            hostId, appId, instanceId = key
            p = dict_dds_endpoints[key]

            print(
                "DDS endpoint detected (hostId={}, appId={}, instanceId={})".format(
                    hostId, appId, instanceId
                )
            )
            print("\t- RTPS version: " + _package_version_str(p))
            # print(p[RTPS].vendorId.vendor_id)
            # print(_rtps_vendor_ids)

            if p[RTPS].vendorId.vendor_id in _rtps_vendor_ids:
                print(
                    "\t- vendorId: " + str(_rtps_vendor_ids[p[RTPS].vendorId.vendor_id])
                )
            else:
                print("\t- vendorId: Unknown")

            transport = "UDP"
            if UDP in p:
                transport = "UDP"
            elif TCP in p:
                transport = "UDP"
            else:
                transport = "unknown"
            print("\t- transport: " + transport)
        return 0
