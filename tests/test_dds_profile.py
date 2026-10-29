"""The Fast DDS profile generator (tools/dds_profile.py, host): valid XML, the right addresses, the right ports.

Pure string and XML work: no ROS, no DDS participant, no network.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import dds_profile      # noqa: E402 - after sys.path

NS = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}


def _parse(text: str):
    root = ET.fromstring(text)
    return root


def test_multicast_profile_is_valid_xml_and_udp_only():
    text = dds_profile.profile("192.168.9.1", [], domain=73, recv_buf=4 << 20, send_buf=1 << 20)
    root = _parse(text)
    td = root.find(".//d:transport_descriptor", NS)
    assert td.find("d:type", NS).text == "UDPv4"
    assert td.find("d:receiveBufferSize", NS).text == str(4 << 20)
    # the whitelist is the one address, and nothing else
    addrs = [a.text for a in td.findall(".//d:interfaceWhiteList/d:address", NS)]
    assert addrs == ["192.168.9.1"]
    p = root.find(".//d:participant", NS)
    assert p.get("is_default_profile") == "true"
    assert p.find(".//d:useBuiltinTransports", NS).text == "false"      # no shared memory: two hosts
    assert p.find(".//d:userTransports/d:transport_id", NS).text == "uw_udp"
    assert root.find(".//d:initialPeersList", NS) is None               # multicast discovery left in place


def test_unicast_peers_get_the_domain_s_metatraffic_ports():
    text = dds_profile.profile("192.168.9.1", ["192.168.9.2", "192.168.9.3"], domain=73,
                               recv_buf=1 << 20, send_buf=1 << 20, peer_participants=3)
    root = _parse(text)
    peers = [(loc.find("d:address", NS).text, int(loc.find("d:port", NS).text))
             for loc in root.findall(".//d:initialPeersList/d:locator/d:udpv4", NS)]
    # 7400 + 250*73 + 10 = 25660, then +2 per participant id, for each peer
    assert peers == [("192.168.9.2", 25660), ("192.168.9.2", 25662), ("192.168.9.2", 25664),
                     ("192.168.9.3", 25660), ("192.168.9.3", 25662), ("192.168.9.3", 25664)]
    mine = root.find(".//d:metatrafficUnicastLocatorList/d:locator/d:udpv4/d:address", NS).text
    assert mine == "192.168.9.1"
    assert dds_profile.PORT_BASE + dds_profile.DOMAIN_GAIN * 74 == 25900        # the ground-truth domain


def test_loopback_profile_cannot_reach_the_hil_interface():
    out = dds_profile.main(["--loopback", "--out", "-"])
    assert out == 0


def test_loopback_profile_addresses(capsys):
    dds_profile.main(["--loopback", "--out", "-"])
    root = _parse(capsys.readouterr().out)
    addrs = [a.text for a in root.findall(".//d:interfaceWhiteList/d:address", NS)]
    assert addrs == ["127.0.0.1"] and root.find(".//d:initialPeersList", NS) is None


def test_an_address_passes_through_and_a_bad_one_is_refused(tmp_path):
    assert dds_profile.iface_address("10.0.0.7") == "10.0.0.7"
    with pytest.raises(SystemExit):
        dds_profile.main(["--iface", "10.0.0.7", "--peers", "not-an-ip", "--out", str(tmp_path / "x.xml")])
    with pytest.raises(SystemExit):
        dds_profile.main(["--out", str(tmp_path / "x.xml")])             # no interface at all
    assert not (tmp_path / "x.xml").exists()


def test_written_file_round_trips(tmp_path):
    path = tmp_path / "dds" / "fastdds_hil.xml"
    dds_profile.main(["--iface", "10.1.2.3", "--peers", "10.1.2.4", "--domain", "74", "--out", str(path)])
    root = _parse(path.read_text())
    assert [a.text for a in root.findall(".//d:interfaceWhiteList/d:address", NS)] == ["10.1.2.3"]
    ports = {int(p.text) for p in root.findall(".//d:initialPeersList//d:port", NS)}
    assert min(ports) == 7400 + 250 * 74 + 10                            # the domain reached the peer ports
