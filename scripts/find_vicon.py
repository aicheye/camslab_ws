#!/usr/bin/env python3
"""Looks for the motion capture's VRPN server (Vicon Tracker, port 3883) on every network
this computer is connected to. Needs python3 and nothing else; no ROS.

    python3 scripts/find_vicon.py
    python3 scripts/find_vicon.py --subnet 129.97.71.0/24

For every IPv4 network of this computer (the /24 around this computer's address when the
network is larger) it sends each address a UDP datagram to the port that names a TCP port
here, which is how a VRPN client asks the server to call back; a server then connects to
that port and greets with "vrpn: ver. ...". It does not open a TCP connection to the
port itself: Tracker does not answer one, and the next client then gets no object list.

For a server that answers it prints the address to put in `mocap_server` of
camslab/config/survey.yaml and the objects it tracks, each with its pose rate and
position. One of them must be the car's `name` (or `tracker`).
"""

import argparse
import ipaddress
import json
import socket
import struct
import subprocess
import time

COOKIE = b"vrpn: ver."


def networks() -> list:
    """(interface, this computer's address, network) for every IPv4 network but loopback
    and Docker's bridges. Without the `ip` command: the network of the default route,
    taken as a /24."""
    found = []
    try:
        out = subprocess.run(["ip", "-j", "-4", "addr"], capture_output=True, text=True, timeout=5)
        for interface in json.loads(out.stdout):
            name = interface.get("ifname", "")
            if name == "lo" or name.startswith(("docker", "br-", "veth")):
                continue
            for address in interface.get("addr_info", []):
                if address.get("family") == "inet":
                    net = ipaddress.ip_network(f"{address['local']}/{address['prefixlen']}",
                                               strict=False)
                    found.append((name, address["local"], net))
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    if not found:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("192.0.2.1", 9))         # no packet is sent
            here = probe.getsockname()[0]
            found.append(("default route", here, ipaddress.ip_network(f"{here}/24", strict=False)))
        except OSError:
            pass
        finally:
            probe.close()
    return found


def hosts_of(here: str, net) -> list:
    """The addresses to try in `net`: all of it up to a /22, else the /24 around `here`."""
    if net.prefixlen < 22:
        net = ipaddress.ip_network(f"{here}/24", strict=False)
    hosts = [str(h) for h in net.hosts()] or [str(net.network_address)]
    return [h for h in hosts if h != here] or hosts


def names_from(stream: socket.socket, wait: float = 5.0) -> list:
    """What a VRPN server sends after the greeting, as a list of lines, one per tracked
    object: its name, how many poses per second it streams and the last position.
    Messages of type -1 name the senders (the objects) and those of type -2 the message
    types; "vrpn_Tracker Pos_Quat" carries a sensor number, the position and the
    quaternion. The server starts a second or two after the greeting, so this reads
    until poses have arrived for a second, or for `wait` seconds."""
    data, started, first_pose = b"", time.time(), None
    senders, types, poses = {}, {}, {}
    stream.settimeout(0.3)
    position = 0
    while time.time() - started < wait and (first_pose is None or time.time() - first_pose < 1.0):
        try:
            chunk = stream.recv(65536)
        except socket.timeout:
            continue
        except OSError:
            break
        if not chunk:
            break
        data += chunk
        while position + 24 <= len(data):
            length, _, _, sender, kind = struct.unpack("!iiiii", data[position:position + 20])
            if length < 20 or length > 65536:
                position = len(data)
                break
            padded = length + (-length) % 8
            if position + padded > len(data):
                break
            body = data[position + 24:position + length]    # the header is padded to 24 bytes
            if kind in (-1, -2) and len(body) > 4:
                name = body[4:].split(b"\0")[0].decode(errors="replace")
                (senders if kind == -1 else types)[sender] = name
            elif types.get(kind) == "vrpn_Tracker Pos_Quat" and len(body) >= 32:
                first_pose = first_pose or time.time()
                count = poses.get(sender, (0, None))[0] + 1
                poses[sender] = (count, struct.unpack("!ddd", body[8:32]))
            position += padded
    seconds = max(time.time() - (first_pose or started), 1e-3)
    lines = []
    for sender, name in senders.items():
        if name == "VRPN Control":
            continue
        if sender in poses:
            count, (x, y, z) = poses[sender]
            lines.append(f"{name}: {count / seconds:.0f} poses/s, at x {x:.3f}  y {y:.3f}  z {z:.3f} m")
        else:
            lines.append(f"{name}: named, but no pose arrived (not seen by the cameras?)")
    return lines


def ask_by_udp(hosts: list, here: str, port: int, wait: float) -> dict:
    """{host: (greeting, names)} of the servers that call back after a UDP request, the
    VRPN client's way to open a connection."""
    answers = {}
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        listener.bind((here, 0))
        listener.listen(16)
        request = f"{here} {listener.getsockname()[1]}".encode() + b"\0"
        for host in hosts:
            try:
                sender.sendto(request, (host, port))
            except OSError:
                pass
        listener.settimeout(0.3)
        end = time.time() + wait
        while time.time() < end:
            try:
                stream, peer = listener.accept()
            except socket.timeout:
                continue
            with stream:
                stream.settimeout(1.0)
                try:
                    greeting = stream.recv(24)
                    if greeting.startswith(COOKIE):
                        stream.sendall(greeting)
                        answers[peer[0]] = (greeting.split(b"\0")[0].decode(errors="replace").strip(),
                                            names_from(stream))
                except OSError:
                    pass
    except OSError as error:
        print(f"  the UDP request from {here} failed: {error}")
    finally:
        listener.close()
        sender.close()
    return answers


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--subnet", action="append", default=[],
                        help="network to scan instead of this computer's, e.g. 129.97.71.0/24")
    parser.add_argument("--port", type=int, default=3883, help="the VRPN port")
    args = parser.parse_args()

    mine = networks()
    if args.subnet:
        targets = []
        for subnet in args.subnet:
            net = ipaddress.ip_network(subnet, strict=False)
            here = next((address for _, address, own in mine if net.overlaps(own)),
                        mine[0][1] if mine else "0.0.0.0")
            targets.append((subnet, here, net))
    else:
        targets = mine
    if not targets:
        print("this computer has no IPv4 network; connect to the lab network first")
        return 1

    found = {}
    for name, here, net in targets:
        hosts = hosts_of(here, net) if not args.subnet else ([str(h) for h in net.hosts()] or [str(net.network_address)])
        print(f"{name}: this computer is {here}, trying {len(hosts)} addresses of "
              f"{ipaddress.ip_network(f'{hosts[0]}/24', strict=False) if net.prefixlen < 22 and not args.subnet else net} "
              f"on port {args.port}", flush=True)
        for host, (greeting, names) in ask_by_udp(hosts, here, args.port, 2.0).items():
            found[host] = (greeting, names)

    if not found:
        print("no VRPN server answered. Check that this computer is on the motion capture's "
              "network, that Tracker is running with its VRPN stream on, and try "
              "--subnet with the network of the motion capture computer.")
        return 1
    for host, (greeting, names) in found.items():
        print(f"\nVRPN server at {host} ({greeting})")
        print(f"  mocap_server: {host}")
        for line in names:
            print(f"  {line}")
        if not names:
            print("  it names no object: none is enabled in Tracker's object list")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
