#!/usr/bin/env python3
"""Minimal stdlib-only pcap parser to classify RTPS submessages by port over time.

No scapy/dpkt/tshark available on either board or the dev machine, and pulling
one in just for a single diagnostic pass wasn't worth it -- pcap's on-disk
format and RTPS's submessage framing are both simple enough to walk directly.

Usage: python3 analyze_rtps_pcap.py capture.pcap
"""

import struct
import sys
from collections import defaultdict

SUBMSG_NAMES = {
    0x01: "PAD",
    0x06: "ACKNACK",
    0x07: "HEARTBEAT",
    0x08: "GAP",
    0x09: "INFO_TS",
    0x0C: "INFO_REPLY",
    0x0D: "SEC_PREFIX",
    0x0E: "SEC_BODY",
    0x0F: "SEC_POSTFIX",
    0x12: "INFO_SRC",
    0x13: "INFO_REPLY_IP4",
    0x15: "DATA",
    0x16: "DATA_FRAG",
    0x18: "HEARTBEAT_FRAG",
    0x1A: "NACK_FRAG",
    0x1B: "INFO_DST",
    0x1C: "SEC_RTPS_PREFIX",
    0x1D: "SEC_RTPS_BODY",
    0x1E: "SEC_RTPS_POSTFIX",
}


def parse_pcap(path):
    with open(path, "rb") as f:
        data = f.read()

    magic = data[:4]
    if magic == b"\xa1\xb2\xc3\xd4":
        endian = ">"
    elif magic == b"\xd4\xc3\xb2\xa1":
        endian = "<"
    else:
        raise ValueError(f"unrecognized pcap magic: {magic!r}")

    off = 24  # global header
    t0 = None
    while off < len(data):
        ts_sec, ts_usec, caplen, _origlen = struct.unpack_from(endian + "IIII", data, off)
        off += 16
        frame = data[off : off + caplen]
        off += caplen
        ts = ts_sec + ts_usec / 1e6
        if t0 is None:
            t0 = ts
        yield ts - t0, frame


def parse_frame(frame):
    """Return (src_ip, sport, dst_ip, dport, udp_payload) or None."""
    if len(frame) < 14:
        return None
    eth_type = struct.unpack_from(">H", frame, 12)[0]
    ip_off = 14
    if eth_type == 0x8100:  # 802.1Q VLAN tag
        ip_off += 4
        eth_type = struct.unpack_from(">H", frame, 16)[0]
    if eth_type != 0x0800:  # not IPv4
        return None
    ver_ihl = frame[ip_off]
    ihl = (ver_ihl & 0x0F) * 4
    proto = frame[ip_off + 9]
    if proto != 17:  # not UDP
        return None
    src_ip = ".".join(str(b) for b in frame[ip_off + 12 : ip_off + 16])
    dst_ip = ".".join(str(b) for b in frame[ip_off + 16 : ip_off + 20])
    udp_off = ip_off + ihl
    if len(frame) < udp_off + 8:
        return None
    sport, dport, udp_len = struct.unpack_from(">HHH", frame, udp_off)
    payload = frame[udp_off + 8 : udp_off + udp_len]
    return src_ip, sport, dst_ip, dport, payload


def classify_rtps(payload):
    """Yield submessage type names found in one RTPS packet."""
    if payload[:4] != b"RTPS":
        return
    off = 20  # 4 magic + 2 version + 2 vendor + 12 guid prefix
    while off + 4 <= len(payload):
        submsg_id = payload[off]
        flags = payload[off + 1]
        endian_flag = flags & 0x01
        fmt = "<H" if endian_flag else ">H"
        octets_to_next = struct.unpack_from(fmt, payload, off + 2)[0]
        yield SUBMSG_NAMES.get(submsg_id, f"0x{submsg_id:02X}")
        off += 4 + octets_to_next
        if octets_to_next == 0:
            break  # last submessage extends to end of packet, per RTPS spec


def main():
    path = sys.argv[1]
    bucket_sec = 10
    # buckets[bucket][(port_pair)][submsg_name] = count
    buckets = defaultdict(lambda: defaultdict(lambda: defaultdict(int)))
    port_traffic = defaultdict(int)
    total = 0
    rtps_total = 0

    for ts, frame in parse_pcap(path):
        parsed = parse_frame(frame)
        if parsed is None:
            continue
        total += 1
        src_ip, sport, dst_ip, dport, payload = parsed
        port_key = f"{src_ip}:{sport}->{dst_ip}:{dport}"
        port_traffic[port_key] += 1
        if payload[:4] != b"RTPS":
            continue
        rtps_total += 1
        bucket = int(ts // bucket_sec) * bucket_sec
        for name in classify_rtps(payload):
            buckets[bucket][port_key][name] += 1

    print(f"total UDP packets: {total}, RTPS packets: {rtps_total}\n")
    print("Top port pairs by packet count:")
    for k, v in sorted(port_traffic.items(), key=lambda kv: -kv[1])[:15]:
        print(f"  {k}: {v}")
    print()

    print(f"{'t(s)':>6}  {'port pair':45}  submessage counts")
    for bucket in sorted(buckets):
        for port_key, counts in sorted(buckets[bucket].items(), key=lambda kv: -sum(kv[1].values())):
            if sum(counts.values()) < 2:
                continue
            counts_str = ", ".join(f"{n}={c}" for n, c in sorted(counts.items(), key=lambda kv: -kv[1]))
            print(f"{bucket:>6}  {port_key:45}  {counts_str}")


if __name__ == "__main__":
    main()
