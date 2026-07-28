#!/usr/bin/env python3
"""Validate SET_POSITION_TARGET_LOCAL_NED frames in a proxy JSONL capture."""

import argparse
import json
import math
import struct
import sys


CRC_EXTRA = 143
EXPECTED_MASK = 0x09F8


def crc_accumulate(byte, crc):
    temporary = byte ^ (crc & 0xFF)
    temporary ^= (temporary << 4) & 0xFF
    return ((crc >> 8) ^ (temporary << 8) ^
            (temporary << 3) ^ (temporary >> 4)) & 0xFFFF


def valid_crc(frame, payload_length, version):
    header_length = 6 if version == 1 else 10
    checksum_offset = header_length + payload_length
    crc = 0xFFFF
    for byte in frame[1:checksum_offset]:
        crc = crc_accumulate(byte, crc)
    crc = crc_accumulate(CRC_EXTRA, crc)
    encoded = struct.unpack_from("<H", frame, checksum_offset)[0]
    return crc == encoded


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture")
    args = parser.parse_args()

    targets = []
    with open(args.capture, encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if (record.get("direction") != "BRIDGE_TO_FC" or
                    record.get("message_id") != 84):
                continue
            frame = bytes.fromhex(record["frame_hex"])
            version = record["mavlink_version"]
            payload_length = record["payload_length"]
            payload_offset = 6 if version == 1 else 10
            if payload_length != 53 or not valid_crc(
                    frame, payload_length, version):
                raise RuntimeError("invalid SET_POSITION_TARGET_LOCAL_NED frame")
            fields = struct.unpack_from(
                "<I11fHBBB", frame, payload_offset)
            target = {
                "time_boot_ms": fields[0],
                "x": fields[1], "y": fields[2], "z": fields[3],
                "vx": fields[4], "vy": fields[5], "vz": fields[6],
                "afx": fields[7], "afy": fields[8], "afz": fields[9],
                "yaw": fields[10], "yaw_rate": fields[11],
                "type_mask": fields[12],
                "target_system": fields[13],
                "target_component": fields[14],
                "coordinate_frame": fields[15],
            }
            if target["time_boot_ms"] == 0:
                raise RuntimeError("zero time_boot_ms")
            if (target["target_system"], target["target_component"]) != (1, 1):
                raise RuntimeError("unexpected target system/component")
            if target["coordinate_frame"] != 1:
                raise RuntimeError("coordinate_frame is not MAV_FRAME_LOCAL_NED")
            if target["type_mask"] != EXPECTED_MASK:
                raise RuntimeError("type_mask is not position+yaw only")
            numeric = [target[key] for key in (
                "x", "y", "z", "vx", "vy", "vz", "afx", "afy",
                "afz", "yaw", "yaw_rate")]
            if not all(math.isfinite(value) for value in numeric):
                raise RuntimeError("non-finite target field")
            targets.append(target)

    if not targets:
        print("POSITION_TARGET_WIRE: FAIL (no message 84)")
        return 1
    last = targets[-1]
    print(
        "POSITION_TARGET_WIRE: PASS "
        f"count={len(targets)} frame={last['coordinate_frame']} "
        f"mask=0x{last['type_mask']:04x} target=1/1 "
        f"xyz=({last['x']:.3f},{last['y']:.3f},{last['z']:.3f}) "
        f"yaw={last['yaw']:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
