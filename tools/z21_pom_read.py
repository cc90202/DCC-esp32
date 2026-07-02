#!/usr/bin/env python3
"""Minimal Z21 POM/RailCom diagnostic client."""

from __future__ import annotations

import argparse
import socket
import sys


HEADER_XBUS = 0x0040
HEADER_RAILCOM_GETDATA = 0x0089
RESP_RAILCOM_DATACHANGED = 0x0088
XHEADER_CV_POM = 0xE6
DB0_POM_LOCO = 0x30
POM_OP_READ = 0xE4
POM_OP_WRITE = 0xEC
RESP_XH_CV_RESULT = 0x64
RESP_DB0_CV_RESULT = 0x14
RESP_XH_TRACK_POWER = 0x61
RESP_DB0_CV_NACK = 0x13


def checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def z21_address(address: int) -> tuple[int, int]:
    if 1 <= address <= 127:
        return 0x00, address
    if 128 <= address <= 10239:
        return 0xC0 | ((address >> 8) & 0x3F), address & 0xFF
    raise ValueError("DCC address must be 1..10239")


def make_pom_read_frame(address: int, cv: int) -> bytes:
    if not 1 <= cv <= 1024:
        raise ValueError("CV must be 1..1024")
    addr_h, addr_l = z21_address(address)
    wire_cv = cv - 1
    cv_h = POM_OP_READ | ((wire_cv >> 8) & 0x03)
    cv_l = wire_cv & 0xFF
    payload = bytes([XHEADER_CV_POM, DB0_POM_LOCO, addr_h, addr_l, cv_h, cv_l, 0x00])
    payload += bytes([checksum(payload)])
    data_len = 4 + len(payload)
    return data_len.to_bytes(2, "little") + HEADER_XBUS.to_bytes(2, "little") + payload


def make_pom_write_frame(address: int, cv: int, value: int) -> bytes:
    if not 1 <= cv <= 1024:
        raise ValueError("CV must be 1..1024")
    if not 0 <= value <= 255:
        raise ValueError("CV value must be 0..255")
    addr_h, addr_l = z21_address(address)
    wire_cv = cv - 1
    cv_h = POM_OP_WRITE | ((wire_cv >> 8) & 0x03)
    cv_l = wire_cv & 0xFF
    payload = bytes([XHEADER_CV_POM, DB0_POM_LOCO, addr_h, addr_l, cv_h, cv_l, value])
    payload += bytes([checksum(payload)])
    data_len = 4 + len(payload)
    return data_len.to_bytes(2, "little") + HEADER_XBUS.to_bytes(2, "little") + payload


def make_railcom_getdata_frame(address: int) -> bytes:
    if not 1 <= address <= 10239:
        raise ValueError("DCC address must be 1..10239")
    payload = bytes([0x01]) + address.to_bytes(2, "little")
    data_len = 4 + len(payload)
    return (
        data_len.to_bytes(2, "little")
        + HEADER_RAILCOM_GETDATA.to_bytes(2, "little")
        + payload
    )


def parse_response(data: bytes) -> str:
    if len(data) < 4:
        return f"short response: {data.hex(' ')}"
    length = int.from_bytes(data[0:2], "little")
    header = int.from_bytes(data[2:4], "little")
    payload = data[4:length]
    if header != HEADER_XBUS:
        return f"non-XBus header=0x{header:04x} raw={data.hex(' ')}"
    if len(payload) >= 6 and payload[0] == RESP_XH_CV_RESULT and payload[1] == RESP_DB0_CV_RESULT:
        wire_cv = ((payload[2] & 0x03) << 8) | payload[3]
        value = payload[4]
        return f"CV{wire_cv + 1} = {value} (0x{value:02X}, 0b{value:08b})"
    if len(payload) >= 3 and payload[0] == RESP_XH_TRACK_POWER and payload[1] == RESP_DB0_CV_NACK:
        return f"CV_NACK raw={data.hex(' ')}"
    return f"unknown response raw={data.hex(' ')}"


def parse_railcom_response(data: bytes) -> str:
    if len(data) < 4:
        return f"short response: {data.hex(' ')}"
    length = int.from_bytes(data[0:2], "little")
    header = int.from_bytes(data[2:4], "little")
    payload = data[4:length]
    if header != RESP_RAILCOM_DATACHANGED:
        return f"non-RailCom header=0x{header:04x} raw={data.hex(' ')}"
    if len(payload) < 12:
        return f"short RailCom response raw={data.hex(' ')}"

    address = int.from_bytes(payload[0:2], "little")
    receive_counter = int.from_bytes(payload[2:6], "little")
    error_counter = int.from_bytes(payload[6:8], "little")
    return (
        f"addr={address} receive_counter={receive_counter} "
        f"error_counter={error_counter} raw={data.hex(' ')}"
    )


def poll_railcom_address(
    sock: socket.socket,
    host: str,
    port: int,
    address: int,
    timeout: float,
    label: str,
) -> None:
    frame = make_railcom_getdata_frame(address)
    print(f"> RailCom CH1 address {label}: {frame.hex(' ')}")
    sock.sendto(frame, (host, port))
    previous_timeout = sock.gettimeout()
    sock.settimeout(timeout)
    try:
        data, remote = sock.recvfrom(1024)
    except TimeoutError:
        print(f"< RailCom CH1 address {label}: timeout/no cached address")
    else:
        print(
            f"< RailCom CH1 address {label} from {remote[0]}:{remote[1]}: "
            f"{parse_railcom_response(data)}"
        )
    finally:
        sock.settimeout(previous_timeout)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="ESP32/Z21 IP address, e.g. 192.168.1.58")
    parser.add_argument("--port", type=int, default=21105)
    parser.add_argument("--addr", type=int, default=3, help="DCC loco address")
    parser.add_argument("--cv", type=int, nargs="+", default=[29, 28], help="CV numbers to read")
    parser.add_argument(
        "--write-cv",
        type=int,
        nargs=2,
        action="append",
        metavar=("CV", "VALUE"),
        default=[],
        help="Write a CV via POM before reads. Requires --allow-write.",
    )
    parser.add_argument(
        "--allow-write",
        action="store_true",
        help="Actually send --write-cv POM writes. Writes are persistent decoder changes.",
    )
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument(
        "--railcom-timeout",
        type=float,
        default=0.35,
        help="Timeout for LAN_RAILCOM_GETDATA address polls",
    )
    parser.add_argument(
        "--no-railcom-address",
        action="store_true",
        help="Do not poll the cached RailCom address around POM reads",
    )
    args = parser.parse_args()

    if args.write_cv and not args.allow_write:
        parser.error("--write-cv changes decoder CVs permanently; add --allow-write to send it")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(args.timeout)
        if not args.no_railcom_address:
            poll_railcom_address(
                sock, args.host, args.port, args.addr, args.railcom_timeout, "before"
            )
        for cv, value in args.write_cv:
            frame = make_pom_write_frame(args.addr, cv, value)
            print(f"> WRITE CV{cv}={value} (0x{value:02X}): {frame.hex(' ')}")
            sock.sendto(frame, (args.host, args.port))
            try:
                data, remote = sock.recvfrom(1024)
            except TimeoutError:
                print(f"< WRITE CV{cv}: timeout")
                continue
            print(
                f"< WRITE CV{cv} from {remote[0]}:{remote[1]}: "
                f"{parse_response(data)}"
            )
            if not args.no_railcom_address:
                poll_railcom_address(
                    sock,
                    args.host,
                    args.port,
                    args.addr,
                    args.railcom_timeout,
                    f"after WRITE CV{cv}",
                )
        for cv in args.cv:
            frame = make_pom_read_frame(args.addr, cv)
            print(f"> CV{cv}: {frame.hex(' ')}")
            sock.sendto(frame, (args.host, args.port))
            try:
                data, remote = sock.recvfrom(1024)
            except TimeoutError:
                print(f"< CV{cv}: timeout")
                continue
            print(f"< CV{cv} from {remote[0]}:{remote[1]}: {parse_response(data)}")
            if not args.no_railcom_address:
                poll_railcom_address(
                    sock,
                    args.host,
                    args.port,
                    args.addr,
                    args.railcom_timeout,
                    f"after CV{cv}",
                )
    return 0


if __name__ == "__main__":
    sys.exit(main())
