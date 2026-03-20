#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import serial


class RelaySerialDriver:
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 0.02,
        relay_address: int = 0xFE,
        coil_map=None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.relay_address = relay_address
        self.coil_map = coil_map or {
            1: 0x0000,
            2: 0x0001,
        }
        self.lock = threading.Lock()
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def build_cmd(self, output_id: int, state: bool):
        if output_id not in self.coil_map:
            raise ValueError(f"Unsupported relay output: {output_id}")

        coil_addr = self.coil_map[output_id]
        frame = bytearray([
            self.relay_address,
            0x05,
            (coil_addr >> 8) & 0xFF,
            coil_addr & 0xFF,
            0xFF if state else 0x00,
            0x00,
        ])

        crc = crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        return frame

    def set_output(self, output_id: int, state: bool):
        cmd = self.build_cmd(output_id, state)
        with self.lock:
            self.ser.write(cmd)
            self.ser.flush()
        return cmd


def crc16(data: bytes):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc
