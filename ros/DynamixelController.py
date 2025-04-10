#!/usr/bin/env python3
from dynamixel_sdk import *

class DynamixelController:

    def __init__(self):
        self.port_handler = None
        self.packet_handler = None

    def open(self, port, baudrate=1000000, protocol_version=2.0):
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(protocol_version)

        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            return

        if self.port_handler.setBaudRate(baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            return

    def close(self):
        self.port_handler.closePort()

    def write_1_byte(self, dxl_id, address, data):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            address, data
        )
        return dxl_comm_result, dxl_error

    def write_4_byte(self, dxl_id, address, data):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            dxl_id,
            address, data
        )
        return dxl_comm_result, dxl_error

    def read_4_byte(self, dxl_id, address):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id,
            address
        )
        return dxl_present_position, dxl_comm_result, dxl_error

    def get_dxl_result(self, res, err):
        if res != COMM_SUCCESS:
            return self.packet_handler.getTxRxResult(res)
        elif err != 0:
            return self.packet_handler.getRxPacketError(err)
        else:
            return "success"

    # https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#torque-enable64
    def torque_enable(self, dxl_id):
        res, err = self.write_1_byte(dxl_id, 64, 1)
        print("DXL_%d torque-enable(64) enable: " % dxl_id, end="")
        print(self.get_dxl_result(res, err))

    def torque_disable(self, dxl_id):
        res, err = self.write_1_byte(dxl_id, 64, 0)
        print("DXL_%d torque-enable(64) disable: " % dxl_id, end="")
        print(self.get_dxl_result(res, err))

    # https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#goal-position116
    def goal_position(self, dxl_id, position):
        res, err = self.write_4_byte(dxl_id, 116, position)
        print("DXL_%d goal-position(116) %d: " % (dxl_id, position), end="")
        print(self.get_dxl_result(res, err))

    # https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#present-position132
    def present_position(self, dxl_id):
        val, res, err = self.read_4_byte(dxl_id, 132)
        return val

if __name__ == "__main__":
    c = DynamixelController()
    c.open("COM4")
    c.torque_enable(1)
    c.goal_position(1, 4000)
    c.torque_disable(1)
    c.close()
