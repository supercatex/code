import serial


class DIHOOL_IPS_HS1(object):
    #                       CTRL, FUNC, MOTOR_ID  , DATA      , CRCL, CRCH
    _CMD_STOP  = bytearray([0x01, 0x06, 0x00, 0x01, 0x00, 0x01, 0x19, 0xCA])
    _CMD_UP    = bytearray([0x01, 0x06, 0x00, 0x01, 0x00, 0x02, 0x59, 0xCB])
    _CMD_DOWN  = bytearray([0x01, 0x06, 0x00, 0x01, 0x00, 0x04, 0xD9, 0xC9])
    _CMD_RESET = bytearray([0x01, 0x06, 0x00, 0x01, 0x00, 0x08, 0xD9, 0xCC])
    _CMD_READ  = bytearray([0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xCA])
    _CMD_MOVE  = bytearray([0x01, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00])

    def __init__(self, port="/dev/ttyCH341USB0", baudrate=9600, timeout=0.2):
        self.serial = serial.Serial(
            port = port,
            baudrate=baudrate,
            timeout=timeout
        )

    def send_cmd(self, data: bytearray):
        data[6], data[7] = self.calculate_modbus_crc(bytes(data[:6]))
        self.serial.write(bytes(data))
        response = self.serial.read(1024)
        return response

    def calculate_modbus_crc(cls, data: bytes):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        crcl = (crc >> 0) & 0xFF # Lower 8 bits
        crch = (crc >> 8) & 0xFF # Upper 8 bits
        return crcl, crch

    # ID (2, 3, 4, 5)
    def get_data(self, ID=2):
        data = self._CMD_READ
        data[3] = ID
        res = self.send_cmd(data)
        return res[3] * 256 + res[4]

    # value [0, 390] mm
    def set_data(self, value, ID=2, wait=False):
        value = min(max(0, value), 390)
        data = self._CMD_MOVE
        data[3] = ID
        data[4] = value // 256
        data[5] = value % 256
        res = self.send_cmd(data)
        while wait and value != self.get_data(ID): pass
        return res

    def up(self):
        self.send_cmd(self._CMD_UP)

    def down(self):
        self.send_cmd(self._CMD_DOWN)

    def stop(self):
        self.send_cmd(self._CMD_STOP)
    

if __name__ == "__main__":
    obj = DIHOOL_IPS_HS1()
    s = ""
    while s != "q":
        s = input("INPUT:")
        if s == "stop":
            obj.send_cmd(obj._CMD_STOP)
        if s == "up":
            obj.send_cmd(obj._CMD_UP)
        if s == "down":
            obj.send_cmd(obj._CMD_DOWN)
        if s == "read":
            print(obj.get_data())
        if s.isnumeric():
            obj.set_data(int(s), wait=True)
        
