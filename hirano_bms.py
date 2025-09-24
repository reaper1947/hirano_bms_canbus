#!/usr/bin/env python3
from hiranoprotocol import HIRANO_BATTERY as HB
import syspy.battery_Can.canpass_base as cb
import syspy.lib.char_utility as cu
import syspy.lib.udp_debug as ud
import syspy.lib.misc_utility as mu
import sys
import os
import time
import can

sys.path.append('/usr/local/etc/.SeerRobotics/rbk/resources/scripts/site-packages')
sys.path.append(os.path.join(os.path.dirname(__file__), "syspy/protobuf"))


class testCanBattery(cb.canPassBase):
    def __init__(self):
        super(testCanBattery, self).__init__()
        self.__debug_out = ud.udpDebug()
        sys.stdout = self.__debug_out
        self.battery_info = self.createBatteryMessage()
        self.connect_timeout_t = mu.Timer(2000)
        self.msg_ok = False
        self.port = "can0"
        self.id1, self.id2, self.id3, self.id4 = False, False, False, False
        self.previous_temperature = None
        self.temperature_buffer = []
        self.clear = False
        self.error = False

        # self.srcname = self.getSrcName()
        # if self.srcname == 'SRC880':
        #     self.port1 = 'can0'
        #     self.port2 = 'can1'
        # else:
        #     self.port1 = 'can1'
        #     self.port2 = 'can0'
        # self.port3 = 'can2'
        # self.srcname = self.getSrcName()
        # if self.srcname == 'SRC880':
        #     self.port1 = 'can1'
        #     self.port2 = 'can0'
        # else:
        #     self.port1 = 'can0'
        #     self.port2 = 'can1'
        # self.port3 = 'can2'

        self.bus = can.interface.Bus(channel=self.port, bitrate=250000, interface="socketcan")

        print(f"[INIT] CAN bus connected on {self.port}")
    # def __init__(self):
    #     super(testCanBattery, self).__init__()
    #     self.connect_timeout_t = mu.Timer(2000)
    #     self.msg_ok = False
    #     self.port = "vcan0"
    #     self.bus = can.interface.Bus(channel=self.port, bustype="socketcan")
    #     self.battery_info = self.createBatteryMessage()
    #     print(f"[INIT] CAN bus connected on {self.port}")

    def getSrcName(self):
        with open('/etc/srcname', 'r') as file:
            srcname = file.readline().strip()
        return srcname

    def recvCanframe(self, timeout=1.0):
        msg = self.bus.recv(timeout)
        if msg:
            return msg.arbitration_id, msg.data
        return None

    def handleData(self, msg):
        try:
            self.judgeCanframe(msg)
            self.judgePublish()
        except ValueError as e:
            print(f"ValueError occurred in handleData: {e}")
        except TypeError as e:
            print(f"TypeError occurred in handleData: {e}")
        except Exception as e:
            print(f"Unexpected exception in handleData: {e}")

        self.judgeMsgok()

    def judgeCanframe(self, msg):
        can_id, data = msg
        # print(f"[RX] ID={hex(can_id)} Data={[hex(x) for x in data]}")

        if can_id == HB.FRAME_VOLT_CUR_SOC:
            battery_voltage = HB.parse_voltage(data)
            battery_current = HB.parse_current(data)
            battery_soc = HB.parse_soc(data)
            display_soc = HB.parse_display_soc(data)
            if battery_voltage is not None:
                print(f"[VOLTAGE] {battery_voltage:.1f} V")
            if battery_current is not None:
                print(f"[CURRENT] {battery_current:.1f} A")
            if battery_soc is not None and display_soc is not None:
                print(f"[SOC] {battery_soc:.2f} %")
            self.battery_info.percetage = battery_soc * 0.1
            self.battery_info.charge_current = battery_current
            self.battery_info.charge_voltage = battery_voltage
            self.id1 = True
            self.msg_ok = True

        elif can_id == HB.FRAME_TEMP:
            battery_temp = HB.parse_temp(data)
            if battery_temp is not None:
                print(f"[TEMP] {battery_temp} C")

            if self.previous_temperature is not None and abs(
                    battery_temp - self.previous_temperature) > 10:
                self.temperature_buffer.append(battery_temp)
                if len(self.temperature_buffer) >= 3:
                    self.previous_temperature = battery_temp
                    self.temperature_buffer = []
            else:
                self.previous_temperature = battery_temp
                self.temperature_buffer = []
                if battery_temp <= -19:
                    self.setError(53140, "The current temperature has reached " + str(
                        battery_temp) + " degrees , low temperature error!")
                elif -19 < battery_temp <= -15:
                    self.setWarning(54400, "The current temperature has reached " + str(
                        battery_temp) + " degrees , low temperature warning.")
                elif 55 <= battery_temp < 59:
                    self.setWarning(54400, "The current temperature has reached " + str(
                        battery_temp) + " degrees , high temperature warning.")
                elif battery_temp >= 59:
                    self.setError(53140, "The current temperature has reached " + str(
                        battery_temp) + " degrees , high temperature error!")
            self.battery_info.temperature = battery_temp
            self.msg_ok = True
            self.id2 = True

        elif can_id == HB.FRAME_ALARM:
            alarm_info = HB.parse_alarm(data)
            if alarm_info and alarm_info["AlarmCode"] != 0xff:
                error_msg = f"Battery pack number: {alarm_info['AlarmCode']} warning msg: {alarm_info['AlarmDesc']}"
                self.setWarning(53140, error_msg)
                self.error = True
            else:
                self.error = False
                print(f"[ALARM] {alarm_info}")

        elif can_id == HB.FRAME_CHARGE_INFO:
            charge_info = HB.parse_charge(data)
            print(f"[CHARGE] {charge_info}")

            if charge_info:
                if charge_info["BMS_ChargeStatus"] == 1:
                    self.battery_info.is_charging = True
                else:
                    self.battery_info.is_charging = False

            self.msg_ok = True
            self.id3 = True

        elif can_id == HB.FRAME_POWER_WATT:
            battery_power = HB.parse_power(data)
            print(f"[POWER] {battery_power} kw")

        elif can_id == HB.FRAME_DATE:
            date = HB.parse_date_time(data)
            print(f"[DATE] {date} ")

        self.msg_ok = True

    def judgePublish(self):
        if self.id1 and self.id2:
            self.publish(self.battery_info)
            # self.a = 1
        else:
            print(f"wait 3 ids all recv: id1{self.id1} id2{self.id2} id3{self.id3}")

    def judgeMsgok(self):
        if self.msg_ok:
            self.msg_ok = False
            self.connect_timeout_t.reset()
            if not self.clear:
                if self.warningExists(54001):
                    print('clear')
                    self.clearTimeout()
                else:
                    self.clear = True
        else:
            if self.connect_timeout_t.isTimeUp():
                print("[WARN] Timeout: no valid frame received")

    def loop(self):
        print("[INIT] Start listening on CAN bus...")
        try:
            while True:
                msg = self.recvCanframe(timeout=1.0)
                if msg:
                    self.handleData(msg)
                else:
                    print("[DEBUG] No CAN frame in last 1s")
                self.judgeMsgok()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("KeyboardInterrupt: shutting down CAN bus")
        finally:
            self.bus.shutdown()


if __name__ == "__main__":
    bt = testCanBattery()
    bt.loop()
