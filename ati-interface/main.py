import select
import time

import matplotlib.pyplot as plt
import numpy as np
from kivy.clock import Clock
from kivy_garden.graph import Graph, LinePlot
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.properties import StringProperty, NumericProperty, ObjectProperty, BooleanProperty

import socket
import json

from kivy.uix.togglebutton import ToggleButton


class CircularArray:
    def __init__(self, size_or_numpy_array):
        if type(size_or_numpy_array) == int:
            self._size = size_or_numpy_array
            self._data = np.zeros((self._size,))
        elif type(size_or_numpy_array) == np.ndarray:
            self._size = np.shape(size_or_numpy_array)[0]
            self._data = size_or_numpy_array
        self._first = 0
        self._last = self._size - 1
        self._curr_iter_idx = 0
        self._assigned_amount = 0

    def __iter__(self):
        self._curr_iter_idx = -1
        return self

    def __next__(self):
        if self._curr_iter_idx + 1 >= min(self._size, self._assigned_amount):
            raise StopIteration
        else:
            self._curr_iter_idx +=1

            return self._data[(self._curr_iter_idx + self._first) % self._size]

    def __getitem__(self, key):
        if key >= self._size:
            raise IndexError('list index out of range')
        idx = (key + self._first) % min(self._size, self._assigned_amount)
        return self._data[idx]

    def __len__(self):
        return self._size

    def __repr__(self):
        return str(self.to_numpy())

    def append(self, value):
            if self._assigned_amount < self._size:
                self._data[self._assigned_amount] = value
                self._assigned_amount += 1
            else:
                self._data[self._first] = value
                self._first = (self._first + 1) % self._size
                self._last = (self._last + 1) % self._size

    def to_numpy(self):
        return np.asarray([v for v in self])

class LabelText(BoxLayout):
    name = StringProperty('name')
    value = StringProperty('value')
    pass

class LEDVisualizer(ToggleButton):
    def set(self):
        self.state = 'down'
    def unset(self):
        self.state = 'normal'
    def get(self):
        return self.state == 'down'

class AtiInterfaceApp(App):
    def build(self):
        return AtiInterfaceRoot()

class AtiInterfaceRoot(BoxLayout):
    status_connection = StringProperty('')
    ip = StringProperty('192.168.7.2')
    port = NumericProperty(8080)
    connected_toggle_button = ObjectProperty()
    serial_number = ObjectProperty()
    firmware_version = ObjectProperty()
    counts_per_force = ObjectProperty()
    counts_per_torque = ObjectProperty()
    force_unit = ObjectProperty()
    torque_unit = ObjectProperty()
    graph_fx = ObjectProperty()
    graph_fy = ObjectProperty()
    graph_fz = ObjectProperty()
    graph_tx = ObjectProperty()
    graph_ty = ObjectProperty()
    graph_tz = ObjectProperty()
    try_connection_btn_text = StringProperty('Try Connection')
    connection_status_text = StringProperty('Disconnected')

    status_dict =   {
        'b00WatchDogReset': BooleanProperty(),
        'b01DAC_ADC_tooHigh': BooleanProperty(),
        'b02DAC_ADC_tooLow': BooleanProperty(),
        'b03ArtificialAnalogGroundOutOfRange': BooleanProperty(),
        'b04PowerSupplyHigh': BooleanProperty(),
        'b05PowerSupplyLow': BooleanProperty(),
        'b06BadActiveCalibration': BooleanProperty(),
        'b07EEPROMFailure': BooleanProperty(),
        'b08ConfigurationInvalid': BooleanProperty(),
        'b11SensorTempHigh': BooleanProperty(),
        'b12SensorTempLow': BooleanProperty(),
        'b14CANBusError': BooleanProperty(),
        'b15AnyError': BooleanProperty(),
    }

    UDPClientSocket = None
    bufferSize= 1024
    connection_failures = 0

    circularBufferSize = 100

    ft_names = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']

    current_time = 0

    is_connected = False

    def __init__(self, **kwargs):
        super(AtiInterfaceRoot, self).__init__(**kwargs)
        self.graphs = [   self.graph_fx,
                          self.graph_fy,
                          self.graph_fz,
                          self.graph_tx,
                          self.graph_ty,
                          self.graph_tz]
        self.ft_array_point_pair=[CircularArray(np.zeros((self.circularBufferSize, 2))) for _ in self.ft_names]

        for i, g in enumerate(self.graphs):
            g.ylabel = self.ft_names[i][0].upper() + self.ft_names[i][1]
            g.x_ticks_minor=5
            g.x_ticks_major=25
            g.y_ticks_major=1
            g.y_grid_label=True
            g.x_grid_label=True
            g.padding=5
            g.x_grid=True
            g.y_grid=True
            g.ymin=-1
            g.ymax=1
            g.add_plot(LinePlot())

    def graph_update(self, dt):
        try:
            msgFromServer = self.UDPClientSocket.recvfrom(self.bufferSize)
        except:
            return
        msgJson = json.loads(msgFromServer[0])
        if msgJson['Type'] == 'status_force_torque':
            print("new message")
            for g, ft_point_pair, ft_value in zip(self.graphs,self.ft_array_point_pair, msgJson['FT']):
                ft_point_pair.append([self.current_time, ft_value])
                g.ymin = min(g.ymin, float(ft_point_pair[-1][1]))
                g.ymax = max(g.ymax, float(ft_point_pair[-1][1]))
                g.xmax = max(g.xmax, self.current_time)
                g.xmin = g.xmax - self.circularBufferSize
                p = g.plots[0]
                p.points = list(ft_point_pair.to_numpy()) #TODO: resolve warning from here (it appears when len(p.points) > self.circularBufferSize)
            #print("id: {}".format(msgJson['SGcount']))

            for i, key in enumerate(self.status_dict.keys()):
                if msgJson['Status'][i] == 'True':
                    self.status_dict[key] = True
                    self.ids[key].status = 'down'
                else:
                    self.status_dict[key] = False
                    self.ids[key].status = 'normal'

        elif msgJson['Type'] == 'sensor_info':
            self.serial_number.value = msgJson['Serial']
            self.firmware_version.value = "v{}.{}.{}".format(
                msgJson['Firmware_version']['major'],
                msgJson['Firmwasfre_version']['minor'],
                msgJson['Firmware_version']['build'])
            self.counts_per_force.value = str(msgJson['Counts_per']['force'])
            self.counts_per_torque.value = str(msgJson['Counts_per']['torque'])
            self.force_unit.value = msgJson['Unit']['force']
            self.torque_unit.value = msgJson['Unit']['torque']

        if msgJson['Last_message'] == 'True':
            self.UDPClientSocket.close()
            Clock.unschedule(self.graph_update_evet)
            self.connection_status_text = "Not Connected"
            self.ids.connected_toggle_button.state = 'normal'
            self.ids.connected_toggle_button.test = 'Disconnected'
            self.ids.port.disabled = False
            self.ids.ip.disabled = False
            self.try_connection_btn_text = "Try Connection"
            self.is_connected = False

        self.current_time += 1

    def setup_connection(self, button):
        if not self.is_connected:
            msgFromClient = "hello ati-netcanoem"
            bytesToSend = str.encode(msgFromClient)

            # Create a UDP socket at client side
            self.UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            self.UDPClientSocket.settimeout(0) # non blocking!

            inputs = [self.UDPClientSocket]
            outputs = []
            timeout = 0.5

            # Send to server using created UDP socket
            self.UDPClientSocket.sendto(bytesToSend, (self.ip, int(self.port)))

            self.ids.connection_status_text = "Trying connection"
            readable, writable, exceptional = select.select(inputs, outputs, inputs, timeout)

            if self.UDPClientSocket in readable:
                msgFromServer = self.UDPClientSocket.recvfrom(self.bufferSize)

                msgJson = json.loads(msgFromServer[0])

                try:
                    if msgJson['Type'] == 'sensor_info':
                        self.serial_number.value = msgJson['Serial']
                        self.firmware_version.value = "v{}.{}.{}".format(
                            msgJson['Firmware_version']['major'],
                            msgJson['Firmware_version']['minor'],
                            msgJson['Firmware_version']['build'])
                        self.counts_per_force.value = str(msgJson['Counts_per']['force'])
                        self.counts_per_torque.value = str(msgJson['Counts_per']['torque'])
                        self.force_unit.value = msgJson['Unit']['force']
                        self.torque_unit.value = msgJson['Unit']['torque']

                        self.connection_failures = 0
                        self.ids.connection_status_text = "Connection success!"
                        self.ids.connected_toggle_button.state = 'down'
                        self.ids.connected_toggle_button.test = 'Connected'
                        self.try_connection_btn_text = "Disconnect"
                        self.ids.port.disabled = True
                        self.ids.ip.disabled = True

                        self.is_connected = True

                        self.graph_update_evet = Clock.schedule_interval(self.graph_update, 0.01)
                except:
                   self.connection_failures += 1
                   self.ids.connection_status.text = "Couldn't connect ({})".format(self.connection_failures)
            else:
                self.connection_failures += 1
                self.ids.connection_status.text = "Couldn't connect ({})".format(self.connection_failures)
        else:
            bytesToSend = str.encode('end UDP communication')
            self.UDPClientSocket.sendto(bytesToSend, (self.ip, int(self.port)))
            print(bytesToSend, (self.ip, int(self.port)))


if __name__ == '__main__':
    AtiInterfaceApp().run()