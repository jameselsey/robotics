import board
import busio
import digitalio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from enum import Enum
from gc9d01_library import GC9D01

class Mode(Enum):
    IDLE = 1
    STT = 2
    TTS = 3

def rgb_to_565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

class Screen(Node):
    def __init__(self):
        super().__init__('screen')
        self.get_logger().info('Screen node has been started.')
        self.mode = Mode.IDLE
        self.last_mode = None

        # Init display
        spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI)
        cs = digitalio.DigitalInOut(board.D21)
        dc = digitalio.DigitalInOut(board.D25)
        rst = digitalio.DigitalInOut(board.D24)
        self.disp = GC9D01(spi, dc, cs, rst)

        self.create_subscription(Bool, '/stt_active', self.on_stt, 10)
        self.create_subscription(Bool, '/tts_active', self.on_tts, 10)
        self.create_timer(0.05, self.update_display)

    def on_stt(self, msg):
        self.mode = Mode.STT if msg.data else Mode.IDLE

    def on_tts(self, msg):
        self.mode = Mode.TTS if msg.data else Mode.IDLE

    def update_display(self):
        if self.mode != self.last_mode:
            if self.mode == Mode.IDLE:
                color = (0, 0, 128)
            elif self.mode == Mode.STT:
                color = (255, 165, 0)
            elif self.mode == Mode.TTS:
                color = (0, 255, 0)

            color565 = rgb_to_565(*color)
            self.disp.fill_screen(color565)
            self.last_mode = self.mode

def main():
    rclpy.init()
    node = Screen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
