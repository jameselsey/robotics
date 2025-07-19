import time
import math
from enum import Enum

from PIL import Image, ImageDraw
from gc9d01_library import GC9D01
import board
import busio
import digitalio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class Mode(Enum):
    IDLE = 1
    STT = 2
    TTS = 3

class Screen(Node):
    def __init__(self):
        super().__init__('screen')
        self.get_logger().info('Screen node has been started.')
        self.mode = Mode.IDLE
        self.phase = 0.0

        # Init display
        spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI)
        cs = digitalio.DigitalInOut(board.D21)
        dc = digitalio.DigitalInOut(board.D25)
        rst = digitalio.DigitalInOut(board.D24)
        self.disp = GC9D01(spi, dc, cs, rst)
        # self.disp = GC9A01(
        #     spi_bus=0,
        #     spi_device=0,
        #     gpio_dc=25,
        #     gpio_rst=24,
        #     gpio_cs=8,
        #     width=240,
        #     height=240,
        #     rotation=0
        # )
        # self.disp.begin()

        # ROS subscriptions
        self.create_subscription(Bool, '/stt_active', self.on_stt, 10)
        self.create_subscription(Bool, '/tts_active', self.on_tts, 10)

        # Main update timer
        self.create_timer(0.05, self.update_display)

    def on_stt(self, msg):
        self.mode = Mode.STT if msg.data else Mode.IDLE

    def on_tts(self, msg):
        self.mode = Mode.TTS if msg.data else Mode.IDLE

    def update_display(self):
        if self.mode == Mode.IDLE:
            brightness = int(64 + 64 * abs(math.sin(self.phase)))
            color = (0, 0, brightness)
            self.phase += 0.1
        elif self.mode == Mode.STT:
            color = (255, 165, 0)  # Orange
        elif self.mode == Mode.TTS:
            color = (0, 255, 0)    # Green
        else:
            color = (0, 0, 0)

        img = Image.new("RGB", (160, 160), (0, 0, 0))
        draw = ImageDraw.Draw(img)
        draw.ellipse((10, 10, 150, 150), fill=color)
        self.disp.display_image(self.image_to_rgb565(img))
    
    def image_to_rgb565(self, image):
        """Convert a PIL RGB image to a byte buffer in RGB565 format."""
        if image.mode != 'RGB':
            image = image.convert('RGB')

        rgb565 = bytearray()
        for r, g, b in image.getdata():
            value = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            rgb565.append((value >> 8) & 0xFF)
            rgb565.append(value & 0xFF)

        return bytes(rgb565)

def main():
    rclpy.init()
    node = Screen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
