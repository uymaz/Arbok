import logging
from typing import List

class Memory:
    def __init__(self, rom: List[int]):
        """
        Initialize memory objet.

        :param rom: The ROM in binary format.
        """
        logging.debug("Memory::init called")

        self.rom = rom
        self.ram = [0] * 0x2000 #8192B
        self.vram = [0] * 0x2000

        self.wram_c = [0] * 0x1000
        self.wram_d = [0] * 0x1000

        self.io = [0] * 0x80
        self.high_ram = [0] * (0xFE - 0x7F)
        self.interrupt_enable = 0


    
    def read_byte(self, address):
        # Read a byte from memory at the given address
        logging.debug("Memory::read_byte called")
    
    def write_byte(self, address, value):
        # Write a byte to memory at the given address
        logging.debug("Memory::write_byte called")