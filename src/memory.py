import logging
from typing import List


class ROMHeader:
    def __init__(self, rom: bytearray):
        """
        Initialize ROMHeader object.

        :param rom: The ROM header in binary format.
        """
        self.title = rom[0x134:0x143].decode("utf-8")
        self.manufacturer_code = rom[0x13F:0x143].decode("utf-8")
        self.cgb_flag = rom[0x143]
        self.new_licensee_code = rom[0x144:0x146].decode("utf-8")
        self.sgb_flag = rom[0x146]
        self.cartridge_type = rom[0x147]
        self.rom_size = rom[0x148]
        self.ram_size = rom[0x149]
        self.destination_code = rom[0x14A]
        self.old_licensee_code = rom[0x14B]
        self.mask_rom_version_number = rom[0x14C]
        self.header_checksum = rom[0x14D]
        self.global_checksum = rom[0x14E:0x150]


class Memory:
    def __init__(self, rom: bytearray):
        """
        Initialize Memory object. Memory is divided into several regions:
        - ROM Bank 0 and switchable ROM bank
        - VRAM
        - Internal RAM
        - Echo RAM
        - OAM (Object Attribute Memory)
        - I/O Registers
        - High RAM
        - Interrupt Enable Register

        Refer to pandocs

        :param rom: The ROM in binary format.
        """
        self.rom = rom
        self.ram = [0] * 0x2000  # 8192B
        self.vram = [0] * 0x2000

        self.wram_c = [0] * 0x1000
        self.wram_d = [0] * 0x1000

        self.io = [0] * 0x80
        self.high_ram = [0] * (0xFE - 0x7F)
        self.interrupt_enable = 0

        self.header = ROMHeader(rom)

    def read_word(self, address):
        # Read a word from memory at the given address
        return self.read_byte(address) | (self.read_byte(address + 1) << 8)

    def write_word(self, address, value):
        # Write a word to memory at the given address
        self.write_byte(address, value & 0xFF)
        self.write_byte(address + 1, (value >> 8) & 0xFF)

    def read_byte(self, address):
        # Read a byte from memory at the given address
        if 0x0000 <= address <= 0x7FFF:
            # ROM Bank 0 and switchable ROM bank
            return self.rom[address]
        elif 0x8000 <= address <= 0x9FFF:
            # VRAM
            return self.vram[address - 0x8000]
        elif 0xC000 <= address <= 0xDFFF:
            # Internal RAM
            return self.ram[address - 0xC000]
        elif 0xE000 <= address <= 0xFDFF:
            # Echo RAM (mirror of 0xC000 - 0xDDFF)
            return self.ram[address - 0xE000]
        elif 0xFE00 <= address <= 0xFE9F:
            # OAM (Object Attribute Memory)
            pass  # Not implemented yet
        elif 0xFF00 <= address <= 0xFF7F:
            # I/O Registers
            return self.io[address - 0xFF00]
        elif 0xFF80 <= address <= 0xFFFE:
            # High RAM
            return self.high_ram[address - 0xFF80]
        elif address == 0xFFFF:
            # Interrupt Enable Register
            return self.interrupt_enable
        else:
            raise ValueError(f"Unsupported memory read at address {hex(address)}")

    def write_byte(self, address, value):
        # Write a byte to memory at the given address
        if 0x0000 <= address <= 0x7FFF:
            # future MBC?
            pass  # Not implemented yet
        elif 0x8000 <= address <= 0x9FFF:
            # VRAM
            self.vram[address - 0x8000] = value
        elif 0xC000 <= address <= 0xDFFF:
            # Internal RAM
            self.ram[address - 0xC000] = value
        elif 0xE000 <= address <= 0xFDFF:
            # Echo RAM (mirror of 0xC000 - 0xDDFF)
            self.ram[address - 0xE000] = value
        elif 0xFE00 <= address <= 0xFE9F:
            # OAM (Object Attribute Memory)
            pass  # Not implemented yet
        elif 0xFF00 <= address <= 0xFF7F:
            # I/O Registers
            self.io[address - 0xFF00] = value
        elif 0xFF80 <= address <= 0xFFFE:
            # High RAM
            self.high_ram[address - 0xFF80] = value
        elif address == 0xFFFF:
            # Interrupt Enable Register
            self.interrupt_enable = value
        else:
            raise ValueError(f"Unsupported memory write at address {hex(address)}")
