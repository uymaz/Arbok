import logging
from dataclasses import dataclass
from typing import Literal, Optional, List
import json

# Operand and Instruction inspired by the following article
# https://www.inspiredpython.com/course/game-boy-emulator/game-boy-emulator-writing-the-z80-disassembler
@dataclass(frozen=True)
class Operand:
    immediate: bool
    name: str
    bytes: int
    value: Optional[int]
    adjust: Optional[Literal["+", "-"]]

    def create(self, value):
        return Operand(immediate=self.immediate,
                       name=self.name,
                       bytes=self.bytes,
                       value=value,
                       adjust=self.adjust)

    def print(self):
        if self.adjust is None:
            adjust = ""
        else:
            adjust = self.adjust
        if self.value is not None:
            if self.bytes is not None:
                val = hex(self.value)
            else:
                val = self.value
            v = val
        else:
            v = self.name
        v = v + adjust
        if self.immediate:
            return v
        return f'({v})'


@dataclass
class Instruction:
    opcode: int
    immediate: bool
    operands: list[Operand]
    cycles: list[int]
    bytes: int
    mnemonic: str
    comment: str = ""

    def create(self, operands):
        return Instruction(opcode=self.opcode,
                           immediate=self.immediate,
                           operands=operands,
                           cycles=self.cycles,
                           bytes=self.bytes,
                           mnemonic=self.mnemonic)

    def print(self):
        ops = ', '.join(op.print() for op in self.operands)
        s = f"{self.mnemonic:<8} {ops}"
        if self.comment:
            s = s + f" ; {self.comment:<10}"
        return s


class Decoder:
    def load_unprefixed_instructions(self) -> list[Instruction]:
        with open('opcodes.json', 'r') as f:
            data = json.load(f)

        instructions = []
        for opcode, info in data.pop('unprefixed').items():
            operands = [Operand(immediate=op['immediate'], name=op['name'], bytes=op.get('bytes', 0),
                                value=op.get('value', None), adjust=op.get('adjust', None)) for op in info['operands']]
            instruction = Instruction(opcode=int(opcode, 16), immediate=info['immediate'], operands=operands,
                                      cycles=info['cycles'], bytes=info['bytes'], mnemonic=info['mnemonic'])
            instructions.append(instruction)

        return instructions

    def load_cbprefixed_instructions(self) -> list[Instruction]:
        with open('opcodes.json', 'r') as f:
            data = json.load(f)

        instructions = []
        for opcode, info in data.pop('cbprefixed').items():
            operands = [Operand(immediate=op['immediate'], name=op['name'], bytes=op.get('bytes', 0),
                                value=op.get('value', None), adjust=op.get('adjust', None)) for op in info['operands']]
            instruction = Instruction(opcode=int(opcode, 16), immediate=info['immediate'], operands=operands,
                                      cycles=info['cycles'], bytes=info['bytes'], mnemonic=info['mnemonic'])
            instructions.append(instruction)

        return instructions

    def decode(self, opcode: int, instructions: list[Instruction]) -> Instruction:
        for instruction in instructions:
            if instruction.opcode == opcode:
                return instruction
        raise ValueError(f"Opcode {opcode} not found in instructions")


class CPU:
    def __init__(self, memory):
        # Initialize CPU registers, flags, and other state

        # Flags as a dict to make it easier to access them
        # self.flags = {'Z': False, 'N': False, 'H': False, 'C': False}
        self.memory = memory
        self.program_counter = 0x100  # Start execution at ROM address 0x100
        self.registers = { # Initialize registers
            'A': 0x01, 'F': 0xB0,
            'B': 0x00, 'C': 0x13,
            'D': 0x00, 'E': 0xD8,
            'H': 0x01, 'L': 0x4D,
        }
        self.SP = 0xFFFE  # Stack Pointer

    def push(self, value: int):
        self.SP = (self.SP - 1) & 0xFFFF
        self.memory.write_byte(self.SP, (value >> 8) & 0xFF)
        self.SP = (self.SP - 1) & 0xFFFF
        self.memory.write_byte(self.SP, value & 0xFF)

    def pop(self) -> int:
        value = self.memory.read_byte(self.SP)
        self.SP = (self.SP + 1) & 0xFFFF
        value |= self.memory.read_byte(self.SP) << 8
        self.SP = (self.SP + 1) & 0xFFFF
        return value

    def fetch_instruction(self):
        return self.memory.read_byte(self.program_counter)

    #used for BC and HL registers, mainly
    #Returns the value of (register_a << 8) + register_b
    def lookup_16bit_register(self, register_a, register_b):
        return (register_a << 8) + register_b

    def decode_and_execute(self, instruction):
        instruction = self.fetch_instruction()
        decoder = Decoder()
        decoder.load_unprefixed_instructions()

        logging.debug(f"{decoder.decode(instruction, decoder.load_unprefixed_instructions()).print()}")

        opcode = instruction & 0xFF  # Extract the opcode from the instruction
        if opcode == 0x00:
            # NOP
            self.program_counter += 1
            pass
        elif opcode == 0x01:
            # LD BC, nn: Load 16-bit immediate value into BC register pair
            immediate_value = self.memory.read_word(self.program_counter)
            self.program_counter += 2
            self.registers['BC'] = immediate_value
        elif opcode == 0x02:
            # LD (BC), A: Store the contents of register A into memory address specified by BC
            address = self.registers['BC']
            self.memory.write_byte(address, self.registers['A'])
        elif opcode == 0x03:
            # INC BC: Increment BC by one
            address = self.registers['BC']
            self.program_counter += 1
        elif opcode == 0x04:
            address = self.registers['B']
            self.program_counter += 1
        elif opcode == 0x05:
            # DEC B: Decrement B by one
            self.registers['B'] -= 1
            self.program_counter += 1
        elif opcode == 0x06:
            # LD B, n: Load 8-bit immediate value into register B
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['B'] = immediate_value
            self.program_counter += 1
        elif opcode == 0x07:
            # RLCA: Rotate A left through carry
            carry = (self.registers['A'] & 0x80) >> 7
            self.registers['A'] = ((self.registers['A'] << 1) & 0xFF) | carry
            self.registers['F'] = 0  # Reset flags
            if carry:
                self.registers['F'] |= 0x10  # Set carry flag
            self.program_counter += 1
        elif opcode == 0x08:
            # LD (nn), SP: Store stack pointer at address nn
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            self.memory.write_word(address, self.registers['SP'])
            self.program_counter += 2
        elif opcode == 0x09:
            # ADD HL, BC: Add BC to HL
            hl = self.registers['HL']
            bc = self.registers['BC']
            result = hl + bc
            self.registers['HL'] = result & 0xFFFF  # 16-bit wrap-around
            # Set flags accordingly
            self.registers['F'] &= 0x80  # Clear N flag and keep Z flag intact
            if result > 0xFFFF:
                self.registers['F'] |= 0x10  # Set carry flag
            if (hl & 0xFFF) + (bc & 0xFFF) > 0xFFF:
                self.registers['F'] |= 0x20  # Set half-carry flag
            self.program_counter += 1
        elif opcode == 0x0A:
            # LD A, (BC): Load value from address in BC into A
            address = self.registers['BC']
            self.registers['A'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x0B:
            # DEC BC: Decrement BC by one
            self.registers['BC'] -= 1
            self.program_counter += 1
        elif opcode == 0x0C:
            # INC C: Increment C by one
            self.registers['C'] += 1
            self.program_counter += 1
        elif opcode == 0x0D:
            # DEC C: Decrement C by one
            self.registers['C'] -= 1
            self.program_counter += 1
        elif opcode == 0x0E:
            # LD C, n: Load 8-bit immediate value into register C
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['C'] = immediate_value
            self.program_counter += 1
        elif opcode == 0x0F:
            # RRCA: Rotate A right through carry
            carry = self.registers['A'] & 0x01
            self.registers['A'] = (self.registers['A'] >> 1) | (carry << 7)
            self.registers['F'] = 0  # Reset all flags
            if carry:
                self.registers['F'] |= 0x10  # Set carry flag
            self.program_counter += 1

        elif opcode == 0x3E:
            # LD A, n: Load 8-bit immediate value into register A
            immediate_value = self.memory.read_byte(self.program_counter)
            self.program_counter += 1
            self.registers['A'] = immediate_value
        elif opcode == 0xAF:
            # XOR A: Bitwise XOR register A with itself (A ^ A)
            self.registers['A'] ^= self.registers['A']
            self.program_counter += 1
        elif opcode == 0x10:
            # STOP 0: Halt CPU and display until a button is pressed
            self.halted = True
            self.program_counter += 1
        elif opcode == 0x11:
            # LD DE, nn: Load 16-bit immediate value into DE register pair
            self.program_counter += 1
            immediate_value = self.memory.read_word(self.program_counter)
            self.registers['DE'] = immediate_value
            self.program_counter += 2
        elif opcode == 0x12:
            # LD (DE), A: Store the contents of register A into memory address specified by DE
            address = self.registers['DE']
            self.memory.write_byte(address, self.registers['A'])
            self.program_counter += 1
        elif opcode == 0x13:
            # INC DE: Increment DE by one
            self.registers['DE'] += 1
            self.program_counter += 1
        elif opcode == 0x14:
            # INC D: Increment D by one
            self.registers['D'] += 1
            self.program_counter += 1
        elif opcode == 0x15:
            # DEC D: Decrement D by one
            self.registers['D'] -= 1
            self.program_counter += 1
        elif opcode == 0x16:
            # LD D, n: Load 8-bit immediate value into register D
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['D'] = immediate_value
            self.program_counter += 1
        elif opcode == 0x17:
            # RLA: Rotate A left through carry flag
            carry = (self.registers['A'] & 0x80) >> 7
            old_carry = (self.registers['F'] & 0x10) >> 4
            self.registers['A'] = ((self.registers['A'] << 1) & 0xFF) | old_carry
            self.registers['F'] = 0  # Reset all flags
            if carry:
                self.registers['F'] |= 0x10  # Set carry flag
            self.program_counter += 1
        elif opcode == 0x18:
            # JR n: Jump relative by signed immediate value
            self.program_counter += 1
            offset = self.memory.read_byte(self.program_counter)
            if offset > 127:
                offset -= 256  # Signed 8-bit value
            self.program_counter += offset + 1
        elif opcode == 0x19:
            # ADD HL, DE: Add DE to HL
            hl = self.registers['HL']
            de = self.registers['DE']
            result = hl + de
            self.registers['HL'] = result & 0xFFFF
            # Set flags
            self.registers['F'] &= 0x80  # Clear N and keep Z flag intact
            if result > 0xFFFF:
                self.registers['F'] |= 0x10  # Set carry flag
            if (hl & 0xFFF) + (de & 0xFFF) > 0xFFF:
                self.registers['F'] |= 0x20  # Set half-carry flag
            self.program_counter += 1
        elif opcode == 0x1A:
            # LD A, (DE): Load value from address in DE into A
            address = self.registers['DE']
            self.registers['A'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x1B:
            # DEC DE: Decrement DE by one
            self.registers['DE'] -= 1
            self.program_counter += 1
        elif opcode == 0x1C:
            # INC E: Increment E by one
            self.registers['E'] += 1
            self.program_counter += 1
        elif opcode == 0x1D:
            # DEC E: Decrement E by one
            self.registers['E'] -= 1
            self.program_counter += 1
        elif opcode == 0x1E:
            # LD E, n: Load 8-bit immediate value into register E
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['E'] = immediate_value
            self.program_counter += 1
        elif opcode == 0x1F:
            # RRA: Rotate A right through carry flag
            carry = self.registers['A'] & 0x01
            old_carry = (self.registers['F'] & 0x10) >> 4
            self.registers['A'] = (self.registers['A'] >> 1) | (old_carry << 7)
            self.registers['F'] = 0  # Reset all flags
            if carry:
                self.registers['F'] |= 0x10  # Set carry flag
            self.program_counter += 1
        elif opcode == 0x20:
            # JR NZ, n: Jump relative if Zero flag is not set
            self.program_counter += 1
            offset = self.memory.read_byte(self.program_counter)
            if offset > 127:
                offset -= 256  # Signed 8-bit value
            if not self.registers['F'] & 0x80:  # Zero flag not set
                self.program_counter += offset
            self.program_counter += 1
        elif opcode == 0x21:
            # LD HL, nn: Load 16-bit immediate value into HL register pair
            self.program_counter += 1
            immediate_value = self.memory.read_word(self.program_counter)
            self.registers['HL'] = immediate_value
            self.program_counter += 2
        elif opcode == 0x22:
            # LD (HL+), A: Store the contents of A into memory at HL and increment HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['A'])
            self.registers['HL'] = (self.registers['HL'] + 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x23:
            # INC HL: Increment HL by one
            self.registers['HL'] = (self.registers['HL'] + 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x24:
            # INC H: Increment H by one
            self.registers['H'] = (self.registers['H'] + 1) & 0xFF
            self.program_counter += 1
        elif opcode == 0x25:
            # DEC H: Decrement H by one
            self.registers['H'] = (self.registers['H'] - 1) & 0xFF
            self.program_counter += 1
        elif opcode == 0x26:
            # LD H, n: Load 8-bit immediate value into register H
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['H'] = immediate_value
            self.program_counter += 1
        elif opcode == 0x27:
            # DAA: Decimal adjust A for BCD addition
            a = self.registers['A']
            if not (self.registers['F'] & 0x40):  # N flag is not set (addition)
                if (self.registers['F'] & 0x20) or (a & 0x0F) > 9:
                    a += 0x06
                if (self.registers['F'] & 0x10) or (a > 0x9F):
                    a += 0x60
            else:  # N flag is set (subtraction)
                if self.registers['F'] & 0x20:
                    a = (a - 0x06) & 0xFF
                if self.registers['F'] & 0x10:
                    a = (a - 0x60) & 0xFF
            self.registers['A'] = a & 0xFF
            # Set flags
            self.registers['F'] &= 0x40  # N flag stays, clear other flags
            if self.registers['A'] == 0:
                self.registers['F'] |= 0x80  # Set Z flag
            if a & 0x100:
                self.registers['F'] |= 0x10  # Set C flag
            self.program_counter += 1
        elif opcode == 0x28:
            # JR Z, n: Jump relative if Zero flag is set
            self.program_counter += 1
            offset = self.memory.read_byte(self.program_counter)
            if offset > 127:
                offset -= 256  # Signed 8-bit value
            if self.registers['F'] & 0x80:  # Zero flag set
                self.program_counter += offset
            self.program_counter += 1
        elif opcode == 0x29:
            # ADD HL, HL: Add HL to itself
            hl = self.registers['HL']
            result = hl + hl
            self.registers['HL'] = result & 0xFFFF
            # Set flags
            self.registers['F'] &= 0x80  # Clear N and keep Z flag intact
            if result > 0xFFFF:
                self.registers['F'] |= 0x10  # Set carry flag
            if (hl & 0xFFF) + (hl & 0xFFF) > 0xFFF:
                self.registers['F'] |= 0x20  # Set half-carry flag
            self.program_counter += 1
        elif opcode == 0x2A:
            # LD A, (HL+): Load value from memory at HL into A and increment HL
            address = self.registers['HL']
            self.registers['A'] = self.memory.read_byte(address)
            self.registers['HL'] = (self.registers['HL'] + 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x2B:
            # DEC HL: Decrement HL by one
            self.registers['HL'] = (self.registers['HL'] - 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x2C:
            # INC L: Increment L by one
            self.registers['L'] = (self.registers['L'] + 1) & 0xFF
            self.program_counter += 1
        elif opcode == 0x2D:
            # DEC L: Decrement L by one
            self.registers['L'] = (self.registers['L'] - 1) & 0xFF
            self.program_counter += 1
        elif opcode == 0x2E:
            # LD L, n: Load 8-bit immediate value into register L
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['L'] = immediate_value
            self.program_counter += 1
        elif opcode == 0x2F:
            # CPL: Complement A (Flip all bits in A)
            self.registers['A'] ^= 0xFF
            # Set N and H flags
            self.registers['F'] |= 0x60  # Set N and H flags
            self.program_counter += 1
        elif opcode == 0x30:
            # JR NC, n: Jump relative if Carry flag is not set
            self.program_counter += 1
            offset = self.memory.read_byte(self.program_counter)
            if offset > 127:
                offset -= 256  # Signed 8-bit value
            if not (self.registers['F'] & 0x10):  # Carry flag not set
                self.program_counter += offset
            self.program_counter += 1
        elif opcode == 0x31:
            # LD SP, nn: Load 16-bit immediate value into Stack Pointer
            self.program_counter += 1
            immediate_value = self.memory.read_word(self.program_counter)
            self.registers['SP'] = immediate_value
            self.program_counter += 2
        elif opcode == 0x32:
            # LD (HL-), A: Store the contents of A into memory at HL, then decrement HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['A'])
            self.registers['HL'] = (self.registers['HL'] - 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x33:
            # INC SP: Increment Stack Pointer by one
            self.registers['SP'] = (self.registers['SP'] + 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x34:
            # INC (HL): Increment the value at the address in HL
            address = self.registers['HL']
            value = self.memory.read_byte(address)
            value = (value + 1) & 0xFF
            self.memory.write_byte(address, value)
            # Set flags
            self.registers['F'] &= 0x10  # Clear N flag, keep C flag
            if value == 0:
                self.registers['F'] |= 0x80  # Set Z flag
            if (value & 0x0F) == 0x00:
                self.registers['F'] |= 0x20  # Set H flag
            self.program_counter += 1
        elif opcode == 0x35:
            # DEC (HL): Decrement the value at the address in HL
            address = self.registers['HL']
            value = self.memory.read_byte(address)
            value = (value - 1) & 0xFF
            self.memory.write_byte(address, value)
            # Set flags
            self.registers['F'] &= 0x10  # Keep C flag, clear N flag
            if value == 0:
                self.registers['F'] |= 0x80  # Set Z flag
            self.registers['F'] |= 0x40  # Set N flag (since it's a decrement)
            if (value & 0x0F) == 0x0F:
                self.registers['F'] |= 0x20  # Set H flag
            self.program_counter += 1
        elif opcode == 0x36:
            # LD (HL), n: Load 8-bit immediate value into the memory address specified by HL
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.memory.write_byte(self.registers['HL'], immediate_value)
            self.program_counter += 1
        elif opcode == 0x37:
            # SCF: Set Carry flag
            self.registers['F'] &= 0x80  # Clear N and H, keep Z
            self.registers['F'] |= 0x10  # Set C flag
            self.program_counter += 1
        elif opcode == 0x38:
            # JR C, n: Jump relative if Carry flag is set
            self.program_counter += 1
            offset = self.memory.read_byte(self.program_counter)
            if offset > 127:
                offset -= 256  # Signed 8-bit value
            if self.registers['F'] & 0x10:  # Carry flag set
                self.program_counter += offset
            self.program_counter += 1
        elif opcode == 0x39:
            # ADD HL, SP: Add SP to HL
            hl = self.registers['HL']
            sp = self.registers['SP']
            result = hl + sp
            self.registers['HL'] = result & 0xFFFF
            # Set flags
            self.registers['F'] &= 0x80  # Clear N, keep Z flag intact
            if result > 0xFFFF:
                self.registers['F'] |= 0x10  # Set carry flag
            if (hl & 0xFFF) + (sp & 0xFFF) > 0xFFF:
                self.registers['F'] |= 0x20  # Set half-carry flag
            self.program_counter += 1
        elif opcode == 0x3A:
            # LD A, (HL-): Load value from memory at HL into A and decrement HL
            address = self.registers['HL']
            self.registers['A'] = self.memory.read_byte(address)
            self.registers['HL'] = (self.registers['HL'] - 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x3B:
            # DEC SP: Decrement Stack Pointer by one
            self.registers['SP'] = (self.registers['SP'] - 1) & 0xFFFF
            self.program_counter += 1
        elif opcode == 0x3C:
            # INC A: Increment A by one
            self.registers['A'] = (self.registers['A'] + 1) & 0xFF
            # Set flags
            self.registers['F'] &= 0x10  # Clear N, keep C flag intact
            if self.registers['A'] == 0:
                self.registers['F'] |= 0x80  # Set Z flag
            if (self.registers['A'] & 0x0F) == 0:
                self.registers['F'] |= 0x20  # Set half-carry flag
            self.program_counter += 1
        elif opcode == 0x3D:
            # DEC A: Decrement A by one
            self.registers['A'] = (self.registers['A'] - 1) & 0xFF
            # Set flags
            self.registers['F'] &= 0x10  # Keep C flag, clear N flag
            if self.registers['A'] == 0:
                self.registers['F'] |= 0x80  # Set Z flag
            self.registers['F'] |= 0x40  # Set N flag (since it's a decrement)
            if (self.registers['A'] & 0x0F) == 0x0F:
                self.registers['F'] |= 0x20  # Set half-carry flag
            self.program_counter += 1
        elif opcode == 0x3E:
            # LD A, n: Load 8-bit immediate value into register A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['A'] = immediate_value
            self.program_counter += 1
        elif opcode == 0x3F:
            # CCF: Complement Carry Flag
            if self.registers['F'] & 0x10:  # If Carry is set, unset it
                self.registers['F'] &= 0xEF
            else:  # If Carry is not set, set it
                self.registers['F'] |= 0x10
            self.registers['F'] &= 0x80  # Keep Z flag, clear N and H flags
            self.program_counter += 1
        elif opcode == 0x40:
            # LD B, B: Load B into B (basically a NOP)
            self.program_counter += 1
        elif opcode == 0x41:
            # LD B, C: Load C into B
            self.registers['B'] = self.registers['C']
            self.program_counter += 1
        elif opcode == 0x42:
            # LD B, D: Load D into B
            self.registers['B'] = self.registers['D']
            self.program_counter += 1
        elif opcode == 0x43:
            # LD B, E: Load E into B
            self.registers['B'] = self.registers['E']
            self.program_counter += 1
        elif opcode == 0x44:
            # LD B, H: Load H into B
            self.registers['B'] = self.registers['H']
            self.program_counter += 1
        elif opcode == 0x45:
            # LD B, L: Load L into B
            self.registers['B'] = self.registers['L']
            self.program_counter += 1
        elif opcode == 0x46:
            # LD B, (HL): Load value at memory address HL into B
            address = self.registers['HL']
            self.registers['B'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x47:
            # LD B, A: Load A into B
            self.registers['B'] = self.registers['A']
            self.program_counter += 1
        elif opcode == 0x48:
            # LD C, B: Load B into C
            self.registers['C'] = self.registers['B']
            self.program_counter += 1
        elif opcode == 0x49:
            # LD C, C: Load C into C (basically a NOP)
            self.program_counter += 1
        elif opcode == 0x4A:
            # LD C, D: Load D into C
            self.registers['C'] = self.registers['D']
            self.program_counter += 1
        elif opcode == 0x4B:
            # LD C, E: Load E into C
            self.registers['C'] = self.registers['E']
            self.program_counter += 1
        elif opcode == 0x4C:
            # LD C, H: Load H into C
            self.registers['C'] = self.registers['H']
            self.program_counter += 1
        elif opcode == 0x4D:
            # LD C, L: Load L into C
            self.registers['C'] = self.registers['L']
            self.program_counter += 1
        elif opcode == 0x4E:
            # LD C, (HL): Load value at memory address HL into C
            address = self.registers['HL']
            self.registers['C'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x4F:
            # LD C, A: Load A into C
            self.registers['C'] = self.registers['A']
            self.program_counter += 1
        elif opcode == 0x50:
            # LD D, B: Load B into D
            self.registers['D'] = self.registers['B']
            self.program_counter += 1
        elif opcode == 0x51:
            # LD D, C: Load C into D
            self.registers['D'] = self.registers['C']
            self.program_counter += 1
        elif opcode == 0x52:
            # LD D, D: Load D into D (NOP)
            self.program_counter += 1
        elif opcode == 0x53:
            # LD D, E: Load E into D
            self.registers['D'] = self.registers['E']
            self.program_counter += 1
        elif opcode == 0x54:
            # LD D, H: Load H into D
            self.registers['D'] = self.registers['H']
            self.program_counter += 1
        elif opcode == 0x55:
            # LD D, L: Load L into D
            self.registers['D'] = self.registers['L']
            self.program_counter += 1
        elif opcode == 0x56:
            # LD D, (HL): Load value from memory address in HL into D
            address = self.registers['HL']
            self.registers['D'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x57:
            # LD D, A: Load A into D
            self.registers['D'] = self.registers['A']
            self.program_counter += 1
        elif opcode == 0x58:
            # LD E, B: Load B into E
            self.registers['E'] = self.registers['B']
            self.program_counter += 1
        elif opcode == 0x59:
            # LD E, C: Load C into E
            self.registers['E'] = self.registers['C']
            self.program_counter += 1
        elif opcode == 0x5A:
            # LD E, D: Load D into E
            self.registers['E'] = self.registers['D']
            self.program_counter += 1
        elif opcode == 0x5B:
            # LD E, E: Load E into E (NOP)
            self.program_counter += 1
        elif opcode == 0x5C:
            # LD E, H: Load H into E
            self.registers['E'] = self.registers['H']
            self.program_counter += 1
        elif opcode == 0x5D:
            # LD E, L: Load L into E
            self.registers['E'] = self.registers['L']
            self.program_counter += 1
        elif opcode == 0x5E:
            # LD E, (HL): Load value from memory address in HL into E
            address = self.registers['HL']
            self.registers['E'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x5F:
            # LD E, A: Load A into E
            self.registers['E'] = self.registers['A']
            self.program_counter += 1
        elif opcode == 0x60:
            # LD H, B: Load B into H
            self.registers['H'] = self.registers['B']
            self.program_counter += 1
        elif opcode == 0x61:
            # LD H, C: Load C into H
            self.registers['H'] = self.registers['C']
            self.program_counter += 1
        elif opcode == 0x62:
            # LD H, D: Load D into H
            self.registers['H'] = self.registers['D']
            self.program_counter += 1
        elif opcode == 0x63:
            # LD H, E: Load E into H
            self.registers['H'] = self.registers['E']
            self.program_counter += 1
        elif opcode == 0x64:
            # LD H, H: Load H into H (NOP)
            self.program_counter += 1
        elif opcode == 0x65:
            # LD H, L: Load L into H
            self.registers['H'] = self.registers['L']
            self.program_counter += 1
        elif opcode == 0x66:
            # LD H, (HL): Load value from memory at HL into H
            address = self.registers['HL']
            self.registers['H'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x67:
            # LD H, A: Load A into H
            self.registers['H'] = self.registers['A']
            self.program_counter += 1
        elif opcode == 0x68:
            # LD L, B: Load B into L
            self.registers['L'] = self.registers['B']
            self.program_counter += 1
        elif opcode == 0x69:
            # LD L, C: Load C into L
            self.registers['L'] = self.registers['C']
            self.program_counter += 1
        elif opcode == 0x6A:
            # LD L, D: Load D into L
            self.registers['L'] = self.registers['D']
            self.program_counter += 1
        elif opcode == 0x6B:
            # LD L, E: Load E into L
            self.registers['L'] = self.registers['E']
            self.program_counter += 1
        elif opcode == 0x6C:
            # LD L, H: Load H into L
            self.registers['L'] = self.registers['H']
            self.program_counter += 1
        elif opcode == 0x6D:
            # LD L, L: Load L into L (NOP)
            self.program_counter += 1
        elif opcode == 0x6E:
            # LD L, (HL): Load value from memory at HL into L
            address = self.registers['HL']
            self.registers['L'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x6F:
            # LD L, A: Load A into L
            self.registers['L'] = self.registers['A']
            self.program_counter += 1
        elif opcode == 0x70:
            # LD (HL), B: Store the contents of B into memory at address HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['B'])
            self.program_counter += 1
        elif opcode == 0x71:
            # LD (HL), C: Store the contents of C into memory at address HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['C'])
            self.program_counter += 1
        elif opcode == 0x72:
            # LD (HL), D: Store the contents of D into memory at address HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['D'])
            self.program_counter += 1
        elif opcode == 0x73:
            # LD (HL), E: Store the contents of E into memory at address HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['E'])
            self.program_counter += 1
        elif opcode == 0x74:
            # LD (HL), H: Store the contents of H into memory at address HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['H'])
            self.program_counter += 1
        elif opcode == 0x75:
            # LD (HL), L: Store the contents of L into memory at address HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['L'])
            self.program_counter += 1
        elif opcode == 0x76:
            # HALT: Stop execution until an interrupt occurs
            self.halted = True
            self.program_counter += 1
        elif opcode == 0x77:
            # LD (HL), A: Store the contents of A into memory at address HL
            address = self.registers['HL']
            self.memory.write_byte(address, self.registers['A'])
            self.program_counter += 1
        elif opcode == 0x78:
            # LD A, B: Load B into A
            self.registers['A'] = self.registers['B']
            self.program_counter += 1
        elif opcode == 0x79:
            # LD A, C: Load C into A
            self.registers['A'] = self.registers['C']
            self.program_counter += 1
        elif opcode == 0x7A:
            # LD A, D: Load D into A
            self.registers['A'] = self.registers['D']
            self.program_counter += 1
        elif opcode == 0x7B:
            # LD A, E: Load E into A
            self.registers['A'] = self.registers['E']
            self.program_counter += 1
        elif opcode == 0x7C:
            # LD A, H: Load H into A
            self.registers['A'] = self.registers['H']
            self.program_counter += 1
        elif opcode == 0x7D:
            # LD A, L: Load L into A
            self.registers['A'] = self.registers['L']
            self.program_counter += 1
        elif opcode == 0x7E:
            # LD A, (HL): Load value from memory at address HL into A
            address = self.registers['HL']
            self.registers['A'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0x7F:
            # LD A, A: Load A into A (NOP)
            self.program_counter += 1
        elif opcode == 0x80:
            # ADD A, B: Add B to A
            self.add(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0x81:
            # ADD A, C: Add C to A
            self.add(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0x82:
            # ADD A, D: Add D to A
            self.add(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0x83:
            # ADD A, E: Add E to A
            self.add(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0x84:
            # ADD A, H: Add H to A
            self.add(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0x85:
            # ADD A, L: Add L to A
            self.add(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0x86:
            # ADD A, (HL): Add value at address HL to A
            value = self.memory.read_byte(self.registers['HL'])
            self.add(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0x87:
            # ADD A, A: Add A to A (double A)
            self.add(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0x88:
            # ADC A, B: Add B + carry flag to A
            self.adc(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0x89:
            # ADC A, C: Add C + carry flag to A
            self.adc(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0x8A:
            # ADC A, D: Add D + carry flag to A
            self.adc(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0x8B:
            # ADC A, E: Add E + carry flag to A
            self.adc(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0x8C:
            # ADC A, H: Add H + carry flag to A
            self.adc(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0x8D:
            # ADC A, L: Add L + carry flag to A
            self.adc(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0x8E:
            # ADC A, (HL): Add value at address HL + carry flag to A
            value = self.memory.read_byte(self.registers['HL'])
            self.adc(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0x8F:
            # ADC A, A: Add A + carry flag to A
            self.adc(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0x90:
            # SUB B: Subtract B from A
            self.sub(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0x91:
            # SUB C: Subtract C from A
            self.sub(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0x92:
            # SUB D: Subtract D from A
            self.sub(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0x93:
            # SUB E: Subtract E from A
            self.sub(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0x94:
            # SUB H: Subtract H from A
            self.sub(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0x95:
            # SUB L: Subtract L from A
            self.sub(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0x96:
            # SUB (HL): Subtract value at address HL from A
            value = self.memory.read_byte(self.registers['HL'])
            self.sub(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0x97:
            # SUB A: Subtract A from A (result is always 0)
            self.sub(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0x98:
            # SBC A, B: Subtract B + carry flag from A
            self.sbc(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0x99:
            # SBC A, C: Subtract C + carry flag from A
            self.sbc(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0x9A:
            # SBC A, D: Subtract D + carry flag from A
            self.sbc(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0x9B:
            # SBC A, E: Subtract E + carry flag from A
            self.sbc(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0x9C:
            # SBC A, H: Subtract H + carry flag from A
            self.sbc(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0x9D:
            # SBC A, L: Subtract L + carry flag from A
            self.sbc(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0x9E:
            # SBC A, (HL): Subtract value at address HL + carry flag from A
            value = self.memory.read_byte(self.registers['HL'])
            self.sbc(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0x9F:
            # SBC A, A: Subtract A + carry flag from A (result is always 0)
            self.sbc(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xA0:
            # AND B: Logical AND B with A
            self.and_op(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0xA1:
            # AND C: Logical AND C with A
            self.and_op(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0xA2:
            # AND D: Logical AND D with A
            self.and_op(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0xA3:
            # AND E: Logical AND E with A
            self.and_op(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0xA4:
            # AND H: Logical AND H with A
            self.and_op(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0xA5:
            # AND L: Logical AND L with A
            self.and_op(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0xA6:
            # AND (HL): Logical AND value at address HL with A
            value = self.memory.read_byte(self.registers['HL'])
            self.and_op(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0xA7:
            # AND A: Logical AND A with A (result is A)
            self.and_op(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xA8:
            # XOR B: Logical XOR B with A
            self.xor_op(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0xA9:
            # XOR C: Logical XOR C with A
            self.xor_op(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0xAA:
            # XOR D: Logical XOR D with A
            self.xor_op(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0xAB:
            # XOR E: Logical XOR E with A
            self.xor_op(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0xAC:
            # XOR H: Logical XOR H with A
            self.xor_op(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0xAD:
            # XOR L: Logical XOR L with A
            self.xor_op(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0xAE:
            # XOR (HL): Logical XOR value at address HL with A
            value = self.memory.read_byte(self.registers['HL'])
            self.xor_op(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0xAF:
            # XOR A: Logical XOR A with A (result is 0)
            self.xor_op(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xB0:
            # OR B: Logical OR B with A
            self.or_op(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0xB1:
            # OR C: Logical OR C with A
            self.or_op(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0xB2:
            # OR D: Logical OR D with A
            self.or_op(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0xB3:
            # OR E: Logical OR E with A
            self.or_op(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0xB4:
            # OR H: Logical OR H with A
            self.or_op(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0xB5:
            # OR L: Logical OR L with A
            self.or_op(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0xB6:
            # OR (HL): Logical OR value at address HL with A
            value = self.memory.read_byte(self.registers['HL'])
            self.or_op(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0xB7:
            # OR A: Logical OR A with A (result is A)
            self.or_op(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xB8:
            # CP B: Compare A with B
            self.cp(self.registers['A'], self.registers['B'])
            self.program_counter += 1
        elif opcode == 0xB9:
            # CP C: Compare A with C
            self.cp(self.registers['A'], self.registers['C'])
            self.program_counter += 1
        elif opcode == 0xBA:
            # CP D: Compare A with D
            self.cp(self.registers['A'], self.registers['D'])
            self.program_counter += 1
        elif opcode == 0xBB:
            # CP E: Compare A with E
            self.cp(self.registers['A'], self.registers['E'])
            self.program_counter += 1
        elif opcode == 0xBC:
            # CP H: Compare A with H
            self.cp(self.registers['A'], self.registers['H'])
            self.program_counter += 1
        elif opcode == 0xBD:
            # CP L: Compare A with L
            self.cp(self.registers['A'], self.registers['L'])
            self.program_counter += 1
        elif opcode == 0xBE:
            # CP (HL): Compare A with value at address HL
            value = self.memory.read_byte(self.registers['HL'])
            self.cp(self.registers['A'], value)
            self.program_counter += 1
        elif opcode == 0xBF:
            # CP A: Compare A with A (result is always zero)
            self.cp(self.registers['A'], self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xC0:
            # RET NZ: Return from subroutine if the Zero flag is not set
            if not self.registers['F'] & 0x80:
                self.program_counter = self.pop()
            else:
                self.program_counter += 1
        elif opcode == 0xC1:
            # POP BC: Pop the top of the stack into register pair BC
            self.registers['C'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.registers['B'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.program_counter += 1
        elif opcode == 0xC2:
            # JP NZ, nn: Jump to address nn if the Zero flag is not set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if not self.registers['F'] & 0x80:
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xC3:
            # JP nn: Unconditional jump to address nn
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            self.program_counter = address
        elif opcode == 0xC4:
            # CALL NZ, nn: Call subroutine at address nn if Zero flag is not set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if not self.registers['F'] & 0x80:
                self.push(self.program_counter + 3)  # +3 because it includes 2 for address and 1 for this opcode TODO STACK POINTER
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xC5:
            # PUSH BC: Push register pair BC onto the stack
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['C'])
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['B'])
            self.program_counter += 1
        elif opcode == 0xC6:
            # ADD A, n: Add immediate value n to A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.add(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xC7:
            # RST 0: Restart at vector 0
            self.push(self.program_counter + 1)
            self.program_counter = 0x0000
        elif opcode == 0xC8:
            # RET Z: Return from subroutine if Zero flag is set
            if self.registers['F'] & 0x80:
                self.program_counter = self.pop()
            else:
                self.program_counter += 1
        elif opcode == 0xC9:
            # RET: Return from subroutine
            self.program_counter = self.pop()
        elif opcode == 0xCA:
            # JP Z, nn: Jump to address nn if the Zero flag is set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if self.registers['F'] & 0x80:
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xCB:
            # This is the prefix CB opcode, which handles extended instructions (not covered yet)
            pass  # Placeholder for extended opcodes
        elif opcode == 0xCC:
            # CALL Z, nn: Call subroutine at address nn if Zero flag is set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if self.registers['F'] & 0x80:
                self.push(self.program_counter + 3)
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xCD:
            # CALL nn: Call subroutine at address nn
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            self.push(self.program_counter + 3)
            self.program_counter = address
        elif opcode == 0xCE:
            # ADC A, n: Add immediate value n and carry flag to A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.adc(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xCF:
            # RST 1: Restart at vector 1
            self.push(self.program_counter + 1)
            self.program_counter = 0x0008
        elif opcode == 0xD0:
            # RET NC: Return from subroutine if Carry flag is not set
            if not self.registers['F'] & 0x10:
                self.program_counter = self.pop()
            else:
                self.program_counter += 1
        elif opcode == 0xD1:
            # POP DE: Pop the top of the stack into register pair DE
            self.registers['E'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.registers['D'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.program_counter += 1
        elif opcode == 0xD2:
            # JP NC, nn: Jump to address nn if Carry flag is not set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if not self.registers['F'] & 0x10:
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xD3:
            self.program_counter += 1
        elif opcode == 0xD4:
            # CALL NC, nn: Call subroutine at address nn if Carry flag is not set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if not self.registers['F'] & 0x10:
                self.push(self.program_counter + 3)
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xD5:
            # PUSH DE: Push register pair DE onto the stack
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['E'])
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['D'])
            self.program_counter += 1
        elif opcode == 0xD6:
            # SUB A, n: Subtract immediate value n from A (similar to the previous SUB instruction)
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.sub(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xD7:
            # RST 2: Restart at vector 2
            self.push(self.program_counter + 1)
            self.program_counter = 0x0010
        elif opcode == 0xD8:
            # RET C: Return from subroutine if Carry flag is set
            if self.registers['F'] & 0x10:
                self.program_counter = self.pop()
            else:
                self.program_counter += 1
        elif opcode == 0xD9:
            # RETI: Return from interrupt
            self.program_counter = self.pop()  # Similar to RET
        elif opcode == 0xDA:
            # JP C, nn: Jump to address nn if Carry flag is set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if self.registers['F'] & 0x10:
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xDB:
            self.program_counter += 1
        elif opcode == 0xDC:
            # CALL C, nn: Call subroutine at address nn if Carry flag is set
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            if self.registers['F'] & 0x10:
                self.push(self.program_counter + 3)
                self.program_counter = address
            else:
                self.program_counter += 2
        elif opcode == 0xDD:
            # This is a prefix DD opcode, which handles extended instructions (not covered yet)
            pass  # Placeholder for extended opcodes
        elif opcode == 0xDE:
            # SBC A, n: Subtract immediate value n and carry flag from A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.sbc(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xDF:
            # RST 3: Restart at vector 3
            self.push(self.program_counter + 1)
            self.program_counter = 0x0018
        elif opcode == 0xE0:
            # LD (FF00+n), A: Load A into memory at address (FF00 + n)
            self.program_counter += 1
            offset = self.memory.read_byte(self.program_counter)
            self.memory.write_byte(0xFF00 + offset, self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xE1:
            # POP HL: Pop the top of the stack into register pair HL
            self.registers['L'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.registers['H'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.program_counter += 1
        elif opcode == 0xE2:
            # LD (C), A: Load A into memory at address pointed by C
            address = self.registers['C']
            self.memory.write_byte(address, self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xE3:
            self.program_counter += 1
        elif opcode == 0xE4:
            self.program_counter += 1
        elif opcode == 0xE5:
            # PUSH HL: Push register pair HL onto the stack
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['L'])
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['H'])
            self.program_counter += 1
        elif opcode == 0xE6:
            # AND A, n: Logical AND immediate value n with A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.and_op(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xE7:
            # RST 4: Restart at vector 4
            self.push(self.program_counter + 1)
            self.program_counter = 0x0020
        elif opcode == 0xE8:
            # ADD SP, n: Add immediate value n to stack pointer SP
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.stack_pointer += immediate_value
            self.program_counter += 1
        elif opcode == 0xE9:
            # JP HL: Jump to address in HL
            self.program_counter = self.registers['HL']
        elif opcode == 0xEA:
            # LD (nn), A: Load A into memory at address nn
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            self.memory.write_byte(address, self.registers['A'])
            self.program_counter += 2
        elif opcode == 0xEB:
            self.program_counter += 1
        elif opcode == 0xEC:
            self.program_counter += 1
        elif opcode == 0xED:
            self.program_counter += 1
        elif opcode == 0xEE:
            # XOR A, n: Logical XOR immediate value n with A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.xor_op(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xEF:
            # RST 5: Restart at vector 5
            self.push(self.program_counter + 1)
            self.program_counter = 0x0028
        elif opcode == 0xF0:
            # LD A, (FF00+n): Load A from memory at address (FF00 + n)
            self.program_counter += 1
            offset = self.memory.read_byte(self.program_counter)
            self.registers['A'] = self.memory.read_byte(0xFF00 + offset)
            self.program_counter += 1
        elif opcode == 0xF1:
            # POP AF: Pop the top of the stack into register pair AF
            self.registers['F'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.registers['A'] = self.memory.read_byte(self.stack_pointer)
            self.stack_pointer += 1
            self.program_counter += 1
        elif opcode == 0xF2:
            # LD A, (C): Load A from memory at address pointed by C
            address = self.registers['C']
            self.registers['A'] = self.memory.read_byte(address)
            self.program_counter += 1
        elif opcode == 0xF3:
            # DI: Disable interrupts (not implemented)
            self.program_counter += 1
        elif opcode == 0xF4:
            self.program_counter += 1
        elif opcode == 0xF5:
            # PUSH AF: Push register pair AF onto the stack
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['F'])
            self.stack_pointer -= 1
            self.memory.write_byte(self.stack_pointer, self.registers['A'])
            self.program_counter += 1
        elif opcode == 0xF6:
            # OR A, n: Logical OR immediate value n with A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.or_op(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xF7:
            # RST 6: Restart at vector 6
            self.push(self.program_counter + 1)
            self.program_counter = 0x0030
        elif opcode == 0xF8:
            # LD HL, SP+n: Load HL with SP + immediate value n
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.registers['HL'] = self.stack_pointer + immediate_value
            self.program_counter += 1
        elif opcode == 0xF9:
            # LD SP, HL: Load stack pointer SP with HL
            self.stack_pointer = self.registers['HL']
            self.program_counter += 1
        elif opcode == 0xFA:
            # LD A, (nn): Load A from memory at address nn
            self.program_counter += 1
            address = self.memory.read_word(self.program_counter)
            self.registers['A'] = self.memory.read_byte(address)
            self.program_counter += 2
        elif opcode == 0xFB:
            # EI: Enable interrupts (not implemented)
            self.program_counter += 1
        elif opcode == 0xFC:
            # CALL nn: Call subroutine at address nn
            self.program_counter += 1
        elif opcode == 0xFD:
            self.program_counter += 1
        elif opcode == 0xFE:
            # CP A, n: Compare immediate value n with A
            self.program_counter += 1
            immediate_value = self.memory.read_byte(self.program_counter)
            self.cp(self.registers['A'], immediate_value)
            self.program_counter += 1
        elif opcode == 0xFF:
            # RST 7: Restart at vector 7
            self.push(self.program_counter + 1)
            self.program_counter = 0x0038

        else:
            raise NotImplementedError(f"Unsupported opcode {opcode}")

    def cp(self, param, immediate_value):
        # Compare the value in param with the immediate value
        result = param - immediate_value
        flag_z = 0
        flag_n = 1
        flag_h = 0
        flag_c = 0

        if result == 0:
            flag_z = 1

        if (param & 0xF) < (immediate_value & 0xF):
            flag_h = 1

        if param < immediate_value:
            flag_c = 1
        #self.registers['F'] = (flag_z << 7) | (flag_n << 6) | (flag_h << 5) | (flag_c << 4)
        self.set_flag('Z', flag_z)
        self.set_flag('N', flag_n)
        self.set_flag('H', flag_h)
        self.set_flag('C', flag_c)

    def get_flag(self, flag: str) -> int:
        # switch case for flags - return the corresponding bit in the F register
        if flag == 'Z':
            return (self.registers['F'] & 0x80) >> 7
        elif flag == 'N':
            return (self.registers['F'] & 0x40) >> 6
        elif flag == 'H':
            return (self.registers['F'] & 0x20) >> 5
        elif flag == 'C':
            return (self.registers['F'] & 0x10) >> 4

    def set_flag(self, flag: str, value: int):
        # switch case for flags - set the corresponding bit in the F register
        if flag == 'Z':
            self.registers['F'] = (self.registers['F'] & 0x7F) | (value << 7)
        elif flag == 'N':
            self.registers['F'] = (self.registers['F'] & 0xBF) | (value << 6)
        elif flag == 'H':
            self.registers['F'] = (self.registers['F'] & 0xDF) | (value << 5)
        elif flag == 'C':
            self.registers['F'] = (self.registers['F'] & 0xEF) | (value << 4)


    def or_op(self, param, immediate_value):
        # Perform a logical OR operation between param and the immediate value
        result = param | immediate_value
        if result == 0:
            self.set_flag('Z', result == 1)
        else:
            self.set_flag('Z', 0)
        self.set_flag('N', 0)
        self.set_flag('H', 0)
        self.set_flag('C', 0)
        return result

    def xor_op(self, param, immediate_value):
        # Perform a logical XOR operation between param and the immediate value
        result = param ^ immediate_value
        if result == 0:
            self.set_flag('Z', result == 1)
        else:
            self.set_flag('Z', 0)
        self.set_flag('N', 0)
        self.set_flag('H', 0)
        self.set_flag('C', 0)
        return result

    def and_op(self, param, immediate_value):
        # Perform a logical AND operation between param and the immediate value
        result = param & immediate_value
        if result == 0:
            self.set_flag('Z', result == 1)
        else:
            self.set_flag('Z', 0)
        self.set_flag('N', 0)
        self.set_flag('H', 0)
        self.set_flag('C', 0)
        return result

    def sbc(self, param, immediate_value):
        # Subtract the immediate value and the carry flag from param
        carry = 1 if self.get_flag('C') == 1 else 0
        result = param - immediate_value - carry
        if result == 0:
            self.set_flag('Z', result == 1)
        else:
            self.set_flag('Z', 0)
        self.set_flag('N', True)
        if (param & 0xF) < (immediate_value & 0xF) + carry:
            self.set_flag('H', 1)
        else:
            self.set_flag('H', 0)
        if param < immediate_value + carry:
            self.set_flag('C', 1)
        else:
            self.set_flag('C', 1)
        return result

    def adc(self, param, immediate_value):
        # Add the immediate value and the carry flag to param
        carry = 1 if self.get_flag('C') == 1 else 0
        result = param + immediate_value + carry
        if result == 0:
            self.set_flag('Z', result == 1)
        else:
            self.set_flag('Z', 0)
        self.set_flag('N', 0)
        if (param & 0xF) + (immediate_value & 0xF) + carry > 0xF:
            self.set_flag('H', 1)
        else:
            self.set_flag('H', 0)
        if param + immediate_value + carry > 0xFF:
            self.set_flag('C', 1)
        else:
            self.set_flag('C', 0)
        return result

    def sub(self, param, immediate_value):
        # Subtract the immediate value from param
        result = param - immediate_value
        if result == 0:
            self.set_flag('Z', result == 1)
        else:
            self.set_flag('Z', 0)
        self.set_flag('N', True)
        if (param & 0xF) < (immediate_value & 0xF):
            self.set_flag('H', 1)
        else:
            self.set_flag('H', 0)
        if param < immediate_value:
            self.set_flag('C', 1)
        else:
            self.set_flag('C', 0)
        return result

    def add(self, param, immediate_value):
        # Add the immediate value to param
        result = param + immediate_value
        if result == 0:
            self.set_flag('Z', result == 1)
        else:
            self.set_flag('Z', 0)
        self.set_flag('N', 0)
        if (param & 0xF) + (immediate_value & 0xF) > 0xF:
            self.set_flag('H', 1)
        else:
            self.set_flag('H', 0)
        if param + immediate_value > 0xFF:
            self.set_flag('C', 1)
        else:
            self.set_flag('C', 0)
        return result


    def step(self):
            # Emulate one CPU cycle: fetch, decode, and execute an instruction
            # Fetch the next instruction
            instruction = self.fetch_instruction()
            # Decode and execute the instruction
            self.decode_and_execute(instruction)


            logging.debug("CPU::emulate_cycle")