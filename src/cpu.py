import logging
from dataclasses import dataclass
from typing import Literal, Optional
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


class CPU:
    def __init__(self, memory):
        # Initialize CPU registers, flags, and other state
        self.memory = memory
        self.program_counter = 0x100  # Start execution at ROM address 0x100
        self.registers = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'H': 0, 'L': 0}
        
    
    def fetch_instruction(self):
        logging.debug("CPU::fetch_instruction called")
        return self.memory.read_byte(self.program_counter)
    
    #used for BC and HL registers, mainly
    #Returns the value of (register_a << 8) + register_b
    def lookup_16bit_register(self, register_a, register_b):
        logging.debug("CPU::lookup_16bit_register called")
        return (register_a << 8) + register_b

    def decode_and_execute(self, instruction):
        # Decode and execute the instruction
        logging.debug("CPU::decode_and_execute")
        instruction = self.fetch_instruction()

        opcode = instruction & 0xFF  # Extract the opcode from the instruction
        if opcode == 0x00:
            # NOP
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
        elif opcode == 0x04:
            address = self.registers['B']
        elif opcode == 0x3E:
            # LD A, n: Load 8-bit immediate value into register A
            immediate_value = self.memory.read_byte(self.program_counter)
            self.program_counter += 1
            self.registers['A'] = immediate_value
        elif opcode == 0xAF:
            # XOR A: Bitwise XOR register A with itself (A ^ A)
            self.registers['A'] ^= self.registers['A']
            self.program_counter += 1

        else:
            raise NotImplementedError(f"Unsupported opcode {opcode}")
    
    def emulate_cycle(self):
        # Emulate one CPU cycle: fetch, decode, and execute an instruction
        logging.debug("CPU::emulate_cycle")