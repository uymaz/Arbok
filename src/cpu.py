import logging

class CPU:
    def __init__(self, memory):
        # Initialize CPU registers, flags, and other state
        self.memory = memory
        self.program_counter = 0x100  # Start execution at ROM address 0x100
        self.registers = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'H': 0, 'L': 0}
        
    
    def fetch_instruction(self):
        # Fetch the next instruction from memory
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