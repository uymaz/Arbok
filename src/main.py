import pygame
import time
import numpy

import cpu
import memory

GB_SCREEN_WIDTH = 160
GB_SCREEN_HEIGHT = 144
SCALE = 4

WINDOW_WIDTH = GB_SCREEN_WIDTH * SCALE
WINDOW_HEIGHT = GB_SCREEN_HEIGHT * SCALE

GB_CPU_FREQ = 4194304 #4.193mhz
FRAME_RATE = 60

TIME_PER_FRAME = 1 / FRAME_RATE
CYCLES_PER_FRAME = GB_CPU_FREQ / FRAME_RATE

pygame.init()

window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Arbok")

framebuffer = numpy.zeros((GB_SCREEN_HEIGHT, GB_SCREEN_WIDTH, 3), dtype = numpy.uint8)

# Initialize emulator components (CPU, memory, display, input, cartridge)

def render_framebuffer():
     surface = pygame.surfarray.make_surface(numpy.kron(framebuffer, numpy.ones((SCALE, SCALE, 1))))
     window.blit(surface, (0,0))

def main():
    running = True

    last_frame_time = time.perf_counter()

    #_memory = memory.Memory()
    #_cpu = cpu.CPU(_memory)

    while running:
        start_time = time.perf_counter()

        for event in pygame.event.get():
             if event.type == pygame.QUIT:
                  running = False

        #rom shit

        #update framebuffer afterwards

        #draw if its time for it - ensure that its whatever fps the user wants
        if time.perf_counter() - last_frame_time >= TIME_PER_FRAME:
             render_framebuffer()
             last_frame_time = time.perf_counter()

        elapsed_time = time.perf_counter() - start_time

        time_to_sleep = TIME_PER_FRAME - elapsed_time

        if time_to_sleep > 0:
             time.sleep(time_to_sleep)

def load_rom(self, rom_file):
        # Load a Game Boy ROM into memory
        print("test load rom")

if __name__ == "__main__":
     main()

pygame.quit()