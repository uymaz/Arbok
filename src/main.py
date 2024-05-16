import sdl2
import sdl2.ext

sdl2.ext.init()

window = sdl2.ext.Window("Arbok", size=(640, 480))
window.show()

renderer = sdl2.ext.Renderer(window)

running = True
while running:
    for event in sdl2.ext.get_events():
        if event.type == sdl2.SDL_KEYDOWN:
            running = False
    
    renderer.clear()
    
    renderer.present()

sdl2.ext.quit()
