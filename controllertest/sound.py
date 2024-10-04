import pygame

pygame.init()
m = pygame.mixer
m.init()
nc = m.get_num_channels()
print(f"num channels {nc}")
#for i in range(0,nc):


tom = pygame.mixer.Sound('tom.wav')
print(f"playback volume {tom.get_volume()}")

tom.play()

