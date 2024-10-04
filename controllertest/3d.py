import pygame
controller = pygame.joystick.Joystick(0)
controller.init()
axis = {}
button = {}
# these are the identifiers for the PS4's accelerometers
AXIS_X = 3
AXIS_Y = 4

# variables we'll store the rotations in, initialised to zero
rot_x = 0.0
rot_y = 0.0

# main loop
while True:

# copy rot_x/rot_y into axis[] in case we don't read any
    axis[AXIS_X] = rot_x
    axis[AXIS_Y] = rot_y

# retrieve any events ...
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            axis[event.axis] = round(event.value,2)
        elif event.type == pygame.JOYBUTTONDOWN:
            button[event.button] = True
        elif event.type == pygame.JOYBUTTONUP:
            button[event.button] = False

    rot_x = axis[AXIS_X]
    rot_y = axis[AXIS_Y]            


glPushMatrix()   
glRotatef(roty_scale * rot_y, 1, 0, 0)
glRotatef(rotx_scale * rot_x, 0, 0, 1)
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
glBegin(GL_LINES)
for edge in edges:
    for vertex in edge:
        glColor3fv((1,1,1))
        glVertex3fv(vertices[vertex])
glEnd()
glPopMatrix()
