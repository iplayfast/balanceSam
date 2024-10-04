import pygame
import time

# Screen dimensions
screen_width = 800
screen_height = 600
LEFTX = 0
LEFTY = 1
RIGHTX = 3
RIGHTY = 4
TRIGL = 2
TRIGR = 5

BTN_CROSS = 0
BTN_CIRCLE = 1
BTN_SQUARE = 3
BTN_TRI = 2

BTN_L1 = 4
BTN_R1 = 5
BTN_SHARE = 8
BTN_OPTIONS = 9

BTN_LTP = 6 #Left Touchpad button
BTN_RTP = 7 #Right Touchpad button

BTN_PS = 10
BTN_LSB = 11 #LEFT STICK BUTTON
BTN_RSB = 12 # Right stick button


# Initialize pygame
pygame.init()
pygame.joystick.init()

# Initialize the screen
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("PS4 Controller Input")

# Check if a controller is connected
if pygame.joystick.get_count() == 0:
    print("No controller found!")
    exit()

# Connect to the first controller
controller = pygame.joystick.Joystick(0)
controller.init()

print("Connected to controller: {}".format(controller.get_name()))
print("Number of axes:", controller.get_numaxes())
print("Number of buttons:", controller.get_numbuttons())
print("Number of hats:", controller.get_numhats())

# Colors for drawing
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
GRAY = (100, 100, 100)
LIGHT_GRAY = (200, 200, 200)

# Button and stick positions relative to the controller layout
"""
button_positions = {
    0: (600, 350),  # Cross
    1: (650, 300),  # Circle
    2: (550, 300),  # Square
    3: (600, 250),  # Triangle
    4: (250, 180),  # L1
    5: (550, 180),  # R1
    6: (330, 250),  # Share
    7: (470, 250),  # Options
    8: (300, 350),  # L3 (Left Stick Button)
    9: (600, 300),  # Right Touchpad Button
    10: (400, 300), # PS Button
    11: (200, 300), # Left Touchpad Button
    12: (500, 350),  # R3 (Right Stick Button)
}
"""
# Stick positions (axes)
axis_positions = {
    0: (300, 350),  # Left stick
    2: (500, 350)   # Right stick
}

# Trigger positions
trigger_positions = {
    4: (250, 130),  # L2 (trigger)
    5: (550, 130)   # R2 (trigger)
}

# D-Pad (Hat) position
hat_position = (180, 300)  # D-Pad

# Helper functions to draw shapes
def draw_button(screen, position, pressed, radius=15):
    color = GREEN if pressed else RED
    pygame.draw.circle(screen, color, position, radius)
    #print(f"{position}")

def draw_symbol_button(screen, position, symbol, pressed):
    color = GREEN if pressed else LIGHT_GRAY
    pygame.draw.circle(screen, color, position, 20)
    font = pygame.font.SysFont(None, 30)
    text = font.render(symbol, True, BLACK)
    text_rect = text.get_rect(center=position)
    screen.blit(text, text_rect)

def draw_axis(screen, position, value_x, value_y):
    pygame.draw.circle(screen, GRAY, position, 30, 2)  # Outer circle
    x = int(position[0] + value_x * 20)
    y = int(position[1] + value_y * 20)
    pygame.draw.circle(screen, BLUE, (x, y), 25)  # Stick position
    return x,y

def draw_hat(screen, position, hat_x, hat_y):
    # Draw D-pad background
    pygame.draw.rect(screen, GRAY, (position[0] - 30, position[1] - 30, 60, 60))
    
    # Draw D-pad buttons
    directions = [('up', 0, -1), ('right', 1, 0), ('down', 0, 1), ('left', -1, 0)]
    for direction, dx, dy in directions:
        button_pos = (position[0] + dx * 20, position[1] + dy * 20)
        color = YELLOW if (hat_x == dx and hat_y == -dy) else LIGHT_GRAY
        pygame.draw.rect(screen, color, (button_pos[0] - 10, button_pos[1] - 10, 20, 20))

def draw_oval_button(screen, position, pressed):
    color = GREEN if pressed else LIGHT_GRAY
    pygame.draw.ellipse(screen, color, (position[0] - 10, position[1] - 25, 20, 50))  # Vertical oval

def draw_trigger(screen, position, value):
    pygame.draw.rect(screen, GRAY, (position[0] - 25, position[1] - 40, 50, 80), 2)
    height = int(value * 70)
    pygame.draw.rect(screen, RED, (position[0] - 23, position[1] + 38 - height, 46, height))
    #print(f"{height}")

def draw_rounded_rect(surface, rect, color, corner_radius):
    ''' Draw a rectangle with rounded corners '''
    pygame.draw.rect(surface, color, rect)
    pygame.draw.circle(surface, color, (rect.left + corner_radius, rect.top + corner_radius), corner_radius)
    pygame.draw.circle(surface, color, (rect.right - corner_radius - 1, rect.top + corner_radius), corner_radius)
    pygame.draw.circle(surface, color, (rect.left + corner_radius, rect.bottom - corner_radius - 1), corner_radius)
    pygame.draw.circle(surface, color, (rect.right - corner_radius - 1, rect.bottom - corner_radius - 1), corner_radius)

def draw_controller_outline():
    # Main body
    draw_rounded_rect(screen, pygame.Rect(150, 200, 500, 250), WHITE, 50)
    pygame.draw.rect(screen, BLACK, (155, 205, 490, 240))  # Inner black fill
    
    # Grips
    pygame.draw.ellipse(screen, WHITE, (130, 250, 100, 250), 5)
    pygame.draw.ellipse(screen, WHITE, (570, 250, 100, 250), 5)
    
    # Stick areas
    pygame.draw.circle(screen, WHITE, (300, 350), 50, 5)
    pygame.draw.circle(screen, WHITE, (500, 350), 50, 5)
    
    # Trigger outlines
    pygame.draw.rect(screen, WHITE, (220, 140, 60, 100), 5)
    pygame.draw.rect(screen, WHITE, (520, 140, 60, 100), 5)

try:
    while True:
        screen.fill(BLACK)
        draw_controller_outline()
        pygame.event.pump()
        #for i in range(0,5):
        #    v = controller.get_axis(i)
        #    print(f"number {i} value {v}")
        # Draw symbol buttons
        #symbols = {0: 'X', 1: 'O', 2: '□', 3: 'Δ'}
        #for i, symbol in symbols.items():
        #    pressed = controller.get_button(i)
        #    draw_symbol_button(screen, button_positions[i], symbol, pressed)

        draw_symbol_button(screen,(620,350),'X',controller.get_button(BTN_CROSS))
        draw_symbol_button(screen,(670,300),'O',controller.get_button(BTN_CIRCLE))
        draw_symbol_button(screen,(570,300),'□',controller.get_button(BTN_SQUARE))
        draw_symbol_button(screen,(620,250),'Δ',controller.get_button(BTN_TRI))

        # Draw analog sticks
        left_x, left_y = controller.get_axis(LEFTX), controller.get_axis(LEFTY)
        right_x, right_y = controller.get_axis(RIGHTX), controller.get_axis(RIGHTY)  # Changed to axis 5 for right stick vertical
        left_x,left_y = draw_axis(screen, axis_positions[0], left_x, left_y)
        right_x,right_y = draw_axis(screen, axis_positions[2], right_x, right_y)

        # Draw triggers
        #l2 = (controller.get_axis(TRIGL) + 1) / 2  # L2 uses axis 3
        #r2 = (controller.get_axis(TRIGR) + 1) / 2  # R2 uses axis 4
        l2 = controller.get_axis(TRIGL)
        r2 = controller.get_axis(TRIGR)
        draw_trigger(screen, trigger_positions[4], l2)
        draw_trigger(screen, trigger_positions[5], r2)

        # Draw D-Pad (hat)
        hat = controller.get_hat(0)
        #print(hat)
        draw_hat(screen, hat_position, hat[0], hat[1])

        # Draw other buttons
        #for i in range(0, 13):  # Buttons 4 to 12
        #    v = controller.get_button(i)
        #    print(f"button {i} value {v}")
        #    if i in button_positions:
        #        draw_button(screen, button_positions[i], controller.get_button(i))
        #draw_button(screen,button_positions[BTN_L1],controller.get_button(BTN_L1))
        draw_button(screen,(250,180),controller.get_button(BTN_L1))
        draw_button(screen,(550,180),controller.get_button(BTN_R1))

        draw_button(screen,(left_x,left_y),controller.get_button(BTN_LSB))
        draw_button(screen,(right_x,right_y),controller.get_button(BTN_RSB))
        #
        draw_button(screen,(600,180),controller.get_button(BTN_RTP))
        draw_button(screen,(200,180),controller.get_button(BTN_LTP))

        draw_button(screen,(400,400),controller.get_button(BTN_PS))
        # Draw Share and Options buttons
        draw_oval_button(screen, (330,250), controller.get_button(BTN_SHARE))  # Share
        draw_oval_button(screen, (470,250), controller.get_button(BTN_OPTIONS))  # Options

        pygame.display.flip()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    pygame.quit()
