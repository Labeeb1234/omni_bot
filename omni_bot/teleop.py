import pygame
import sys

class Teleop:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((400, 200))
        pygame.display.set_caption("Teleop Control")
        self.font = pygame.font.Font(None, 36)
        self.text = "Press arrow keys (ESC to exit)"
        self.running = True

    def handle_event(self, event):
        """Handle keyboard input."""
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                self.text = "Up arrow pressed"
            elif event.key == pygame.K_DOWN:
                self.text = "Down arrow pressed"
            elif event.key == pygame.K_LEFT:
                self.text = "Left arrow pressed"
            elif event.key == pygame.K_RIGHT:
                self.text = "Right arrow pressed"
            elif event.key == pygame.K_ESCAPE:
                self.running = False

    def update_screen(self):
        """Update display with the latest text."""
        self.screen.fill((255, 255, 255))
        text_surface = self.font.render(self.text, True, (0, 0, 0))
        self.screen.blit(text_surface, (50, 80))
        pygame.display.flip()

    def run(self):
        """Main loop."""
        while self.running:
            for event in pygame.event.get():
                self.handle_event(event)
            self.update_screen()
        
        pygame.quit()
        sys.exit()