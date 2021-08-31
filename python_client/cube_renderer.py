import pygame
import numpy as np
from math import *
import itertools

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
WIDTH, HEIGHT = 800, 600

RED = (255, 0, 0)
GREEN = (0,255,0)
BLUE = (0,0,255)
YELLOW = (0,255,255)

BOX_WIDTH = 1
BOX_HEIGHT = 0.5
BOX_LENGTH = 3

# render a cube in pygame
# takes in a rotation matrix and renders appropriate orientation
# Source: https://www.youtube.com/watch?v=qw0oY6Ld-L0
class CubeRenderer:
    def __init__(self):
        pygame.display.set_caption("3D projection in pygame!")
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.scale = 50
        self.circle_pos = [WIDTH/2, HEIGHT/2]  # x, y
        self.rotation_matrix = np.eye(3)

        # create our cube representing our handheld sensor suite
        self.points = self.create_cube(BOX_LENGTH, BOX_HEIGHT, BOX_WIDTH)
        self.projected_points = [
            [n, n] for n in range(len(self.points))
        ]

        # we are viewing the cube from the z-y plane
        self.projection_matrix = np.matrix([
            [0, 1, 0],
            [0, 0, 1]
        ])

        # self.corner_colors = [RED,RED,YELLOW,YELLOW,BLUE,BLUE,GREEN,GREEN]
        self.corner_colors = list(itertools.product([50,170], repeat=3))

        # we can disable this flag to exit the async game loop
        self.is_running = True
    
    # render an edge
    def connect_points(self, i, j, points):
        pygame.draw.line(
            self.screen, BLACK, (points[i][0], points[i][1]), (points[j][0], points[j][1]))

    # draw the cube with consideration for the x-order
    # we are viewing the cube from the z-y plane
    def draw_cube(self):
        # render edges (any order)
        for p in range(4):
            self.connect_points(p,      (p+1) % 4,          self.projected_points)
            self.connect_points(p+4,    ((p+1) % 4) + 4,    self.projected_points)
            self.connect_points(p,      (p+4),              self.projected_points)

        # render corners in x-order
        # +x is into the screen, -x is out of the screen
        # we treat the x-value of the rotated point as our z-order value
        z_values = []
        for i, point in enumerate(self.points):
            rotated2d = np.dot(self.rotation_matrix, point.reshape((3, 1)))
            projected2d = np.dot(self.projection_matrix, rotated2d)
            zmat = np.matrix([1,0,0])
            z = -np.dot(zmat, rotated2d)[0,0]
            z = (z/BOX_LENGTH + 3) * 4
            z_values.append(z)

        z_values = np.array(z_values) 
        idx = np.argsort(z_values)

        # render the corners in order, so corners closer to the
        # camera are rendered at the top
        for i in idx:
            point = self.points[i]
            color = self.corner_colors[i]
            z = z_values[i]
            z = np.ceil(z)

            rotated2d = np.dot(self.rotation_matrix, point.reshape((3, 1)))
            projected2d = np.dot(self.projection_matrix, rotated2d)

            x = int(projected2d[0][0] * self.scale) + self.circle_pos[0]
            y = int(projected2d[1][0] * self.scale) + self.circle_pos[1]

            self.projected_points[i] = [x, y]
            pygame.draw.circle(self.screen, color, (x, y), z)

    # create vertices for our cube
    def create_cube(self, length, width, height):
        points = []
        points.append(np.matrix([-length, -height,  width]))
        points.append(np.matrix([ length, -height,  width]))
        points.append(np.matrix([ length,  height,  width]))
        points.append(np.matrix([-length,  height,  width]))
        points.append(np.matrix([-length, -height, -width]))
        points.append(np.matrix([ length, -height, -width]))
        points.append(np.matrix([ length,  height, -width]))
        points.append(np.matrix([-length,  height, -width]))
        return points

    # asyncio pygame loop 
    async def run(self):
        import asyncio

        while self.is_running:
            await asyncio.sleep(0.01)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    self.is_running = False
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        pygame.quit()
                        self.is_running = False
                        return

            self.screen.fill(WHITE)
            self.draw_cube()
            pygame.display.update()
        
        pygame.quit()
        
