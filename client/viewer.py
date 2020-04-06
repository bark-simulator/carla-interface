import pygame as pg
import math
import logging


class CosimulationViewer:
    def __init__(self, bark_screen_size,
                 FPS=0, num_cameras=0):
        self.bark_screen_size = bark_screen_size
        self.FPS = FPS
        self.clock = pg.time.Clock()

        self.num_cameras = num_cameras
        self.cameras_window_position = []
        self.cameras_window_size = None

        pg.init()

        try:
            if self.num_cameras != 0:
                self.screen = pg.display.set_mode((self.bark_screen_size[0] * 2,
                                                   self.bark_screen_size[1]), pg.HWSURFACE | pg.DOUBLEBUF)

                sq = math.ceil(math.sqrt(self.num_cameras))
                w = self.bark_screen_size[0] / sq
                h = self.bark_screen_size[1] / sq
                self.windwows_size = (int(w), int(h))

                i = 0
                for i in range(sq**2):
                    if i > self.num_cameras:
                        return
                    else:
                        self.cameras_window_position.append(
                            (int(self.bark_screen_size[0] + (i % sq) * w), int(i // sq) * h))
            else:
                self.screen = pg.display.set_mode(
                    self.bark_screen_size,
                    pg.HWSURFACE | pg.DOUBLEBUF)

            pg.display.set_caption("Carla Interface")

        except pg.error:
            self.screen = None
            logging.exception("No available video device")

    def tick(self):
        self.clock.tick(self.FPS)

    def update_cameras(self, surfaces_dict, agents_ids=None):
        if self.num_cameras != 0:
            surfaces = surfaces_dict.values() if agents_ids is None else [
                surfaces_dict[k] for k in agents_ids]
            for s, p in zip(surfaces, self.cameras_window_position):
                if s is not None:
                    s = pg.transform.scale(s, self.windwows_size)
                    self.screen.blit(s, p)

    def update_bark(self, surface, position=(0, 0)):
        self.screen.blit(surface, position + self.bark_screen_size)

    def show(self):
        pg.display.flip()
