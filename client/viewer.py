import pygame as pg


class Viewer:
  def __init__(self, ego_camera_size, bark_screen_size, FPS=None):
    self.ego_camera_size = ego_camera_size
    self.bark_screen_size = bark_screen_size
    self.FPS = FPS
    self.clock = pg.time.Clock()

    pg.init()

    try:
      self.screen = pg.display.set_mode((self.ego_camera_size[0]+self.bark_screen_size[0],
                                         max(self.ego_camera_size[1], self.bark_screen_size[1])), pg.DOUBLEBUF)
      pg.display.set_caption("Carla Interface")

    except pg.error:
      self.screen = None
      print("No available video device")

  def tick(self):
    if self.FPS:
      self.clock.tick(self.FPS)
    else:
      self.clock.tick()

  def update(self, surface, rect):
    assert len(rect) == 4
    self.screen.blit(surface, rect)
    pg.display.update(rect)
