import pygame as pg
import math

class Viewer:
  def __init__(self, num_camera, bark_screen_size, FPS=None):
    self.bark_screen_size = bark_screen_size
    self.FPS = FPS
    self.clock = pg.time.Clock()

    pg.init()

    try:
      self.screen = pg.display.set_mode((self.bark_screen_size[0]*2,
                                         self.bark_screen_size[1]), pg.HWSURFACE | pg.DOUBLEBUF)
      pg.display.set_caption("Carla Interface")

    except pg.error:
      self.screen = None
      print("No available video device")

    self.windows_position=[]

    sq=math.ceil(math.sqrt(num_camera))
    w=self.bark_screen_size[0]/sq
    h=self.bark_screen_size[1]/sq

    self.windwows_size=(int(w),int(h))

    i=0
    for i in range(sq**2):
        if i > num_camera:
          return
        else:
          self.windows_position.append((int(self.bark_screen_size[0]+(i%sq)*w),int(i//sq)*h,
          int(self.bark_screen_size[0]+(i%sq+1)*w),int(i//sq+1)*h))
          

  def tick(self):
    if self.FPS:
      self.clock.tick(self.FPS)
    else:
      self.clock.tick()

  def update_cameras(self, surfaces_dict,agents_id=None):
    surfaces=surfaces_dict.values() if agents_id is None else [surfaces_dict[k] for k in agents_id]
    for s,p in zip(surfaces,self.windows_position):
      if s is not None:
        s = pg.transform.scale(s, self.windwows_size)
        self.screen.blit(s, p)

  def update(self, surface, position,size):
    # surface = pg.transform.scale(surface, size)
    self.screen.blit(surface, position)
    # pg.display.update(position+size)

  def flip(self):
    pg.display.flip()