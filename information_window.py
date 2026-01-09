import krpc
import threading
import time
from datetime import timedelta

WINDOW_WIDTH = 250
WINDOW_HEIGHT = 140  

TEXT_SIZE = 14
TEXT_START_POS = (-40, WINDOW_HEIGHT / 2 - 20)

class InformationWindow:
   def __init__(self, liftoff_time:int):
      self.liftoff_time = liftoff_time

      self.client = krpc.connect(name='Flight Info')
   
      self.canvas = self.client.ui.stock_canvas
      screen_size = self.canvas.rect_transform.size
      self.panel = self.canvas.add_panel()

      rect = self.panel.rect_transform
      rect.size = (WINDOW_WIDTH, WINDOW_HEIGHT)
      rect.position = (screen_size[0]/2 - rect.size[0]/2 - 50, screen_size[1]/2 - rect.size[1]/2 - 10)

      self.flight_time_text = self.panel.add_text('')
      self.flight_time_text.visible = True
      self.flight_time_text.rect_transform.position = TEXT_START_POS
      self.flight_time_text.color = (1, 0, 0)
      self.flight_time_text.size = TEXT_SIZE

      self.upfg_status_text = self.panel.add_text('UPFG: Inactive')
      self.upfg_status_text.visible = True
      self.upfg_status_text.rect_transform.position = (TEXT_START_POS[0], TEXT_START_POS[1] - (TEXT_SIZE + 6))
      self.upfg_status_text.color = (0, 0, 1)
      self.upfg_status_text.size = TEXT_SIZE

      self.target_speed_text = self.panel.add_text('Target Speed: NaN')
      self.target_speed_text.visible = True
      self.target_speed_text.rect_transform.position = (TEXT_START_POS[0], TEXT_START_POS[1] - 2*(TEXT_SIZE + 6))
      self.target_speed_text.color = (0, 0, 1)
      self.target_speed_text.size = TEXT_SIZE

      self.orbit_speed_text = self.panel.add_text('Orbit Speed: NaN')
      self.orbit_speed_text.visible = True
      self.orbit_speed_text.rect_transform.position = (TEXT_START_POS[0], TEXT_START_POS[1] - 3*(TEXT_SIZE + 6))
      self.orbit_speed_text.color = (0, 0, 1)
      self.orbit_speed_text.size = TEXT_SIZE

      self.tgo_text = self.panel.add_text('TGO: NaN')
      self.tgo_text.visible = True
      self.tgo_text.rect_transform.position = (TEXT_START_POS[0], TEXT_START_POS[1] - 4*(TEXT_SIZE + 6))
      self.tgo_text.color = (0, 0, 1)
      self.tgo_text.size = TEXT_SIZE

      self.vgo_text = self.panel.add_text('VGO: NaN')
      self.vgo_text.visible = True
      self.vgo_text.rect_transform.position = (TEXT_START_POS[0], TEXT_START_POS[1] - 5*(TEXT_SIZE + 6))
      self.vgo_text.color = (0, 0, 1)
      self.vgo_text.size = TEXT_SIZE

      self.running = True  
      self.display_thread = threading.Thread(target=self.display_time)
      self.display_thread.daemon = True
      self.display_thread.start()

   def __del__(self):
      self.running = False    

   @property
   def upfg_status(self) -> str:
      return self.upfg_status_text.content

   @upfg_status.setter
   def upfg_status(self, status:str):
      self.upfg_status_text.content = status

   @property
   def target_speed(self) -> str:
      return self.target_speed_text.content
   
   @target_speed.setter
   def target_speed(self, speed:str):
      self.target_speed_text.content = speed

   @property
   def orbit_speed(self) -> str:
      return self.orbit_speed_text.content
   
   @orbit_speed.setter
   def orbit_speed(self, speed:str):
      self.orbit_speed_text.content = speed

   @property
   def tgo(self) -> str:
      return self.tgo_text.content
   
   @tgo.setter
   def tgo(self, tgo:str):
      self.tgo_text.content = tgo

   @property
   def vgo(self) -> str:
      return self.vgo_text.content
   
   @vgo.setter
   def vgo(self, vgo:str):
      self.vgo_text.content = vgo

   def seconds_to_hhmmss(self, seconds):
      td = timedelta(seconds=seconds)
      return str(td).replace("days, ", "").replace("day, ", "")   
   
   def display_time(self):
      while self.running:
         current_time = int(round(self.client.space_center.ut))
         ft = current_time - self.liftoff_time
         sign = True if ft < 0 else False
         ft = abs(ft)

         self.flight_time_text.content = 'T {}{}'.format('-' if sign else '+', self.seconds_to_hhmmss(int(ft)))  
         time.sleep(0.1)

   def clear_info(self, info:str):
      self.target_speed_text.remove()
      self.orbit_speed_text.remove()
      self.tgo_text.remove()
      self.vgo_text.remove() 
      self.upfg_status_text.content = info 

