import krpc
import threading
import time
from datetime import timedelta

class FlightTimeWindow:
   def __init__(self, liftoff_time:int):
      self.liftoff_time = liftoff_time

      self.client = krpc.connect(name='Flight Time')
   
      self.canvas = self.client.ui.stock_canvas
      screen_size = self.canvas.rect_transform.size
      self.panel = self.canvas.add_panel()

      rect = self.panel.rect_transform
      rect.size = (150, 50)
      rect.position = (-screen_size[0]/2 + rect.size[0]/2 + 20, screen_size[1]/2 - rect.size[1]/2 - 50)

      self.text = self.panel.add_text('')
      self.text.visible = True
      self.text.rect_transform.position = (20, 0)
      self.text.color = (1, 0, 0)
      self.text.size = 25

      self.running = True  
      self.display_thread = threading.Thread(target=self.display_time)
      self.display_thread.daemon = True
      self.display_thread.start()

   def __del__(self):
      self.running = False    

   def seconds_to_hhmmss(self, seconds):
      td = timedelta(seconds=seconds)
      return str(td).replace("days, ", "").replace("day, ", "")   
   
   def display_time(self):
      while self.running:
         current_time = int(round(self.client.space_center.ut))
         ft = current_time - self.liftoff_time
         sign = True if ft < 0 else False
         ft = abs(ft)

         self.text.content = 'T {}{}'.format('-' if sign else '+', self.seconds_to_hhmmss(int(ft)))  
         time.sleep(0.1)

