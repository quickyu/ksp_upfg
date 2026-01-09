import sys 
sys.path.append("..") 

import os
import unittest
import random
import csv
import tempfile
import string

import utilities

def random_string(length=8):
   chars = string.ascii_letters + string.digits
   return ''.join(random.choice(chars) for _ in range(length))

class TestReadControlFile(unittest.TestCase):
   def __init__(self, methodName="runTest"):
      super(TestReadControlFile, self).__init__(methodName)

   def test_read_control_file_invalid_format(self):
      with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
         f.write("invalid,format\n")
         temp_file = f.name
      
      try:
         with self.assertRaises(Exception):
            utilities.read_control_file(temp_file)
      finally:
         os.unlink(temp_file) 

   def test_read_guidance_data(self):   
      test_data = []
      for _ in range(13): 
         test_data.append([random.random() for _ in range(100)])

      str_row = [random_string() for _ in range(100)]
      test_data.append(str_row)
      str_row = [random_string() for _ in range(100)]
      test_data.append(str_row)

      with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False, newline='') as f:
         writer = csv.writer(f)
         writer.writerows(test_data) 
         temp_file = f.name

      try:
         data = utilities.read_guidance_file(temp_file)   
         self.assertEqual(data, test_data)
      finally:
         os.unlink(temp_file)

if __name__ == '__main__':
   unittest.main()   

