import sys
sys.path.append('..')

from mouse3d import MouseClient
import time
import spacenav
import atexit


mouse = MouseClient()
mouse.run()

#for i in range(100):
while True:
	if mouse.event == None:
		print("nothing!")
	else:
		print(mouse.event[:])
	time.sleep(0.1)

print("stopping mouse")
mouse.stop()

time.sleep(3)

