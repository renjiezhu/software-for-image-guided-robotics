import multiprocessing as mp
import spacenav
import atexit
import sys

#Have to use multiprocessing library, tried it with threads does not work since the spacenav library has 
#cPython which uses global interpreter lock. Use multiprocessing array to pass data back and forth
#index the data container like c

class MouseClient:
	def __init__(self):
		self.running = mp.Value('i',0) #initialize running value to 0
		self.event = mp.Array('i',6) #create a 6 dimensional array filled with 0's

	def mouseThreadFunction(self, shared_event, running):
		try:
			# open the connection
			print("Opening connection to SpaceNav driver ...")
			spacenav.open()
			print("... connection established.")
			# register the close function if no exception was raised
			atexit.register(spacenav.close)
		except spacenav.ConnectionError:
			# give some user advice if the connection failed 
			print("No connection to the SpaceNav driver. Is spacenavd running?")
			sys.exit(-1)

		# loop over space navigator events
		scaling_value = 1.0
		while running.value:
			# wait for next event
			event = spacenav.wait()

			if type(event) is spacenav.ButtonEvent and event.pressed == 0:
				print("button pressed {}".format(event.button))

				if event.button == 1:
					scaling_value = scaling_value + 0.1
					if scaling_value > 2:
						scaling_value = 2
					print("scaling up by .1: {}".format(scaling_value))

				elif event.button == 0:
					scaling_value = scaling_value - 0.1
					if scaling_value < 0.1:
						scaling_value = 0.1
					print("scaling down by .1: {}".format(scaling_value))

			elif type(event) is spacenav.MotionEvent:
				#Update shared memory with mouse information
				shared_event[0] = int(event.x * scaling_value)
				shared_event[1] = int(event.y * scaling_value)
				shared_event[2] = int(event.z * scaling_value)
				shared_event[3] = int(event.rx * scaling_value)
				shared_event[4] = int(event.ry * scaling_value)
				shared_event[5] = int(event.rz * scaling_value)

	def run(self):
		self.running.value = 1
		mouseThread = mp.Process(target = self.mouseThreadFunction, args=(self.event,self.running))
		mouseThread.daemon = True #autimatically kills the thread when the main thread is finished
		mouseThread.start()

	def stop(self):
		#ends mouse process
		self.running.value = 0
