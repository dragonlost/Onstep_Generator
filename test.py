# test.py

import time
import os

def test_config(mount_type, board_type, max_rate, pec, auto_sid, worm1, gear1, stepper1, micro1, slew1, driver1, worm2, gear2, stepper2, micro2, slew2, driver2,path=os.getcwd()+'/config_'+time.strftime("%d_%Y_%H_%M_%S", time.localtime())+'.txt'):


	print("mount_type", mount_type)
	print("board_type", board_type)
	print("max_rate", max_rate)
	print("pec", pec)
	print("auto_sid", auto_sid)
	print("worm1", worm1)
	print("gear1", gear1)
	print("stepper1", stepper1)

	print( "micro1", micro1)
	print("slew1", slew1)
	print("driver1", driver1)
	print("worm2", worm2)
	print("gear2", gear2)
	print("stepper2", stepper2)
	print("micro2", micro2)
	print("slew2", slew2)
	print("driver2", driver2)
	print(path)

