#!/usr/bin/env python
#Contains Useful Misc Code
import sys
import Task1 as T1
import numpy as np
class color:
	PURPLE = '\033[95m'
	CYAN = '\033[96m'
	DARKCYAN = '\033[36m'
	BLUE = '\033[94m'
	GREEN = '\033[92m'
	YELLOW = '\033[93m'
	RED = '\033[91m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'
	END = '\033[0m'

def PubCheck( verbose ): #Publishes True if Subs are fine
		PubList = [[T1.hC.headTrajectoryPublisher, "Head Publisher"], [T1.aC.hand_publisher, "Hand + Arm Publisher"], [T1.aC.armTrajectoryPublisher, "Arm (Old) Publisher"], [T1.pC.PelvisHeightTrajectoryPublisher, "Pelvis Publisher"], [T1.nC.neck_publisher, "Neck Publisher"]]
		PubStatus = []
		PubDead = []
		for x in PubList:
			PubStatus.append(x[0].get_num_connections())
			if x[0].get_num_connections() == 0:
				PubDead.append(x[1])
		if verbose:
			if 0 in PubStatus:
				lost = np.size(PubStatus)-np.sum(PubStatus)
				if lost == np.size(PubStatus):
					lost = "All"
				print color.BOLD + color.RED + "Lost {0} Subscribers".format(lost) + color.END
				for x in PubDead:
					print x
				print "----------------"
				return False
			else:
				return True
		else:
			if 0 in PubStatus: return False
			else: return True

"""
#USE THIS TO PUBLISH INTO TXT
print "PUBLISHING"
f = open("thetaValues.txt", "a")
f.write(str(bla) + "\n")
f.close()
"""
