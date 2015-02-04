def mm(in_l):
	st = ""
	for i in in_l:
		st += " OR \"FID\" = " + str(i)
	return st
