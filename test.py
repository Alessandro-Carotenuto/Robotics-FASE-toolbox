from robot import Robot

R1=Robot("P2R")
print(R1.jointsequence)
print(R1.jointlist[2].getDHTransform())