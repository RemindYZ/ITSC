# pns=psn=1/4 pew=pwe=1/8 plt=1/4 prt=1/4
import numpy as np
import matplotlib.pyplot as plt
import xml.dom.minidom

ACT_queue = "log/ACT_Queue.npy"
ACT_Queue_Length=np.load(ACT_queue)
n_road,n_lane,step = ACT_Queue_Length.shape
X=np.arange(0,step,1)
# plt.figure(1)
# for i in range(n_road):
#     for j in range(n_lane):
#         plt.subplot(4, 3, 3*i+j+1)
#         plt.plot(X,ACT_Queue_Length[i,j,:])
# plt.show()
ACT_phase = "log/ACT_phases.npy"
Phase = np.load(ACT_phase)
plt.figure(1)
plt.plot(X,ACT_Queue_Length[3,1,:])
plt.plot(X,Phase==0)
plt.show()