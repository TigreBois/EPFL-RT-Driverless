import numpy as np
import matplotlib.pyplot as plt
import sys

n_straightLine = 41*1
straightLine = np.zeros((2,n_straightLine))
straightLine[1,:] = np.linspace(-15,25,n_straightLine)
startLine = straightLine[:,:16]
endLine = straightLine[:,16:]


thetaRight = np.linspace(np.pi, -np.pi, 16, endpoint=False)
rightCircle = np.array([9.125+9.125*np.cos(thetaRight), 9.125*np.sin(thetaRight)])

thetaLeft = np.linspace(0, 2*np.pi, 16, endpoint=False)
leftCircle = np.array([-9.125+9.125*np.cos(thetaLeft), 9.125*np.sin(thetaLeft)])

skidpad = np.concatenate((startLine[:,:15], rightCircle, rightCircle, leftCircle, leftCircle, endLine[:,1:]), axis=1)
# skidpadEnd = 15 + 2*rightCircle.shape[1] + 2*leftCircle.shape[1] 
skidpadEnd = 79
print(skidpadEnd, skidpad.shape[1])


if __name__ == "__main__":
    print(startLine[1,:])
    # plt.plot(straightLine[0,:], straightLine[1,:], '+', color="orange")
    # plt.plot(startLine[0,:], startLine[1,:], '+', color="orange")
    # plt.plot(endLine[0,:], endLine[1,:], '+', color="violet")
    # plt.plot(rightCircle[0,:], rightCircle[1,:], '+', color="red")
    # plt.plot(leftCircle[0,:], leftCircle[1,:], '+', color="blue")
    # plt.axis('equal')
    # plt.show()

    # for i in range(0, len(skidpad[0,:])):
    #     plt.clf()
    #     plt.figure(1)
    #     plt.ylim(-17,27)
    #     plt.xlim(-20,20)
    #     plt.axis('equal')
    #     plt.plot(skidpad[0,:i], skidpad[1,:i], '+', color="green", zorder=1)
    #     plt.pause(0.01)
    # input('Enter anything to finish program')