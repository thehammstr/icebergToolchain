import csv
import time
import sys
import numpy as np
import utm
import AeroQuaternion as AQ
import matplotlib.pyplot as plt
from scipy import signal
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('../Data/Soquel20121031')
# what's what
_TIME_ = 0
_LAT_ = 1
_LON_ = 2
_DEP_ = 3
_HDG_ = 4
_SPD_ = 5
_BMX_ = 6
_BMY_ = 7
_BMZ_ = 8

def stateParse(rowList, refState = np.zeros((6,1))):
   # state = [x y z phi theta psi]
   [zone,east,north] = utm.latlon_to_utm("Everest",float(rowList[_LAT_]),float(rowList[_LON_]))
   psi = np.deg2rad(float(rowList[_HDG_])) + refState[-1]
   dep = float(rowList[_DEP_])
   state = np.array([[north,east, dep, 0., 0., psi]]).T - refState 
   speed  = float(rowList[_SPD_])
   return [state, speed]

def beamParse(rowList):
   
   bx = float(rowList[_BMX_])
   by = float(rowList[_BMY_])
   bz = float(rowList[_BMZ_])
   beam = np.array([[bx,by,bz]]).T
   return beam

def main(args):
   if (len(sys.argv) < 2) :
      # default
      filename = '../DATA/Soquel20121031/fullWallextractedData.csv'
   else:
      # something exciting?
      filename = sys.argv[1]

   print filename
   if (len(sys.argv) < 3):
      outfilePath = '../DATA/Soquel20121031/'
   else:
      outfilePath = sys.argv[2]
   csvfile = open(filename,'rb')
   datareader = csv.reader(csvfile,delimiter=',')
   header = next(datareader,None)
   print header
   time.sleep(1)
   lastTime = 0.
   scan = []
   # read in data, group by timestamp
   startPos = next(datareader,None)
   print startPos
   startTime = float(startPos[_TIME_])
   lastTime = startTime-.333
   startState,spd = stateParse(startPos)
   lastState = np.zeros((6,1))
   state = startState
   logCount = 0
   maxLogs = 70000
   statehist = np.zeros((maxLogs,6))
   #statehist[logCount] = state.T
   dX = np.zeros((3,1))
   vels = np.zeros((maxLogs,3))
   omegas = np.zeros((maxLogs,1))
   times = np.zeros((maxLogs,1))
   beamCounters = np.zeros((maxLogs,1))
   logCount = 0
   # read in data, group by timestamp
   beams_t = np.zeros((512,3))
   beamCounter = 0
   beamHist = []
   startFlag = True
   for row in datareader:
    if logCount < maxLogs:
      # record time
      time_t = float(row[_TIME_])
      if startFlag:
         # initialize
         logCount = 1
         lastTime = time_t 
         times[0] = time_t
         startFlag = False
      else:
         # run normally
         if (time_t != lastTime):
           # New timestamp. 
           # store beams from last timestamp
           beamCounters[logCount-1] = beamCounter
           beams_t_trim = beams_t[0:beamCounter,:]
           beamHist.append(beams_t_trim) 
           # new state
           dT = time_t - lastTime
           times[logCount] = time_t
           [state,speed] = stateParse(row,startState)
           statehist[logCount] = state.T
           #print statehist[logCount], state
           omega = (state[5] - lastState[5])/(dT) # makes assumption that all w is in z direction
           omegas[logCount] = omega
           # Calculate DVL velocity
           att = AQ.Quaternion([0.,0.,(180./np.pi)*(state[5])])
           # finite differencing of pos for velocity requires a bit of smoothing
           smoothPos = .7*(1.-.9**logCount) # time-varying gain allows quick convergence. Approaches 0.7 asymptotically quite quickly
           dX = smoothPos*dX + (1.-smoothPos)*(state[0:3] - lastState[0:3])
           # rotate dx into body frame for velocity
           vel = np.dot(att.asRotMat,(1./(dT))*(dX))
           #vel = 1./dT*dX
           vels[logCount] = vel.T[0]
           #print vels[logCount], vel.T
           #print vel.T, att.asEule
           logCount += 1
           beams_t = np.zeros((512,3))
           beam_t = beamParse(row);
           beams_t[0,:] = beam_t[:,0]
           beamCounter = 1
           lastTime = time_t
           lastState = state
           if (logCount%1000 == 0):
              print logCount
         else:
            # if we're not on a new timestamp
            # process scan
            if (beamCounter < 500):
               beam_t = beamParse(row);
               beams_t[beamCounter,:] = beam_t[:,0]
               beamCounter += 1
   beamHist.append(beams_t[0:beamCounter,:])
   #print logCount
   # unwrap psi and start at zero
   initialHeading = statehist[0,5]
   stateHist = statehist[0:logCount]
   stateHist[:,5] = np.unwrap(stateHist[:,5])-initialHeading
   Vels = vels[0:logCount]
   Vels[0] = Vels[1]
   Vels[:,0] = signal.medfilt(Vels[:,0],5);
   Vels[:,1] = signal.medfilt(Vels[:,1],7);
   Omegas = omegas[0:logCount]
   Times = times[0:logCount] - times[0]
   BeamCounters = beamCounters[1:logCount+1]
   BeamCounters[-1] = beamCounter
      # rotate positions to make consistent with new psi
 
   psi0 = AQ.Quaternion([0.,0.,180./np.pi*initialHeading])
   posRot = np.dot(psi0.asRotMat,stateHist.T[0:3,:])
   rotatedPos = posRot.T
   rotatedStateHist = np.hstack((rotatedPos,stateHist[:][:,3:]))
   #for ii in range(Vels.shape[0]):
   #   print Vels[ii]
   #plt.plot(Times)
   #plt.plot(rotatedStateHist[:,0],rotatedStateHist[:,1],'r')
   #plt.axis('equal')
   #plt.legend(['vx','vy','vz'])
   #plt.show()
   # concatenate full state
   bigMama = np.hstack((Times,rotatedStateHist,
             np.ones((rotatedStateHist.shape[0],1)),
             Omegas,
             np.ones((rotatedStateHist.shape[0],1)),
             Vels))
   #print to file
   np.savetxt(outfilePath+"fullstatehist.csv",bigMama,fmt='%1.6e',delimiter=",")
   # save state and range readings
   printScansToCSV(bigMama,beamHist,outfile=outfilePath+"state_and_ranges.csv")


def cleanScan(scanInput,subsample = 2):
    scan = scanInput[0:scanInput.shape[0]:subsample]
    #return cleaned scan and normals
    nScans,a = scan.shape
    # range jump threshold for declaring new regions
    threshold = 2.
    # number of returns in a given region needed to declare it good
    nThresh = 5
    norms = np.zeros(nScans)
    norms[0]=np.linalg.norm(scan[0])
    regionLabel = 0
    regions = np.zeros(nScans)
    regionsCount = [1]
    goodRegions = [False]
    dx = np.zeros(nScans-1)
    # bin ranges into regions
    for idx in range(1,nScans):
        norms[idx] = np.linalg.norm(scan[idx])
        dx[idx-1] = norms[idx] - norms[idx-1]
        if (abs(norms[idx] - norms[idx-1]) > threshold):
            if (regionsCount[regionLabel] > nThresh):
                goodRegions[regionLabel] = True
            goodRegions.append(False)
            regionLabel+=1
            regionsCount.append(1)
        regions[idx] = regionLabel
        regionsCount[regionLabel] +=1
    # label last region good or not
    if (regionsCount[regionLabel] > nThresh):
                goodRegions[regionLabel] = True 
    # show stuff
    #plt.plot((1./80.)*norms)
    #plt.plot(regions)
    #print np.array(goodRegions)
    goodindices =  np.array(range(0,regionLabel+1))[np.array(goodRegions)]
    selectors = np.zeros(regions.shape, dtype=bool)
    normMeans = []
    for ind in goodindices:
       mask = (regions == ind)
       regionNorms = norms[mask]
       if np.std(regionNorms) > 2:
          selectors = np.bitwise_or(selectors,mask)
          
    newScan = scan[selectors,:].copy()
    #print norms
    #print selectors
    #print norms[selectors].copy()
    if selectors.any() == True:
      pass
      #plt.plot(np.array(norms)[selectors],'ro')
      #print np.diff(norms[selectors].copy())
    #fig = plt.figure()
    #ax = fig.gca(projection='3d')
    #ax.axis('equal')
    #g = ax.scatter(newScan[:,1],newScan[:,0],-newScan[:,2])
    #plt.plot(norms,'ro')
      #plt.show()
    return [newScan, np.ones((newScan.shape[0],1))]


def printScansToCSV(trajectory,beamHist,outfile='state_and_ranges.csv'):
   print 'writing data to file: ',outfile
   file_object = open(outfile, 'w') 
   # how much we be writing?
   logCnt,stateSize = np.shape(trajectory)
   # header
   file_object.write('Format: time - x - y - z - phi - theta - psi - commanded speed - omega - dvl flag - dvl - measurements (scan number 4dof)\n')
   # ok, write it then
   scanNum = 0
   for ii in range(logCnt):
      # write state info
      file_object.write("%f" % trajectory[ii,0])
      for jj in range(1,stateSize):
         file_object.write(",%f" % trajectory[ii,jj])
      # clean the scan
      scans,normals = cleanScan(beamHist[ii],2)
      numScans = np.shape(scans)[0]
      # write to file
      for jj in range(numScans):
         file_object.write(",%d"%scanNum)
         for val in scans[jj]:
            file_object.write(",%f"%val)
         # include normal inclination info (dummy for now)
         file_object.write(",%f"%0.) 
         scanNum+=1
      file_object.write("\n")
   file_object.close()


def normRows(mat):
    return np.sqrt(np.sum(mat*mat,axis=1))

def normCols(mat):
    return np.sqrt(np.sum(mat*mat,axis=0))

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
