import cv2
import numpy as np
import glob
import sys
import os
import csv

#get imagenames from files 
if (len(sys.argv) < 2):
   print 'please specify directory'
else:

  if (sys.argv[1][-1] == '/'):
     path = sys.argv[1] 
  else:
     path = sys.argv[1] + '/'

  print path
  keypoints = []
  descriptors = []
  Images = sorted(glob.glob(path + 'images/range*.png'))
  #Masks =  glob.glob(path + 'masks/*.png')

  # start output file
  csvfile = open('matches.csv','wb')
  csvwriter = csv.writer(csvfile, delimiter=',')
  csvwriter.writerow(['Name, index, match indices ...'])

  # extract sifts from each image
  ii = 0
  for im in Images:
    print im
    img = cv2.imread(im)
    gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  
    sift = cv2.SIFT()
    kp,kd = sift.detectAndCompute(gray,None)
    if kd is None:
      kp = np.array([])
      kd = np.array([])
    keypoints.append(kp)
    descriptors.append(kd)
    img=cv2.drawKeypoints(gray,kp)
    print ii
    ii += 1
    cv2.imshow("image",img)
    cv2.waitKey(10)

  # now do matching
  # BFMatcher with default params
  for i in range(len(Images)):
    csvrow = [i,Images[i]]
    # if not empty
    if len(descriptors[i]) >= 3:
     for j in range(i+1,len(Images)):
     # if not empty
      if len(descriptors[j]) >= 3:
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors[i],descriptors[j], k=2)
        # Apply ratio test
        good = []
        #print "matches: ",len(matches)
        
        for m,n in matches:
          if m.distance < 0.75*n.distance:
             good.append(m)

        if len(good) > 12:
          print i,j, ":",len(descriptors[i]),len(descriptors[j])

          #print "good: " , len(good)
          kp1 = keypoints[i]
          kp2 = keypoints[j] 
          #print m.queryIdx, n
          src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
          dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2) 
          M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,3.0)
          matchesMask = mask.ravel().tolist()
          img1 = cv2.cvtColor(cv2.imread(Images[i]),cv2.COLOR_BGR2GRAY)
          img2 = cv2.cvtColor(cv2.imread(Images[j]),cv2.COLOR_BGR2GRAY)
          h,w = img1.shape
          pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
          dst = cv2.perspectiveTransform(pts,M)
          img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3)
          #cv2.imshow("image",img2)
          print M
          print np.linalg.norm(np.dot(M,np.array([[1],[0],[0]])))
          if (np.linalg.norm(M) > 0.0001):
            csvrow.append(j)
            csvrow.append("   ")
            csvrow.append(Images[j])
        else:
          #print "Not enough matches are found - %d/%d" % (len(good),10)
          matchesMask = None
    csvwriter.writerow(csvrow)

# cv2.drawMatchesKnn expects list of lists as matches.
#img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,flags=2)
  print len(descriptors)
  print descriptors[2]
  '''
  cv2.imwrite('sift_keypoints.jpg',img)'''
