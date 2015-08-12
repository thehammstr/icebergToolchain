import glob
import sys
import os

totalDigits = 8

if (len(sys.argv) < 2):
   print 'please specify directory'
else:

  if (sys.argv[1][-1] == '/'):
     path = sys.argv[1] 
  else:
     path = sys.argv[1] + '/'

  print path

  Images = glob.glob(path + '*.png')

  for im in Images:
    
    num = im[-9]
    idx = -10
    while (im[idx].isdigit()):
       num = im[idx] + num
       idx = idx-1
 
  
    dst = im[0:idx+1]
    for ii in range(0,totalDigits-len(num)):
       dst = dst+'0'
    dst = dst + num + '.png'
    print dst
    os.rename(im,dst)
