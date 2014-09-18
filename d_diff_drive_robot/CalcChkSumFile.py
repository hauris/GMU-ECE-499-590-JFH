def CalcChkSum(bfr):
  accum = 0
  for ii in range(len(bfr)-2):
    accum = accum + bfr[ii+2]
  #print "ACCUM = ", accum
  Caccum = ~accum
  #print "Caccum = ", Caccum
  MaskCaccum = 255 & Caccum
  #print "MaskCaccum = ", MaskCaccum

  return MaskCaccum
