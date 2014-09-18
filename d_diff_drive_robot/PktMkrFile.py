import CalcChkSumFile as ccs
import struct

def PktMkr(speed,rid):
  # Create a command packet for spped control for AX-12
  # [255, 255, 254, 4, 32, 255, 3, 216]
  bfr = []
  bfr.append(255)
  bfr.append(255)
  # append servo ID = 53  ???OR??? 0 = Left Motor, 1 = Right Motor
  # = 254 = Broadcast to all servos on the network
  #bfr.append(254)
  #bfr.append(1)
  bfr.append(rid)
  # append LENGTH = Number of parameters + 2
  # Number of parameters = 2 = L byte & H byte
  bfr.append(4)
  # append instruction = 32 for move, with 2 parameters give L & H bytes
  bfr.append(32)

  """
  i = 753 #32897 #5138 #1500 #3235830701  # 0xC0DEDBAD
  s = struct.pack(">L", i)  # ">" = Big-endian, "<" = Little-endian
  #print s         # '\xc0\xde\xdb\xad'
  #print s[0]      # '\xc0'
  d0 = ord(s[0])
  d1 = ord(s[1])
  d2 = ord(s[2])
  d3 = ord(s[3])
  ln = len(s)
  print "len = ", ln
  print "d0 = ", d0
  print "d1 = ", d1
  print "d2 = ", d2
  print "d3 = ", d3
  #print ord(s[0]) # 192 (which is 0xC0)
  """

  s = struct.pack(">L", speed)  # ">" = Big-endian, "<" = Little-endian
  # The above "s" consists of 4 bytes.  
  # The order (index 0,1,2,3) is Highest Byte to Lowest Byte, Left to Right
  # For number less than or equal to 65535 = FFFF = 16 bits, then only want
  # bytes 3 & 4!
  #
  # BUT - max speed = x3ff = 1023
  """
  print "speed = ", speed
  print "Byte0 = ", ord(s[0])
  print "Byte1 = ", ord(s[1])
  print "Byte2 = ", ord(s[2])
  print "Byte3 = ", ord(s[3])
  """
  LoByte = ord(s[3])
  HiByte = ord(s[2])
  # append L byte
  bfr.append(LoByte)
  # append H byte
  bfr.append(HiByte)
  # append checksum
  checksum = ccs.CalcChkSum(bfr)
  bfr.append(checksum)
  
  """
  bfrhex = []
  for bfrel in bfr:
    bfrhex.append(hex(bfrel))
  return bfrhex
  """
  return bfr
