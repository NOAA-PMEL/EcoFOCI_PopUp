#ESTIMATED DATA USAGE FOR ONE POP-UP UNIT



#b is the number of bytes in a MegaByte
b = 1024000

#x is number of emails
x = 4000

#y is message size bytes  
y = 338

#z is the estimated total number of bytes 
z = x*y




##Iridium Gov commercial gateway costs

  #m is mailbox check fee
  m = .03
  #MB is sbd message cost per MegaByte
  cMB = 1.03
  #cost per byte
  IGb = 1.03/1024000
  print(IGb)

  #estimated cost of all the data transmitted
  #contract charges $1.03 per MB of data transmitted
  #The conversion between MegaBytes and Bytes is 1:1024000
  #this does not include the monthly fee of $13.45
  IGsbd = (IGb*z)
  print(IGsbd)

  #estimated cost for one unit over 1 year
  #includes $60 unlock fee for using gateway other than Rock7
  IGyr = (((cMB/b)*z)+(13.45*12)+60)
  print(IGyr)

  #estimated cost for one unit in drfiter phase per day
  #drfiter messages average 19 messages per day
  #at 110 bytes per message
  IGday = ((cMB/b)*19*110)+(13.45/30)+(60/365)
  print(IGday)
  
  #cost per byte
  IGb = 1.03/1024000
  print(IGb)

##Rock7 iridium contract costs
  #cost per byte
  Rb = 0.045/50
  print(Rb)
  
  #estimated cost of all the data transmitted
  #Rock7 charges a minimum of 0.045 euro per 50 bytes transmitted
  #this does not include a 12euro/mo line rental cost
  #as of 12/18/2019 1 euro = 1.11 USD
  Rsbd = (Rb*z)*1.11
  print(Rsbd)

  #estimated cost for one unit over 1 year
  Ryr = ((z/50)*0.045)+(12*12*1.11)
  print(Ryr)

  #estimated cost for one unit in drifter phase per day 
  #drfiter messages average 19 messages per day
  #at 110 bytes per message
  Rock7day = (((19*110/50)*0.045)+(12*1.11/30))
  print(Rock7day)
  



  #est annual cost Iridium GOV for one unit
  print(IGyr)
  
  #est annual cost Rock7 for one unit
  print(Ryr)
  













