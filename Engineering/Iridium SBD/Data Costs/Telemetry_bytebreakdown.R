################################################################################################################################################################

#days on bottom
bd = 360

#single bottom sample .. bb is "bottom bytes"
bb = 17

#botdat is size of total stored bottom data in bytes
#this is based on an hourly sample rate, or 24 samples per day
botdat = (bb*24*bd)

#bm is number of bottom messages generated
#botdat is split into chunks 332 bytes long and given a 6 byte header
#total bm message size is variable, but does not exceed 338
bm = botdat/332


#single profile sample .. pb is "profile bytes"
pb = 11

#prodat is the size of profile data stored in bytes
#profile collects 4 samples per second for 90 seconds
#the first sample is 15 bytes?
prodat = 15+(4*pb*90)

#pm is number of profile messages generated
pm = prodat/332

#days under ice
id = 60

#single under ice sample
ib = 17

#icedat is size of total stored bottom data in bytes
#this is based on an hourly sample rate, or 24 samples per day
icedat = (ib*24*id)

#im is the number of ice messages generated
im= icedat/332

#single under ice image file @....resolution
jpg = 350000

#image files..daily under ice images taken
#total bytes of image files
img = jpg*id

#imgm is number of image files generated
#header for img message is 8 bytes long, leaving 330 bytes of data
imgm = img/330


#days drifting on surface
sd = 160

#single surface sample bytes
sb = 37

#sstdat is total size of surface data
#this is using sample frequency of 3Hrs, or 8 samples per day
sstdat = sb*8*sd

