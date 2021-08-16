#IG cost per byte
IGb = 1.03/1024000

#RS cost per byte
RSb = 0.045/50

#a is number of emails
a = 4000

#b is message size bytes  
b = 338

#c is the estimated total number of bytes 
c = a*b
#c= 350000*60*a*b

#IGmr is the Inmarsat Gov monthly rate
IGmr = 13.45

#uf is the unlock fee (only applies to other )
uf = 60

#RSmr is the Rock Seven monthly rate (in euros)
RSmr = 12

#er is the echange rate between euros to usd
er = 1.10

IG  = function(x){x*((12*IGmr)+(c*IGb)+uf)}

RS = function(x){x*(((12*RSmr)+(c*RSb))*er)}

plot(IG(1:50), ylim=range(0,10000), type='l', col='green', xlab='# of Pop-Up Units', ylab='USD')

plot(RS(1:50), ylim=range(0,10000), type='l', col='red', xlab='# of Pop-Up Units', ylab='USD')



costperunit = (1*((12*IGmr)+(c*IGb)+uf))
    print(costperunit) 
    
npop = 22

totalcost = npop*costperunit

print(totalcost)    
