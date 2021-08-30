install.packages("tidyverse")
install.packages("ggplot2")
install.packages("colorspace")
install.packages("dplyr")
install.packages("patchwork")

library(ggplot2)
library(dplyr)
library(patchwork)

setwd("~/POPUP/Engineering/Calibrations/Temperature Response")
getwd()

#import calibration bottom data file
#This file contains both sea surface temperature (SST) and Topside Temperature (TTP) readings once every 30 seconds (0.3Hz)
#The temperature in Celsius is calibrated with the Steinhart-Hart Coefficients unique to each probe
cal <- read.table("C:/Users/donohoe/Documents/POPUP/Engineering/Calibrations/2020/2001/BOTDAT.csv",
                      header=TRUE, sep=",")

#import profile data file
#This file contains only TTP readings
prof <- read.table("C:/Users/donohoe/Documents/POPUP/GitHub/ITAE_popup/results/18_01_300434063921240/2019/300434063921240_profile_data.csv",
                   header=TRUE, sep=",")

#create an elapsed_time column (in seconds) by:
#designating date-time format as.POSIXct()
prof$datetime <- as.POSIXct(prof$datetime)
prof$start_time <- as.POSIXct(prof[1,9])
prof$elapsed_time <- ((prof$datetime) - (prof$start_time))
 

#import response test data from unit 2007 
#this test set all delays to 0 in setup and sensor check arduino script 
#except for test probe function, which was set to 250ms
#This test shows sst and temperature reference resistor response at 4Hz 
#Conducted through the setup and sensor check script
sst_4hz <- read.table(file="data/hz4sst07.txt", header=FALSE, sep=",", dec=".")

#name columns
names(sst_4hz)[1] <- "RefRes"
names(sst_4hz)[2] <- "Millis"
names(sst_4hz)[3] <- "TempC"

#create an elapsed time column
sst_4hz$ElapsedTime <- ((sst_4hz$Millis)-(sst_4hz[1,2]))/1000

#fixes ggplot2 bug
dev.off()

#plot profile temperature
profplot_temp <- ggplot(data=prof, aes(x=elapsed_time, y=topside_temp)) +
                geom_line(
                color = "deepskyblue4",
                size = 1) +
                geom_point(
                color = "red",
                size = 1)
profplot_temp

#plot calibration file temperature
calplot_temp <- ggplot(cal, aes(x=time, y=topside_temp))+
                  geom_line(
                  color="blue",
                  size=1)+
                  labs(title = "Temperature Calibration, unit 2001 @ 0.3Hz")

#plot calibration file reference resistance
calplot_ref <-  ggplot(cal, aes(x=time, y=temp_ref))+
                  geom_line(
                  color="red",
                  size=1)+
                  labs(title = "Reference Resistor, unit 2001 @ 0.3Hz")

#show both plots side by side
calplot_temp + calplot_ref

#plot sst @4hz temp
sst4plot_temp <- ggplot(data=sst_4hz, aes(x=ElapsedTime, y=TempC)) +
                  geom_line(
                  color = "deepskyblue4",
                  size = 1) +
                  labs(title = "SST Temp Response at 4Hz")

#plot sst @4hz reference resistor
sst4plot_ref <- ggplot(sst_4hz, aes(x=ElapsedTime, y=RefRes)) +
                    geom_line(
                    color="darkseagreen3",
                    size=1) +
                    labs(title = "SST Reference Resistance at 4Hz")

#show both plots side by side
sst4plot_temp +sst4plot_ref


#alternative way to make plot via pipe...
#sst07_4hz %>%
#  ggplot(aes(x=ElapsedTime, y=TempC))+
#  geom_line()+
#  geom_line(y=RefRes)
