#profile analysis of FY18 pop-ups interested primarily in rate of ascent 
#and factors influencing rate of ascent
#GOALS with this script: animation of data as pop-up ascends from seafloor to surface,
#compare TTP with SST, response time and precision and accuracy 

require("ggplot2")
require("devtools")
require("lubridate") #make dealing with dates a little easier
require("tidyverse")
require("dplyr") #data transformation 

#import dataset
`300434063921240_profile_data` <- read.csv("~/EcoFOCI_PopUp/Software_and_Analysis/300434063921240/2019/300434063921240_profile_data.csv")
  View(`300434063921240_profile_data`)

#rename dataset for convenience
profile <- `300434063921240_profile_data`

#time
#Fractional seconds are printed only if options("digits.secs") is set: see strftime.
str(as.POSIXlt(profile$datetime, "" tz="UTC"))
options(digits.secs = 4)
unclass(as.POSIXlt(profile$datetime, "GMT"))     

#convert bars to standard meters sea water (msw), add to variable columns
#0.1bar=1msw
profile$msw <- profile$pressure*10

#subset data for ascent (asc) only (>2.8msw)
asc <- subset.data.frame(profile, msw >= 2.8)

#use ggplot...graph change in msw over change in time
ggplot(aes(datetime, msw, data=asc, color="teal", geom="point", xaxt="n", ylab("Depth in Meters"),
  scale_y_reverse())
)

#calculate ascent rate, fit with linear regression, lm is linear model
lin.mod <- lm(msw ~ datetime, data=asc)

#add linear regression line to plot
abline(lin.mod)


