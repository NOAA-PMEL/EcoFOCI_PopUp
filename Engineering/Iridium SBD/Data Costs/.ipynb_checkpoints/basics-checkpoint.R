packages <- c("DT", "dataone", "datapack", "devtools", "dplyr", "EML", "ggmap", "ggplot2", "leaflet","readxl", "tidyr", "scales", "sf", "rmarkdown", "roxygen2", "usethis", "broom", "captioner")
for (package in packages) {if (!(package %in% installed.packages())){install.packages(package)}}
rm(packages)
rm(package)
