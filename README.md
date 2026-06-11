# aqita-wd_pm

The repository contains all the C source files for the AQITA propulsion system. The propulsion system is driven by BLDC motors operating in FOC mode. The BLDC motor control system is sensorless w.r.t. the rotor position and uses a special rotor position estimator. The control algorithms are designed to run on the STM32H743XIHx MCU.

The source code in this repository is based on the code for the XOSS (a.k.a Alpha) UAV propulsion system, developed by the UVIONIX team in the period between 2018 and 2021.
Keil uVision v5.26.2.0 configuration

The propulsion system source code was developed using the Keil uVision IDE v5.26.2.0. The following configuration has to be done in order to successfully build the project.

Under "Pack installer" the following Packs have to be installed:

Device Specific

Keil::STM32H7xx_DFP version 2.2.0 (2018-09-04)

Generic

ARM::CMSIS version 5.4.0 (2018-08-01)
ARM::CMSIS-Driver version 2.3.0 (2018-06-15)
Keil::ARM_Compiler version 1.6.0 (2018-09-06)
Keil::MDK-Middleware version 7.7.0 (2018-05-25)

Under the menu "Project/Manage/Project Items" and under the tab "Folders/Extension" configure the "Tool Base Folder" to

C:\Program Files (x86)\Keil_v5\ARM\

Under "Options for Target" and under the tab "Target" set "ARM Compiler" to "V5.06 update 6 (build 750)".
