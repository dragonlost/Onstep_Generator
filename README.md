# Onstep_Generator

![OnStep_logo](https://github.com/dragonlost/Onstep_Generator/raw/master/OnStep_Logo_Medium.png)

Program generate config file for Onstep program.

## Compatibility :
Version Beta_0.7 of Generator is compatible with 15/05/2018 OnStep Alpha Version.

The default settings are the __same__ as OnStep

Executable is compatible with __Windows__ and __Linux__ (__MacOS__ *ToDo...*).

## Installation :

__Two methods :__

----

### Executable :

I Use [Pyinstaller](https://www.pyinstaller.org/) For compiling

* For __Windows__ --> [Windows](https://github.com/dragonlost/Onstep_Generator/raw/master/executable/OnStep_Generator_Linux64_vB0.6.zip)
* For __Linux__ --> [Linux](https://github.com/dragonlost/Onstep_Generator/raw/master/executable/OnStep_Generator_Win64_vB0.6.zip)

Unzip and Run Onstep_Generator (*double click*)

----

### Python :

Need install Python 3, 
* for Windows -->  [Python 3.6](https://www.python.org/ftp/python/3.6.5/python-3.6.5-amd64.exe) 
* for Linux   -->`sudo apt install python3-dev` or Anaconda/Miniconda 3
                     
And need add librairis : sip and pyQt5

* For __Linux__ and __MacOs__ (in Shell): `pip install sip` ;
                                          `pip install pyQt5`

* For __Windows__ (in CMD in Admin): `pip install sip` ;
                                     `pip install pyQt5`

Go in Onstep Generator Folder (via command `cd`)    
Run : `python main.py`  (in CMD or Shell)

----

## ToDo :

* Add Tuto and explanation in Readme

* Add Axis5/Focus3 and just activate for Ramps 1.4.
* Add MacOS executable version
* Add Warning for Low Rate
* Add Warning for Low Tracking resolution ([xls file](http://www.stellarjourney.com/assets/downloads/OnStep-Calculations.xls))
* Add Maximum slew rate update dynamically
* Add Steps Per Degree update dynamically
* Add option to generate without comment
* Add French language and control of the display language
* Add comment bubble for parameters
* *Add Possibility for upload OnStep code in controller (teensy and arduino in first time)* 

-------------------------

## Screen Capture 

__Axis 1, Axis 2 and basic Configuration :__

![](https://github.com/dragonlost/Onstep_Generator/raw/master/screen_capt/Onstep_Generator_menu1.png)

__Axis 3 (focus1), Axis 4 (focus2) and Rotator configuration :__ (need add Axis 5 (focus3) for Ramps1.4).

![](https://github.com/dragonlost/Onstep_Generator/raw/master/screen_capt/Onstep_Generator_menu2.png)

__Advanced Configuration :__

![](https://github.com/dragonlost/Onstep_Generator/raw/master/screen_capt/Onstep_Generator_menu3.png)
