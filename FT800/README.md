# README #


### What is this repository for? ###

* This repository is for using the FT800 EVE (Enhanced Video Engine) with the TI Delfino family

* Version 03

* This current version of the library uses the following GPIO's
    * The list can be found in the function setup_spi_gpio() in FT800.c
    * If it is desired to change the GPIO's, it can easily be done by adding a new #define in FT800.h and a new #ifdef section within
    	* setup_spi_gpio()
    	* ft800_spi_fifo_init
    	* ft800_spi_init


### How do I get set up? ###

* Place root folder somewhere on your computer
* In Code Composer Studio open the project you want to add it to.
* Open windows explorer and navigate to the downloaded library
* Drag the root folder to the desired project in CCS
![drag.png](https://bitbucket.org/repo/ry8kjj/images/2520699173-drag.png)
* A dialog will open and ask how to import the folder
* Select "Link to files and folders"
* Check "Create link locations relative to:" PROJECT_LOC
* Click "OK"
![link_files.png](https://bitbucket.org/repo/ry8kjj/images/4192987007-link_files.png)
* Add the FT800 include folder to the include options of the project

![include.png](https://bitbucket.org/repo/ry8kjj/images/2078103931-include.png) 

* Include the "FT800.h" somewhere in your project
* Declare external global variable for use with display horizontal size and vertical size

```
#!c

//Display Global Variables
extern unsigned int hsize, vsize;
```