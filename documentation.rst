0. Overview
===========

:author: Lutz Rossa

:version: 0.1 of 2024-08-15

**EPICS support for Measurement Computing Corporation daq hats for Raspberry Pi**

*vendor of hats:*

  * Measurement Computing Corporation
    <https://www.mccdaq.com/>
  * now Digilent (part of Emerson)
    <https://digilent.com/>

*possible types:*

  * MCC118:
    8-ch 12-bit 100kS/s analog input, single ended -10V...+10V, 
    common clock input/output, common trigger input
  * MCC128:
    8-ch 16-bit 100kS/s analog input, single ended or
    4-ch 16-bit 100kS/s analog input, differential,
    -10V...+10V, -5V...+5V, -2V...+2V, -1V...+1V,
    common clock input/output, common trigger input
  * MCC134:
    4-ch 24-bit thermocouple input
  * MCC152:
    2-ch 12-bit analog output 0V...5V,
    8-ch bit-configureable digital I/O
  * MCC172:
    2-ch 24-bit 51.2kS/s analog input, differential -5V...+5V,
    common trigger input

1. Requirements
===============

1.1. Raspberry PI
-----------------

This should be self-explaining for intended use of the hardware. If possible,
install an operating system, which is supported by EPICS (see next chapter).
Raspberry-Pi-OS_ (Linux) is recommended. Additionally these packets should be
installed on this Linux system:

  * ``git``
  * ``g++`` (GNU C++ compiler)
  * ``make`` (GNU make)
  * optional ``libreadline-dev`` (GNU readline and history)

This documentation often says to use ``make``, which means to build the sources
in the directory with a terminal and using a single command line

  ``make``

. You could build the sources in different ways, if you know how. The output
should not show any errors while compiling or linking output files.

1.2. required MCC library
-------------------------

Please clone this MCC_ library with ``git`` into a separate directory and build
it with ``make``. The library comes with a configuration tool
``daqhats_read_eeproms``, which should be used to read the installed modules
and store some required information.

1.3. required EPICS
-------------------

EPICS documentation could be found here_ . You should clone the repositories of
base_ and asyn_ with ``git`` into separate directories, called *<base>* and
*<asyn>* here. To configure EPICS base, optionally modify file

  *<base>/configure/CONFIG_SITE*

or put a local file

  *<base>/configure/CONFIG_SITE.local*

with a single line

  *BUILD_IOCS=YES*

and build it with ``make``.

To configure the asyn package, modify the file

  *<asyn>/configure/RELEASE*

or put a local file

  *<base>/configure/RELEASE.local*

and point to EPICS base with a single line (absolute directory path)

  *EPICS_BASE=<base>*

and build it with ``make``.

2. MCC DAQ hats support module
------------------------------

2.1. Cloning and configuring
----------------------------

Please clone the support_ module with ``git`` into a separate directory called
*<mccdaqhats>* here. To configure the support module, modify the file

  *<mccdaqhats>/configure/RELEASE*

or put a local file

  *<mccdaqhats>/configure/RELEASE.local*

and point to EPICS base with these lines (absolute directory paths)

  *EPICS_BASE=<base>*

  *ASYN=<asyn>*

.

Please open the file ``<mccdaqhats>/iocBoot/iocmccdaqhats/st.cmd`` in your
favorite text editor and go to the line starting with

  ``dbLoadRecords("generated.db","P=pi,``

and modify substition ``P=pi`` (which is used as prefix) into
another EPICS prefix in your EPICS set up, e.g. ``PI=myprefix`` or something
better.

2.2. Compiling
--------------

Execute ``make`` in the directory *<mccdaqhats>*, which should not produce an
error message.
The build process should generated these directories/files:

  * file ``<mccdaqhats>/bin/linux-arm/mccdaqhats`` : generated IOC
  * directory ``<mccdaqhats>/dbd/`` with EPICS description(s) of possibile fields
  * file ``<mccdaqhats>/iocBoot/iocmccdaqhats/envPaths`` : absolute path information
  * possibly more directories and files, which are not important here.

Use the command ``make distclean`` to remove most of generated files.

2.3. Using
----------

The file ``<mccdaqhats>/iocBoot/iocmccdaqhats/st.cmd`` should contain an
example, how to automatically find and use all installed MCC hats.

There are two ways to start the IOC:

  1) Make the file ``<mccdaqhats>/iocBoot/iocmccdaqhats/st.cmd`` as executable

     ``chmod +x <mccdaqhats>/iocBoot/iocmccdaqhats/st.cmd``

     go into the same directory

     ``cd <mccdaqhats>/iocBoot/iocmccdaqhats/``

     and start it with

     ``./st.cmd``

  2) Go into the same directory

     ``cd <mccdaqhats>/iocBoot/iocmccdaqhats/``

     and start the IOC with the command line

     ``<mccdaqhats>/bin/linux-arm/mccdaqhats st.cmd``

The IOC is started and reads the file ``st.cmd`` as input, which triggers these
important functions:

  1) The line with ``envPaths`` reads the absolute paths, which are used here.
  2) The function ``dbLoadDatabase`` reads and prepares the database definitions.
  3) The function ``mccdaqhats_registerRecordDeviceDriver`` initializes the
     support, that next commands are useable.
  4) The function ``mccdaqhatsInitialize`` initializes the required MCC library
     and searches for / reads all installed modules and generates an internal
     parameter list.
  5) The function ``mccdaqhatsWriteDB`` writes the internal parameter list into
     a file suitable as EPICS database.
  6) The function ``dbLoadRecords`` reads this freshly generated EPICS database
     and generates EPICS PVs for them, here the prefix is important for using
     unique EPICS PV names.
  7) The function ``iocInit`` starts the IOC.

No error message should appear and a prompt as last line. A command ``dbl``
could show existing EPICS PVs. See EPICS documentation for more help.

.. _MCC: https://github.com/mccdaq/daqhats
.. _here: https://epics-controls.org/
.. _base: https://github.com/epics-base/epics-base
.. _asyn: https://github.com/epics-modules/asyn
.. _Raspberry-Pi-OS: https://www.raspberrypi.com/software
.. _support: https://github.com/lrossa/mccdaqhats
