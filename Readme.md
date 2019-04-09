The ultimate goal of this project is to have a project that can be easily used to stream raw adc data off of the IWR6843 via the DCA1000EVM. 

Right now the code is in very rough shape but it will stream data from memory out of the IWR6843 via the DCA1000EVM. The next step in development will be to setup the adcbuf to sample  data and transmit it out of the cbuff. 

The code is built against mmwave_sdk_03_01_01_02. 

There are 2 primary pieces to the code.

# multi_gesture_68xx_dss

This is the IWR6843 firmware that sets up the cbuff and streams the data out. You will notice that the there is an mss project as well. This isn't needed right now but may be used in the future. 

# DCA1000CollectC

This is a C desktop application used to receive data from the DCA1000EVM. Note you'll need to have mmwave_studio installed or at least have have a copy of the RF_API dll. Note that the application is currently built against version the DLL included with version 02.00.00.02 of mmWaveStudio.

# Running the Code

Right now the desktop application only records for 20 seconds then the application stops. I would recommend you program the IWR6843 firmware and start it then start the DCA1000CollectC application. 

