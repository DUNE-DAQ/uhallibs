# uhallibs
uHAL extension libraries for DUNE DAQ ipbus platforms

The main purpose of this package is to provide a custom ipBus transport for use with FELIX (https://atlas-project-felix.web.cern.ch/atlas-project-felix/).

A minimal example for use in Python :
```
import uhal
from ctypes import cdll

cdll.LoadLibrary("libuhallibs.so")
manager = uhal.ConnectionManager("file://my_connections.xml", ['ipbusflx-2.0'])
```

And in C++ :
```
uhal::ConnectionManager cm( std::string("file://my_connections.xml"), {"ipbusflx-2.0"} );
```
