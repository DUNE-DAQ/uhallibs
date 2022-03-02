# uhallibs
uHAL extension libraries for DUNE DAQ ipbus platforms

This package provides two custom transports for ipBus :
- via FELIX register map : "ipbusflx-2.0"
- via Axi4Lite : "ipbusaxi4lite-2.0"

Minimal examples for use are shown below.

```
import uhal
from ctypes import cdll

cdll.LoadLibrary("libuhallibs.so")
manager = uhal.ConnectionManager("file://my_connections.xml", ["ipbusflx-2.0"])
```

And in C++ :
```
uhal::ConnectionManager cm( std::string("file://my_connections.xml"), {"ipbusflx-2.0"} );
```
