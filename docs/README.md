# uhallibs
uHAL extension libraries for DUNE DAQ ipbus platforms

This package provides two custom transports for IPBus :
- `ipbusflx-2.0`: via FELIX register map
- `ipbusaxi4lite-2.0`: via Axi4Lite memory mapping

## IPbus over FELIX register map

## IPBus over AXI4Lite memory mapping

### Minimal examples for use are shown below.

```python
import uhal
from ctypes import cdll

cdll.LoadLibrary("libuhallibs.so")
manager = uhal.ConnectionManager("file://my_connections.xml", ["ipbusflx-2.0"])
```

And in C++ :

```cpp
uhal::ConnectionManager cm( std::string("file://my_connections.xml"), {"ipbusflx-2.0"} );
```
