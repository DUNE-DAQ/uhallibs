/*
---------------------------------------------------------------------------

    This file is part of uHAL.

    uHAL is a hardware access library and programming framework
    originally developed for upgrades of the Level-1 trigger of the CMS
    experiment at CERN.

    uHAL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    uHAL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with uHAL.  If not, see <http://www.gnu.org/licenses/>.


      Andrew Rose, Imperial College, London
      email: awr01 <AT> imperial.ac.uk

      Marc Magrans de Abril, CERN
      email: marc.magrans.de.abril <AT> cern.ch

      Tom Williams, Rutherford Appleton Laboratory, Oxfordshire
      email: tom.williams <AT> cern.ch

---------------------------------------------------------------------------
*/

/**
    @file
    @author Tom Williams
    @date September 2017
*/

#ifndef _uhal_ProtocolAxi4Lite_hpp_
#define _uhal_ProtocolAxi4Lite_hpp_


#include <chrono>
#include <deque>                           // for deque
#include <memory>
#include <mutex>
#include <stddef.h>                        // for size_t
#include <stdint.h>                        // for uint32_t, uint8_t
#include <string>                          // for string
#include <utility>                         // for pair
#include <vector>                          // for vector


#include "uhal/ClientInterface.hpp"
#include "uhal/log/exception.hpp"
#include "uhal/ProtocolIPbus.hpp"

#include "uhal/ipc/RobustMutex.hpp"
#include "uhal/ipc/SharedObject.hpp"
#include "uhal/formatters.hpp"

namespace uhal
{
  class Buffers;
  struct URI;

  namespace exception
  {
    //! Exception class to handle the case in which the Axi4Lite connection timed out.
    UHAL_DEFINE_DERIVED_EXCEPTION_CLASS ( Axi4LiteTimeout , ClientTimeout , "Exception class to handle the case in which the Axi4Lite connection timed out." )
    //! Exception class to handle a failure to read from the specified device files during initialisation
    UHAL_DEFINE_DERIVED_EXCEPTION_CLASS ( Axi4LiteInitialisationError , TransportLayerError , "Exception class to handle a failure to read from the specified device files during initialisation." )
    //! Exception class to handle a low-level seek/read/write error after initialisation
    UHAL_DEFINE_DERIVED_EXCEPTION_CLASS ( Axi4LiteCommunicationError , TransportLayerError , "Exception class to handle a low-level seek/read/write error after initialisation." )
  }

  //! Transport protocol to transfer an IPbus buffer over Axi4Lite
  class PCIeAxi4Lite : public IPbus< 2 , 0 >
  {

  private:
    typedef ipc::RobustMutex IPCMutex_t;
    typedef std::unique_lock<IPCMutex_t> IPCScopedLock_t;

    ipc::SharedObject<IPCMutex_t> mIPCMutex;
    bool mIPCExternalSessionActive;
    uint64_t mIPCSessionCount;
  };

} // namespace uhal

#endif /* _uhal_ProtocolAxi4Lite_hpp_ */
