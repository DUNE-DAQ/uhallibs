#include <stdint.h>                        // for uint32_t, uint8_t
#include <string>                          // for string
#include <vector>                          // for vector
#include <utility>                         // for pair

namespace uhal {

class PacketFmt {
      public:
        PacketFmt(const uint8_t* const, const size_t);
        PacketFmt(const std::vector< std::pair<const uint8_t*, size_t> >& aData);
        ~PacketFmt();

        const std::vector< std::pair<const uint8_t*, size_t> > mData;
      };

      class DMAFmt {
      public:
        DMAFmt(const uint8_t* const, const size_t);
        ~DMAFmt();

        const std::vector< std::pair<const uint8_t*, size_t> > mData;
      };

}