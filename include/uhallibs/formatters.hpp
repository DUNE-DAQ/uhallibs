#ifndef _formatters_hpp_
#define _formatters_hpp_

#include <stdint.h> // for uint32_t, uint8_t
#include <string>   // for string
#include <utility>  // for pair
#include <vector>   // for vector

namespace uhallibs {

class PacketFmt
{
public:
  PacketFmt(const uint8_t* const, const size_t);
  PacketFmt(const std::vector<std::pair<const uint8_t*, size_t>>& aData);
  ~PacketFmt();

  const std::vector<std::pair<const uint8_t*, size_t>> mData;
};
std::ostream&
operator<<(std::ostream& aStream, const PacketFmt& aPacket);

class DMAFmt
{
public:
  DMAFmt(const uint8_t* const, const size_t);
  ~DMAFmt();

  const std::vector<std::pair<const uint8_t*, size_t>> mData;
};
std::ostream&
operator<<(std::ostream& aStream, const DMAFmt& aPacket);

} // namespace uhal

#endif /*_formatters_hpp_*/
