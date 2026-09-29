#ifndef KEYENCE_UTILS_H_
#define KEYENCE_UTILS_H_

#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

namespace keyence
{

template <typename T>
void extract(const void* ptr, T& target, size_t offset = 0)
{
  std::memcpy(&target, static_cast<const char*>(ptr) + offset, sizeof(T));
}

template<typename T>
void insert(void* ptr, const T& data, size_t offset = 0)
{
  std::memcpy(static_cast<char*>(ptr) + offset, &data, sizeof(T));
}

/**
 * @brief The first @e max_bytes of a response, as hex, for a log line.
 *
 * A length alone does not identify an unexpected reply: "truncated: 24 bytes"
 * leaves the return code to be INFERRED (48 wire bytes = 24 header + 24 body)
 * rather than read. Print the bytes instead, and the next such reply is
 * settled by looking at it.
 *
 * The default of 64 bytes covers the response header, the fixed body segment
 * and the whole profile header (the fixed fields end 72 bytes in), which is
 * every field anything here decides on; the profile payload itself is
 * thousands of bytes and says nothing about the classification.
 */
inline std::string hexDump(const void* data, std::size_t size, std::size_t max_bytes = 64)
{
  const unsigned char* bytes = static_cast<const unsigned char*>(data);
  const std::size_t shown = size < max_bytes ? size : max_bytes;
  std::ostringstream os;
  os << size << " bytes";
  if (shown < size)
  {
    os << ", first " << shown;
  }
  os << ':' << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < shown; ++i)
  {
    os << ' ' << std::setw(2) << static_cast<unsigned>(bytes[i]);
  }
  return os.str();
}

}

#endif