#ifndef KEYENCE_MESSAGE_H
#define KEYENCE_MESSAGE_H

#include <stdint.h>

#include <iostream>
#include <keyence/impl/keyence_buffer.h>
#include <keyence/impl/keyence_utils.h>

namespace keyence
{

/**
 * @brief What a response is, decided from its HEADER and its LENGTH before
 *        any body decoder is handed the bytes.
 *
 * LOCAL PATCH. A controller with its laser switched off answers a profile
 * request with a well-formed 48-byte message: the 24-byte response header and
 * fixed body segment, then the GetProfile common block, and no profile
 * attached. There is nothing wrong with it. Handing those 24 body bytes to
 * SingleProfile::Response::decodeFrom() regardless makes it (rightly) refuse
 * to decode a profile header that is not there, and the exception unwinds past
 * the socket -- tearing down a healthy connection and rebuilding it once a
 * second for as long as the laser stays off.
 *
 * So classify first. `no_data` is a normal answer and is not an error; only
 * corruption and a desynchronised stream still throw.
 */
/**
 * The controller's return code for "I have no profile for you right now".
 *
 * MEASURED, not taken from the manual: a real LJ-V7080 answers a profile
 * request with return code 160 (0xA0) and controller status 0 whenever its
 * laser is off, interleaved with good profiles as the laser is switched.
 * Treated as `no_data` rather than `controller_error` so that an idle head
 * reads as idle -- see Client::sendReceive.
 */
static const uint8_t kNoProfileAvailable = 0xA0;

enum class ResponseStatus
{
  ok,                ///< return code 0, and a body carrying a payload. Decoded.
  no_data,           ///< return code 0, well formed, nothing attached to decode.
  controller_error   ///< non-zero return code; the body is not a payload at all.
};

inline const char* describe(ResponseStatus status)
{
  switch (status)
  {
  case ResponseStatus::ok:
    return "ok";
  case ResponseStatus::no_data:
    return "no data attached";
  case ResponseStatus::controller_error:
    return "controller error return code";
  }
  return "unknown";
}

class Message
{
public:
  static const uint32_t prefix_size = 4;
  static const uint32_t request_header_size = 12 + 4;  //header plus first 4 bytes of body since it is constant
  static const uint32_t response_header_size = 12 + 12;  //header plus first 12 bytes of body since it is constant

  struct RequestHeader
  {
    // Actual header info
    uint32_t body_length;
    // Fixed Body Segment
    uint8_t command_code;
  };

  struct ResponseHeader
  {
    // Actual header info
    uint8_t header_return_code;
    uint32_t body_length;
    // Fixed Body Segment
    uint8_t command_code;
    uint8_t return_code;
    uint8_t controller_status;
    uint8_t active_program_no;
  };
};

inline void encodeRequestHeader(uint8_t code, uint32_t size, MutableBuffer buffer)
{
  insert(buffer.data, static_cast<uint32_t>(Message::request_header_size + size), 0);
  insert(buffer.data, static_cast<uint32_t>(0x00F00001), 4); // fixed
  insert(buffer.data, static_cast<uint32_t>(0x00000000), 8); // fixed
  insert(buffer.data, size + 4, 12); // body size
  insert(buffer.data, code, 16); // command code
  insert(buffer.data, static_cast<uint8_t>(0), 17); // fixed
  insert(buffer.data, static_cast<uint8_t>(0), 18); // fixed
  insert(buffer.data, static_cast<uint8_t>(0), 19); // fixed
}

inline void decodeResponseHeader(MutableBuffer buffer, Message::ResponseHeader& header)
{
  extract(buffer.data, header.header_return_code, 4);
  extract(buffer.data, header.body_length, 8);
  extract(buffer.data, header.command_code, 12);
  extract(buffer.data, header.return_code, 13);
  extract(buffer.data, header.controller_status, 14);
  extract(buffer.data, header.active_program_no, 20);
}

inline std::ostream& operator<<(std::ostream& os, const Message::ResponseHeader& hdr)
{
  os << "Header return code: " << static_cast<int>(hdr.header_return_code) << '\n';
  os << "Body length: " << static_cast<int>(hdr.body_length) << '\n';
  os << "Command code: " << static_cast<int>(hdr.command_code) << '\n';
  os << "Command return code: " << static_cast<int>(hdr.return_code) << '\n';
  os << "Controller status: " << static_cast<int>(hdr.controller_status) << '\n';
  os << "Active program no: " << static_cast<int>(hdr.active_program_no) << '\n'; 
  return os;
}

}

#endif
