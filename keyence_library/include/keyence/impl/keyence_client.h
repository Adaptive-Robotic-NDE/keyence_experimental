#ifndef KEYENCE_CLIENT_H
#define KEYENCE_CLIENT_H

#include <keyence/impl/keyence_exception.h>
#include <keyence/impl/keyence_message.h>
#include <keyence/impl/keyence_utils.h>

#include <sstream>
#include <string>

namespace keyence
{


class Client
{
public:
  virtual int send(const void* data, std::size_t size) = 0;
  virtual int recv(void* data, std::size_t size) = 0;

  // Supporting type. For a given request message, this template
  // generates a response type which contains the appropriate body data
  // and a fixed header
  template <typename ReqT>
  struct Response
  {
    Message::ResponseHeader header;
    typename ReqT::response_type body;

    /// What this reply is. See ResponseStatus: only `ok` means
    /// `body` was decoded and holds anything.
    ResponseStatus status = ResponseStatus::ok;
    /// Bytes received on the wire, and the part of them the body decoder saw.
    std::size_t received = 0;
    std::size_t body_size = 0;
    /// Hex of the first bytes, filled in ONLY when status is not `ok`, for the
    /// caller to log at DEBUG. Empty otherwise: it is a string allocation per
    /// reply, and a healthy stream is thousands of replies.
    std::string dump;

    /// Did the controller accept the request? Says nothing about a payload.
    bool good() const { return header.return_code == 0; }
    /// Is there a decoded body to read? THIS is the one to branch on.
    bool has_data() const { return status == ResponseStatus::ok; }
  };

  // Synchronous interface
  // May throw instance of 'KeyenceException' on error
  template <typename ReqT>
  Response<ReqT> sendReceive(ReqT& msg)
  {
    // Get total size of request
    const std::size_t total_size = ReqT::size + // size of data payload
                                   Message::request_header_size + // size of header + fixed body segment
                                   Message::prefix_size; // size of message prefix indicating size

    // Ensure space and then encode message into local buffer
    buffer_.ensure(total_size);
    encodeRequestHeader(ReqT::command_code, ReqT::size, buffer(buffer_));
    // call message's encode function
    msg.encodeInto(buffer(buffer_) + (Message::prefix_size + Message::request_header_size));

    // Send out the raw bytes and recieve back into same buffer.
    // Output is overwritten
    const std::size_t response_size = sendAndGetResponse(buffer_, total_size);

    // Decode response.
    //
    // LOCAL PATCH: hand the decoders the number of bytes actually received,
    // not buffer_.size(). StreamBuffer::ensure only ever grows, so the vector's
    // capacity is a high-water mark: pass that and every response after the
    // largest one looks longer than it is, and no size check downstream can
    // mean anything.
    Response<ReqT> response;
    response.received = response_size;
    response.body_size = response_size - Message::response_header_size;
    decodeResponseHeader(MutableBuffer(buffer_.data(), response_size), response.header);

    // LOCAL PATCH: CLASSIFY BEFORE DECODING.
    //
    // Running decodeFrom() unconditionally makes two whole classes of
    // perfectly normal reply indistinguishable from corruption:
    //
    //  * a non-zero return code, whose body is an error report and not a
    //    payload at all. The callers' own `if (!resp.good())` handling could
    //    never run, because the decoder threw on the way to it.
    //  * a reply that carries NO payload -- which for a profile request is
    //    what a controller with its laser off says all day, and is not an
    //    error in any sense. See ResponseStatus.
    //
    // Both now come back as a status. Genuine corruption and a desynchronised
    // stream still throw, which is what the reconnect loop is for.
    if (response.header.return_code != 0)
    {
      // 0xA0 IS THE LASER-OFF REPLY, measured against a real LJ-V7080: return
      // code 160 (0xA0) with controller status 0, interleaved with good
      // profiles as the laser is switched. It means "nothing to give right
      // now", not a fault, so classifying it as controller_error reports an
      // idle head as broken at ERROR. Neither classification reconnects, so
      // only the status word and the log level change. Any OTHER non-zero code
      // is still a genuine refusal.
      response.status = (response.header.return_code == kNoProfileAvailable)
                            ? ResponseStatus::no_data
                            : ResponseStatus::controller_error;
      response.dump = hexDump(buffer_.data(), response_size);
      return response;
    }
    if (!ReqT::response_type::carriesPayload(response.body_size))
    {
      response.status = ResponseStatus::no_data;
      response.dump = hexDump(buffer_.data(), response_size);
      return response;
    }

    try
    {
      response.body.decodeFrom(MutableBuffer(buffer_.data(), response_size) +
                               Message::response_header_size);
    }
    catch (const KeyenceException& ex)
    {
      // The decoder knows what it wanted and not what it got. Append the
      // bytes: this is the branch that reconnects, so it is rare, and it is
      // the one where a log line has to be enough to decide what happened.
      throw KeyenceException(std::string(ex.what()) + ". Response was " +
                             hexDump(buffer_.data(), response_size));
    }
    response.status = ResponseStatus::ok;

    return response;
  }

protected:
  // An LJ-V7000 high-speed profile response is a few kB. This bound only has
  // to be loose enough never to reject a real message and tight enough that a
  // desynchronised stream cannot ask for a huge allocation.
  static const uint32_t max_response_size = 1u << 20;  // 1 MiB

  // Helper function that sends the first 'size' bytes from the stream
  // buffer 'buf' and then receives the response back into that same
  // buffer. Returns the number of bytes received into it.
  std::size_t sendAndGetResponse(StreamBuffer& buf, std::size_t size)
  {
    // Send Request
    this->send(buf.data(), size);

    // Receive response size (the message 'prefix')
    uint32_t response_size;
    this->recv(&response_size, sizeof(response_size));

    // LOCAL PATCH: response_size comes off the wire. Passed straight into
    // ensure(), a desynchronised stream can ask for a 4 GB vector
    // (std::bad_alloc, which nothing catches -> terminate) or return fewer
    // bytes than the fixed header the decoders then read.
    if (response_size < Message::response_header_size || response_size > max_response_size)
    {
      std::ostringstream msg;
      msg << "Response prefix declares an implausible length of " << response_size
          << " bytes (expected " << Message::response_header_size << ".."
          << max_response_size << "); the stream is out of sync. Prefix was "
          << hexDump(&response_size, sizeof(response_size));
      throw KeyenceException(msg.str());
    }

    // Receive response (including header, fixed-body, and message payload)
    buffer_.ensure(response_size);
    this->recv(buf.data(), response_size);
    return response_size;
  }

private:
  StreamBuffer buffer_;
};

}

#endif
