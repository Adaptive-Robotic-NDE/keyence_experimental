// What Client::sendReceive() decides a reply IS, before any decoder sees it.
//
// THE FAILURE THIS IS FOR. With the Keyence laser switched off -- a UR
// digital output -- an unclassified reply makes the driver log this once a
// second for as long as the laser stays off:
//
//   [ERROR] Keyence exception caught: Profile response truncated: 24 bytes,
//           need at least 48 for the profile header. Reconnecting...
//   [INFO]  Connection established to Keyence controller.
//   [INFO]  Beginning with program 1 with a sampling rate of 1000.000000
//
// Nothing is truncated. 24 body bytes is a 48-byte wire message: the 24-byte
// response header and fixed body segment, then the GetProfile common block,
// and no profile attached -- a controller answering "I have nothing to
// measure". Handing it to the profile decoder makes the decoder (rightly)
// refuse, and the exception unwinds past the `keyence::TcpClient` declared
// inside the driver's try block, destroying a perfectly good socket. Connect,
// three settings round-trips, throw, repeat, forever. From outside /profiles
// is silent and the driver looks dead, so the cell appears not to boot with
// the laser off.
//
// So: classify first. These drive a scripted Client, so they need no
// controller, no laser and no network.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <keyence/impl/keyence_client.h>
#include <keyence/impl/keyence_exception.h>
#include <keyence/impl/keyence_utils.h>
#include <keyence/impl/messages/get_setting.h>
#include <keyence/impl/messages/high_speed_single_profile.h>

namespace
{

using keyence::Client;
using keyence::KeyenceException;
using keyence::Message;
using keyence::ResponseStatus;
using keyence::command::GetSetting;
using keyence::command::SingleProfile;

// Offsets within the WIRE message (prefix already consumed), as
// decodeResponseHeader reads them.
const std::size_t kHeaderReturnCode = 4;
const std::size_t kBodyLength = 8;
const std::size_t kCommandCode = 12;
const std::size_t kReturnCode = 13;
const std::size_t kControllerStatus = 14;
const std::size_t kActiveProgram = 20;

// The body starts here; the decoder's own offsets are relative to it.
const std::size_t kBody = Message::response_header_size;   // 24

/// One wire response: header + fixed body segment, then `body_bytes` of body.
std::vector<char> reply(std::size_t body_bytes, uint8_t return_code = 0,
                        uint8_t command_code = 0x42)
{
  std::vector<char> msg(kBody + body_bytes, 0);
  keyence::insert(msg.data(), static_cast<uint8_t>(0), kHeaderReturnCode);
  keyence::insert(msg.data(), static_cast<uint32_t>(body_bytes + 12), kBodyLength);
  keyence::insert(msg.data(), command_code, kCommandCode);
  keyence::insert(msg.data(), return_code, kReturnCode);
  keyence::insert(msg.data(), static_cast<uint8_t>(0), kControllerStatus);
  keyence::insert(msg.data(), static_cast<uint8_t>(1), kActiveProgram);
  return msg;
}

/// The reply a controller gives when it has no profile: 48 bytes on the wire.
/// This is the reply the classification exists for.
std::vector<char> noProfileReply()
{
  return reply(SingleProfile::Response::common_block_end);
}

/// A reply carrying `points` profile points.
std::vector<char> profileReply(uint16_t points)
{
  const std::size_t body = SingleProfile::Response::profile_data_offset +
                           static_cast<std::size_t>(points) * 5 / 2;
  std::vector<char> msg = reply(body);
  char* b = msg.data() + kBody;
  keyence::insert(b, points, 24);                            // num_profiles
  keyence::insert(b, static_cast<uint16_t>(50), 26);         // data_unit
  keyence::insert(b, static_cast<int32_t>(-40000), 28);      // x_start
  keyence::insert(b, static_cast<int32_t>(100), 32);         // x_increment
  keyence::insert(b, static_cast<uint32_t>(7), 40);          // trigger_count
  keyence::insert(b, static_cast<uint32_t>(11), 44);         // encoder_count
  return msg;
}

/// Overwrite the point count of an already-built profile reply WITHOUT
/// resizing it: the message claims more data than it carries.
void claimPoints(std::vector<char>& msg, uint16_t points)
{
  keyence::insert(msg.data() + kBody, points, 24);
}

/**
 * @brief A Client fed from a script of replies instead of a socket.
 *
 * Client::sendAndGetResponse() does one recv() for the 4-byte length prefix
 * and one for the message, so this serves a stream of
 * [prefix][message][prefix][message]... A `prefix_override` exists so that a
 * desynchronised stream -- a length that cannot be right -- can be tested too.
 */
class ScriptedClient : public Client
{
public:
  explicit ScriptedClient(std::vector<std::vector<char>> replies)
    : replies_(std::move(replies))
  {
  }

  /// Declare this length instead of the next message's real one.
  void setPrefixOverride(uint32_t value)
  {
    prefix_override_ = value;
    has_override_ = true;
  }

  int send(const void*, std::size_t size) override
  {
    bytes_sent_ += size;
    return static_cast<int>(size);
  }

  int recv(void* data, std::size_t size) override
  {
    if (pending_.empty())
    {
      if (next_ >= replies_.size())
      {
        throw KeyenceException("scripted client ran out of replies");
      }
      const std::vector<char>& msg = replies_[next_++];
      const uint32_t prefix = has_override_ ? prefix_override_
                                            : static_cast<uint32_t>(msg.size());
      has_override_ = false;
      const char* raw = reinterpret_cast<const char*>(&prefix);
      pending_.assign(raw, raw + sizeof(prefix));
      pending_.insert(pending_.end(), msg.begin(), msg.end());
    }
    if (pending_.size() < size)
    {
      throw KeyenceException("scripted client: remote closed mid-message");
    }
    std::memcpy(data, pending_.data(), size);
    pending_.erase(pending_.begin(), pending_.begin() + static_cast<long>(size));
    return static_cast<int>(size);
  }

  std::size_t bytes_sent() const { return bytes_sent_; }

private:
  std::vector<std::vector<char>> replies_;
  std::vector<char> pending_;
  std::size_t next_ = 0;
  std::size_t bytes_sent_ = 0;
  uint32_t prefix_override_ = 0;
  bool has_override_ = false;
};

}  // namespace

// =============================================================================
// The reply that caused it
// =============================================================================

TEST(Classification, ANoProfileReplyIsNoDataAndDoesNotThrow)
{
  ScriptedClient client({noProfileReply()});
  SingleProfile::Request req;

  Client::Response<SingleProfile::Request> resp;
  ASSERT_NO_THROW(resp = client.sendReceive(req));

  EXPECT_EQ(ResponseStatus::no_data, resp.status);
  EXPECT_FALSE(resp.has_data());
  // The controller ACCEPTED the request. It simply has nothing to measure.
  EXPECT_TRUE(resp.good());
  EXPECT_EQ(0, static_cast<int>(resp.header.return_code));
  EXPECT_EQ(48u, resp.received);
  EXPECT_EQ(24u, resp.body_size);
  EXPECT_TRUE(resp.body.profile_points.empty());
}

TEST(Classification, ANoProfileReplyCarriesTheBytesForTheLog)
{
  // Without the bytes, the return code of this reply can only be inferred
  // from arithmetic on its length. Carry them so it can be read instead.
  ScriptedClient client({noProfileReply()});
  SingleProfile::Request req;
  auto resp = client.sendReceive(req);

  EXPECT_FALSE(resp.dump.empty());
  EXPECT_NE(std::string::npos, resp.dump.find("48 bytes")) << resp.dump;
}

TEST(Classification, NoDataRepeatsForeverWithoutThrowing)
{
  // The acquisition loop has to tolerate this indefinitely -- a laser can be
  // off for a whole shift. Nothing here may accumulate or throw.
  std::vector<std::vector<char>> script;
  for (int i = 0; i < 500; ++i)
  {
    script.push_back(noProfileReply());
  }
  ScriptedClient client(std::move(script));
  SingleProfile::Request req;
  for (int i = 0; i < 500; ++i)
  {
    ASSERT_NO_THROW({
      auto resp = client.sendReceive(req);
      EXPECT_EQ(ResponseStatus::no_data, resp.status);
    }) << "reply " << i;
  }
}

// =============================================================================
// ... and everything that must still work
// =============================================================================

TEST(Classification, AFullProfileStillDecodes)
{
  // 800 points across the laser line is what an LJ-V7080 delivers.
  ScriptedClient client({profileReply(800)});
  SingleProfile::Request req;

  Client::Response<SingleProfile::Request> resp;
  ASSERT_NO_THROW(resp = client.sendReceive(req));

  EXPECT_EQ(ResponseStatus::ok, resp.status);
  EXPECT_TRUE(resp.has_data());
  EXPECT_EQ(800, resp.body.profile_info.num_profiles);
  EXPECT_EQ(800u, resp.body.profile_points.size());
  EXPECT_EQ(50, resp.body.profile_info.data_unit);
  EXPECT_EQ(-40000, resp.body.profile_info.x_start);
  EXPECT_EQ(100, resp.body.profile_info.x_increment);
  EXPECT_EQ(7u, resp.body.trigger_count);
  EXPECT_EQ(11u, resp.body.encoder_count);
  EXPECT_TRUE(resp.dump.empty()) << "a healthy reply should not pay for a hex dump";
}

TEST(Classification, ATruncatedProfileStillThrows)
{
  // Past the common block -- so it claims a profile -- and short of the
  // header that profile needs. THAT is a truncation, and it still throws.
  for (std::size_t body : {std::size_t(25), std::size_t(40), std::size_t(47)})
  {
    ScriptedClient client({reply(body)});
    SingleProfile::Request req;
    EXPECT_THROW(client.sendReceive(req), KeyenceException) << "body " << body;
  }
}

TEST(Classification, ThePointCountOverrunStillThrows)
{
  // The out-of-bounds read: 65528 points (a whole number of 8-point rows, so
  // it passes the grouping check) claims 163820 bytes of packed data from a
  // buffer holding 2060.
  std::vector<char> msg = profileReply(800);
  claimPoints(msg, 65528);
  ScriptedClient client({msg});
  SingleProfile::Request req;

  try
  {
    client.sendReceive(req);
    FAIL() << "expected a KeyenceException";
  }
  catch (const KeyenceException& ex)
  {
    const std::string what(ex.what());
    EXPECT_NE(std::string::npos, what.find("65528")) << what;
    // ... and the bytes, so the next one is read rather than deduced.
    EXPECT_NE(std::string::npos, what.find("Response was")) << what;
  }
}

TEST(Classification, AnErrorReturnCodeIsReportedAndNotDecoded)
{
  // A non-zero return code means the body is an error report, not a profile.
  // Decoding it was how an error response took the SIGSEGV path.
  std::vector<char> msg = profileReply(800);
  keyence::insert(msg.data(), static_cast<uint8_t>(0x23), kReturnCode);
  ScriptedClient client({msg});
  SingleProfile::Request req;

  Client::Response<SingleProfile::Request> resp;
  ASSERT_NO_THROW(resp = client.sendReceive(req));
  EXPECT_EQ(ResponseStatus::controller_error, resp.status);
  EXPECT_FALSE(resp.good());
  EXPECT_FALSE(resp.has_data());
  EXPECT_EQ(0x23, static_cast<int>(resp.header.return_code));
  EXPECT_TRUE(resp.body.profile_points.empty());
  EXPECT_FALSE(resp.dump.empty());
}

TEST(Classification, TheLaserOffReturnCodeIsNoDataNotAnError)
{
  // 0xA0, MEASURED rather than taken from the manual: a real LJ-V7080 answers
  // a profile request with return code 160 and controller status 0 whenever
  // its laser is off, interleaved with good profiles as the laser is switched.
  // That is the head being idle, not refusing: reporting it as
  // controller_error puts an ERROR in the log and `controller_error` on the
  // status topic every time the laser goes off.
  std::vector<char> msg = profileReply(800);
  keyence::insert(msg.data(), keyence::kNoProfileAvailable, kReturnCode);
  ScriptedClient client({msg});
  SingleProfile::Request req;

  Client::Response<SingleProfile::Request> resp;
  ASSERT_NO_THROW(resp = client.sendReceive(req));
  EXPECT_EQ(ResponseStatus::no_data, resp.status);
  EXPECT_FALSE(resp.has_data());
  EXPECT_EQ(160, static_cast<int>(resp.header.return_code));
  // Still not decoded, and the bytes are still kept for the log.
  EXPECT_TRUE(resp.body.profile_points.empty());
  EXPECT_FALSE(resp.dump.empty());
}

TEST(Classification, ADesynchronisedLengthPrefixStillThrows)
{
  // A length prefix comes off the wire and must not reach a vector resize
  // unchecked.
  for (uint32_t prefix : {uint32_t(0), uint32_t(8), uint32_t(0x7FFFFFFF)})
  {
    ScriptedClient client({profileReply(800)});
    client.setPrefixOverride(prefix);
    SingleProfile::Request req;
    EXPECT_THROW(client.sendReceive(req), KeyenceException) << "prefix " << prefix;
  }
}

TEST(Classification, AShortReplyAfterALongOneIsStillShort)
{
  // StreamBuffer::ensure() only ever grows, so after one 2 kB profile the
  // buffer stays 2 kB. Classification must use the bytes RECEIVED, not the
  // buffer's high-water mark -- otherwise a laser switched off mid-scan looks
  // like a full profile of stale bytes, which is worse than any error.
  ScriptedClient client({profileReply(800), noProfileReply()});
  SingleProfile::Request req;

  auto first = client.sendReceive(req);
  ASSERT_EQ(ResponseStatus::ok, first.status);
  ASSERT_EQ(800u, first.body.profile_points.size());

  Client::Response<SingleProfile::Request> second;
  ASSERT_NO_THROW(second = client.sendReceive(req));
  EXPECT_EQ(ResponseStatus::no_data, second.status);
  EXPECT_EQ(48u, second.received);
}

// =============================================================================
// Settings reads take the same route
// =============================================================================

TEST(Classification, ASettingReplyWithAPayloadDecodes)
{
  std::vector<char> msg = reply(4, 0, 0x31);
  msg[kBody] = 0x05;
  ScriptedClient client({msg});
  GetSetting::Request req(0, 0, 0, 0, 0, 0, 0, 0);

  Client::Response<GetSetting::Request> resp;
  ASSERT_NO_THROW(resp = client.sendReceive(req));
  EXPECT_EQ(ResponseStatus::ok, resp.status);
  ASSERT_FALSE(resp.body.data.empty());
  EXPECT_EQ(0x05, resp.body.data[0]);
  EXPECT_EQ(1, static_cast<int>(resp.header.active_program_no));
}

TEST(Classification, ASettingReplyWithNoPayloadIsNoDataNotAnException)
{
  // Every caller reads data[0]. An empty payload is no answer -- but it is a
  // well-formed message, so the transport reports it and the caller decides.
  ScriptedClient client({reply(0, 0, 0x31)});
  GetSetting::Request req(0, 0, 0, 0, 0, 0, 0, 0);

  Client::Response<GetSetting::Request> resp;
  ASSERT_NO_THROW(resp = client.sendReceive(req));
  EXPECT_EQ(ResponseStatus::no_data, resp.status);
  EXPECT_FALSE(resp.has_data());
  EXPECT_TRUE(resp.body.data.empty());
}
