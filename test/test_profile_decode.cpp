// The profile decoder's bounds checks, which no amount of running against a
// healthy controller exercises.
//
// Unbounded, SingleProfile::Response::decodeFrom() reads `num_profiles` -- a
// uint16_t -- straight off the wire and then reads num_profiles * 5 / 2 bytes
// from offset 60 with no reference to the buffer's size, so up to 163 kB past
// the end of a ~2 kB buffer. That is a SIGSEGV of the driver seconds after a
// successful connect. Client::sendReceive() also calls decodeFrom() BEFORE
// anyone checks good(), so an error response, whose body is not a profile at
// all, takes the same path.
//
// These are the malformed responses that must throw KeyenceException, which
// the driver's reconnect loop catches, rather than reading out of bounds.
//
// These are the DECODER's rules. A body carrying no profile at all -- 24
// bytes, the common block and nothing after it, which is what a controller
// with its laser off answers -- never reaches the decoder: Client::sendReceive
// classifies it as ResponseStatus::no_data first. See
// test_response_classification.cpp. decodeFrom() still refuses it, because a
// caller that reaches here has already been told there is a profile attached.

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include <keyence/impl/keyence_buffer.h>
#include <keyence/impl/keyence_exception.h>
#include <keyence/impl/keyence_utils.h>
#include <keyence/impl/messages/high_speed_single_profile.h>

namespace
{
// Offsets decodeFrom() reads, relative to the buffer it is handed (which
// sendReceive() places at Message::response_header_size into the response).
const std::size_t kNumProfiles = 24;
const std::size_t kDataUnit = 26;
const std::size_t kXStart = 28;
const std::size_t kXIncrement = 32;
const std::size_t kProfileData = 60;

/// A well-formed single-profile body carrying `points` points.
std::vector<char> body(uint16_t points, std::size_t size = 0)
{
  if (size == 0)
  {
    size = kProfileData + static_cast<std::size_t>(points) * 5 / 2;
  }
  std::vector<char> buf(size, 0);
  keyence::insert(buf.data(), points, kNumProfiles);
  keyence::insert(buf.data(), static_cast<uint16_t>(50), kDataUnit);
  keyence::insert(buf.data(), static_cast<int32_t>(-40000), kXStart);
  keyence::insert(buf.data(), static_cast<int32_t>(100), kXIncrement);
  return buf;
}

keyence::command::SingleProfile::Response decode(std::vector<char>& buf)
{
  keyence::command::SingleProfile::Response resp;
  resp.decodeFrom(keyence::MutableBuffer(buf.data(), buf.size()));
  return resp;
}
}  // namespace

TEST(ProfileDecode, AGoodResponseStillDecodes)
{
  // 800 points across the laser line is what an LJ-V7080 delivers.
  std::vector<char> buf = body(800);
  keyence::command::SingleProfile::Response resp;
  ASSERT_NO_THROW(resp = decode(buf));
  EXPECT_EQ(800, resp.profile_info.num_profiles);
  EXPECT_EQ(800u, resp.profile_points.size());
  EXPECT_EQ(50, resp.profile_info.data_unit);
  EXPECT_EQ(-40000, resp.profile_info.x_start);
}

TEST(ProfileDecode, ALongerBufferThanTheDataIsFine)
{
  // StreamBuffer::ensure() only grows, so the buffer is routinely larger than
  // the response. That must not be mistaken for a malformed message.
  std::vector<char> buf = body(800, kProfileData + 800 * 5 / 2 + 4096);
  EXPECT_NO_THROW(decode(buf));
}

TEST(ProfileDecode, ABufferTooShortForTheHeaderThrows)
{
  // Anything under the 48 bytes of fixed fields: the extracts themselves
  // would have read past the end.
  for (std::size_t size : {std::size_t(0), std::size_t(24), std::size_t(47)})
  {
    std::vector<char> buf(size, 0);
    EXPECT_THROW(decode(buf), keyence::KeyenceException) << "size " << size;
  }
}

TEST(ProfileDecode, WhereTheProfileBeginsIsStatedOnce)
{
  // The offsets this file hard-codes are the header's, so that the decoder and
  // Client::sendReceive's classification cannot drift apart. If this fails,
  // one of the two moved and the other did not -- which is exactly the class
  // of mistake that made a 24-byte "no profile" reply read as a truncation.
  typedef keyence::command::SingleProfile::Response R;
  EXPECT_EQ(kNumProfiles, R::common_block_end);
  EXPECT_EQ(std::size_t(48), R::fixed_field_end);
  EXPECT_EQ(kProfileData, R::profile_data_offset);
}

TEST(ProfileDecode, ABodyWithNoProfileAttachedIsNotAPayload)
{
  // carriesPayload() is what keeps a laser switched off out of the decoder.
  // A body of exactly the common block has no profile header and no points;
  // one byte more claims a profile, and is decoded (and refused, if short).
  typedef keyence::command::SingleProfile::Response R;
  EXPECT_FALSE(R::carriesPayload(0));
  EXPECT_FALSE(R::carriesPayload(R::common_block_end - 1));
  EXPECT_FALSE(R::carriesPayload(R::common_block_end));
  EXPECT_TRUE(R::carriesPayload(R::common_block_end + 1));
  EXPECT_TRUE(R::carriesPayload(R::profile_data_offset + 2000));
}

TEST(ProfileDecode, APointCountLargerThanTheBufferThrows)
{
  // The segfault. 65528 points -- a whole number of 8-point rows, so it gets
  // past the grouping check -- claims 163820 bytes of packed data; the buffer
  // holds 2060.
  std::vector<char> buf = body(800);
  keyence::insert(buf.data(), static_cast<uint16_t>(65528), kNumProfiles);
  EXPECT_THROW(decode(buf), keyence::KeyenceException);

  // And the boundary: one 8-point row too many for a buffer sized for 800.
  // 808 points want 60 + 2020 bytes; the buffer holds 60 + 2000.
  std::vector<char> exact = body(800);
  keyence::insert(exact.data(), static_cast<uint16_t>(808), kNumProfiles);
  EXPECT_THROW(decode(exact), keyence::KeyenceException);
}

TEST(ProfileDecode, APointCountThatIsNotAWholeNumberOfRowsThrows)
{
  // The packing is 8 points to a 20-byte row and the unpacker's loop advances
  // 8 at a time against `j < num_pp`, so anything else makes it write past
  // the end of the output vector.
  for (uint16_t points : {uint16_t(0), uint16_t(1), uint16_t(2), uint16_t(799),
                          uint16_t(802)})
  {
    std::vector<char> buf = body(800);
    keyence::insert(buf.data(), points, kNumProfiles);
    EXPECT_THROW(decode(buf), keyence::KeyenceException) << "points " << points;
  }
}

TEST(ProfileDecode, TheMessageNamesTheCountAndTheBuffer)
{
  // The point of throwing rather than dying: the reconnect loop logs this.
  std::vector<char> buf = body(800);
  keyence::insert(buf.data(), static_cast<uint16_t>(65528), kNumProfiles);
  try
  {
    decode(buf);
    FAIL() << "expected a KeyenceException";
  }
  catch (const keyence::KeyenceException& ex)
  {
    const std::string what(ex.what());
    EXPECT_NE(std::string::npos, what.find("65528")) << what;
    EXPECT_NE(std::string::npos, what.find("buffer holds")) << what;
  }
}
