#include <keyence/impl/messages/high_speed_single_profile.h>
#include <keyence/impl/keyence_exception.h>
#include <keyence/impl/keyence_utils.h>
#include <keyence/impl/ljv7_rawdata.h>

#include <sstream>

void keyence::command::SingleProfile::Request::encodeInto(MutableBuffer buffer)
{
  using keyence::insert;

  insert(buffer.data, static_cast<uint32_t>(0x00000000), 0);
  insert(buffer.data, static_cast<uint32_t>(0x00000000), 4);
  insert(buffer.data, static_cast<uint32_t>(0x1), 8);
  insert(buffer.data, static_cast<uint32_t>(0x00000101), 12);
}

void keyence::command::SingleProfile::Response::decodeFrom(MutableBuffer buffer)
{
  using keyence::extract;

  // Offsets read below: the common block ends at 24, the profile header runs
  // from there to 48, and the packed profile data starts at 60. Declared in
  // the header so that Client::sendReceive's classification and this decoder
  // cannot disagree about where a profile begins.
  const std::size_t kCommonBlockEnd = Response::common_block_end;
  const std::size_t kFixedFieldEnd = Response::fixed_field_end;
  const std::size_t kProfileDataOffset = Response::profile_data_offset;

  // LOCAL PATCH: bound the buffer before reading it. Without the check below,
  // everything that follows runs on whatever bytes the socket produced:
  //   * num_profiles is a uint16_t read straight off the wire at offset 24,
  //   * profile data is then read from offset 60 for num_profiles * 5 / 2
  //     bytes with no reference to how large the buffer actually is, and
  //   * ljv7_unpack_profile_data()'s only src bound is
  //     src_end > src + src_sz, where src_sz is itself derived from
  //     num_profiles -- so it is vacuous and can never catch this.
  // A short, error-carrying or desynchronised response then reads up to
  // 65535 * 5 / 2 = 163 kB past the end of a ~2 kB buffer, which is a SIGSEGV.
  // Throwing KeyenceException instead lands in the driver's reconnect loop,
  // which is survivable.
  //
  // A 24-byte body -- a controller with its laser off saying "no profile" --
  // is neither a truncation nor an error. Client::sendReceive classifies it as
  // ResponseStatus::no_data (Response::carriesPayload) and never calls this,
  // so a body reaching here claims to carry a profile. The message below says
  // so explicitly, because the two cases are told apart by arithmetic on a
  // length and are easy to confuse.
  if (buffer.size < kFixedFieldEnd)
  {
    std::ostringstream msg;
    msg << "Profile response truncated: the body is " << buffer.size
        << " bytes, which is past the " << kCommonBlockEnd
        << "-byte common block -- so a profile is attached -- but short of the "
        << kFixedFieldEnd << " bytes its header needs. A body of exactly "
        << kCommonBlockEnd << " bytes would be the controller saying it has no "
           "profile, which is not this";
    throw KeyenceException(msg.str());
  }

  extract(buffer.data, profile_info.num_profiles, 24);
  extract(buffer.data, profile_info.data_unit, 26);
  extract(buffer.data, profile_info.x_start, 28);
  extract(buffer.data, profile_info.x_increment, 32);
  extract(buffer.data, trigger_count, 40);
  extract(buffer.data, encoder_count, 44);

  // 20 bits per profile point, packed 8 points to a 20-byte "row"
  // (ljv7_rawdata.cpp: bpp=20, ppr=8, bpr=20). The unpacker's loop advances
  // 8 points at a time against `j < num_pp`, so a count that is not a
  // multiple of 8 makes it write up to 7 points PAST the end of the output
  // vector. Its own `src_sz % bpr` test happens to reject every such count
  // first -- floor(5n/2) is a multiple of 20 only when n is a multiple of 8 --
  // but that is arithmetic nobody should have to redo, so say it here.
  if (profile_info.num_profiles == 0 || (profile_info.num_profiles % 8) != 0)
  {
    std::ostringstream msg;
    msg << "Profile response declares " << profile_info.num_profiles
        << " points; the packing is 8 points to a 20-byte row, so the count "
           "must be a non-zero multiple of 8";
    throw KeyenceException(msg.str());
  }

  const std::size_t packed_size = static_cast<std::size_t>(profile_info.num_profiles) * 5 / 2;
  if (buffer.size < kProfileDataOffset + packed_size)
  {
    std::ostringstream msg;
    msg << "Profile response declares " << profile_info.num_profiles << " points ("
        << packed_size << " packed bytes from offset " << kProfileDataOffset
        << ") but the buffer holds only " << buffer.size << " bytes";
    throw KeyenceException(msg.str());
  }

  // Prepare room for the profiles
  profile_points.resize(profile_info.num_profiles);

  unsigned char* src_data = static_cast<unsigned char *>(buffer.data) + kProfileDataOffset;

  const int rc =
      ljv7_unpack_profile_data(src_data,                 // buffer of data from keyence
                               static_cast<uint32_t>(packed_size),  // size of keyence data
                               profile_info.num_profiles, // number of profile points
                               profile_points.data(),     // output buffer
                               static_cast<uint32_t>(profile_points.size() *
                                                     sizeof(profile_pt_t)));  // output bytes
  if (rc != 0)
  {
    std::ostringstream msg;
    msg << "ljv7_unpack_profile_data() rejected the profile response (rc=" << rc << ")";
    throw KeyenceException(msg.str());
  }
}
