#ifndef KEYENCE_HIGH_SPEED_SINGLE_PROFILE_H
#define KEYENCE_HIGH_SPEED_SINGLE_PROFILE_H

#include <keyence/impl/high_speed_defs.h>
#include <keyence/impl/keyence_message.h>

namespace keyence
{
namespace command
{

class SingleProfile
{
public:
  // Forward declares
  struct Request;
  struct Response;

  struct Request
  {
    typedef Response response_type;
    
    const static uint32_t size = 16;
    const static uint8_t command_code = 0x42;

    // Required encode function
    void encodeInto(MutableBuffer buffer);
  };

  struct Response
  {
    typedef int32_t profile_pt_t;

    // Offsets WITHIN THE BODY -- what decodeFrom() is handed, which starts
    // Message::response_header_size (24) bytes into the wire message.
    //
    //   0 .. 24   the GetProfile common block. Always present.
    //  24 .. 48   the profile header: num_profiles, data_unit, x_start,
    //             x_increment, trigger_count, encoder_count. Present only
    //             when a profile is attached.
    //  60 ..      the packed profile points, 20 bits each.
    static constexpr std::size_t common_block_end = 24;
    static constexpr std::size_t fixed_field_end = 48;
    static constexpr std::size_t profile_data_offset = 60;

    /**
     * @brief Is there a profile attached to a body of @e body_size bytes?
     *
     * A controller that has nothing to give -- laser off, no trigger, nothing
     * in range -- answers the request normally and attaches no profile: a
     * 48-byte wire message, so a 24-byte body, the common block and no more.
     * It is a legitimate answer, and Client::sendReceive turns it into
     * ResponseStatus::no_data rather than handing it here.
     *
     * Anything longer claims to carry a profile and is decoded; if it is then
     * too short for the header it names, THAT is a truncation and decodeFrom()
     * still throws.
     */
    static bool carriesPayload(std::size_t body_size)
    {
      return body_size > common_block_end;
    }

    ProfileInformation profile_info;
    uint32_t trigger_count;
    uint32_t encoder_count;
    std::vector<profile_pt_t> profile_points;

    void decodeFrom(MutableBuffer buffer);
  };

};

} // namespace command
} // namespace keyence

#endif
