#ifndef KEYENCE_CHANGE_PROGRAM_H
#define KEYENCE_CHANGE_PROGRAM_H

// #include <rclcpp/rclcpp.hpp>
#include <keyence/impl/keyence_message.h>
#include <keyence/impl/keyence_utils.h>

namespace keyence
{
namespace command
{

class ChangeProgram : public rclcpp::ServiceBase
{
public:
  // Forward declares
  struct Request;
  struct Response;

  ChangeProgram(std::shared_ptr<rclcpp::Node> node);

  std::shared_ptr<void> create_request() override;
  std::shared_ptr<rmw_request_id_s> create_request_header() override;
  void handle_request(std::shared_ptr<rmw_request_id_s> request_header,
                      std::shared_ptr<void> request) override;

  struct Request
  {
    typedef Response response_type;
    const static uint32_t size = 4;
    const static uint8_t command_code = 0x39;

    Request(uint8_t program_no) : program_no(program_no) {}

    void encodeInto(MutableBuffer buffer)
    {
      insert(buffer.data, program_no);
    }

    uint8_t program_no;
  };

  struct Response
  {
    void decodeFrom(MutableBuffer) {}
  };

private:
  std::shared_ptr<rclcpp::Node> node_;
};

} // namespace command
} // namespace keyence

#endif // KEYENCE_CHANGE_PROGRAM_H
