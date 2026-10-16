#include "panda_arm_tools/motor_reader.hpp"

namespace panda_arm_tools
{

MotorReader::MotorReader()
{
}

MotorReader::~MotorReader()
{
  close();
}

bool MotorReader::open(const std::string & port, double timeout_sec)
{
  // 工具读取场景：放宽反馈等待超时（默认 0.2s），确保电机有充足时间响应。
  // （实时控制循环用的是协议层默认的 3ms，以快速失败、不阻塞控制周期。）
  protocol_.set_recv_timeout(timeout_sec);
  return protocol_.open_serial(port, panda_arm_hardware::BAUD_RATE, timeout_sec);
}

void MotorReader::close()
{
  if (is_open()) {
    protocol_.close_serial();
  }
}

bool MotorReader::is_open() const
{
  return protocol_.is_open();
}

MotorState MotorReader::read_one(int motor_id)
{
  MotorState st;
  st.motor_id = motor_id;

  try {
    // mode=1(FOC闭环) / timeout=0(禁用超时保护) / 力矩·速度·位置·刚度·阻尼 全 0
    // → kp=kd=0 使 τ=0，电机零力矩自由状态，不会因本指令而转动
    panda_arm_hardware::MotorFeedback fb = protocol_.send_and_receive(
      static_cast<uint8_t>(motor_id), 1, 0,
      0.0, 0.0, 0.0, 0.0, 0.0);

    st.ok = true;
    st.output_pos = fb.output_pos;
    st.ex_pos = fb.ex_pos;
    st.output_spd = fb.output_spd;
    st.output_tor = fb.output_tor;
    st.temp = fb.temp;
    st.vol = fb.vol;
    st.merror = fb.merror;
  } catch (const std::exception & e) {
    st.ok = false;
    st.error_msg = e.what();
  }

  return st;
}

std::vector<MotorState> MotorReader::read_all(const std::vector<int> & motor_ids)
{
  std::vector<MotorState> states;
  states.reserve(motor_ids.size());
  for (int id : motor_ids) {
    states.push_back(read_one(id));
  }
  return states;
}

}  // namespace panda_arm_tools
