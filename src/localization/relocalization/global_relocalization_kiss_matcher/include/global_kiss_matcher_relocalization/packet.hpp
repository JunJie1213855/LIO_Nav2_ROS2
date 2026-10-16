#ifndef GLOBAL_KISS_MATCHER_RELOCALIZATION_PACKET_HPP
#define GLOBAL_KISS_MATCHER_RELOCALIZATION_PACKET_HPP

namespace global_kiss_matcher_relocalization {
  
enum class RelocState {
  KISS_GLOBAL_INIT,     // KISS 全局初始化粗配准
  GICP_TRACKING,   // GICP 正常连续跟踪
  GLOBAL_RECOVERY  // GICP 连续失败，KISS 重新全局重定位
};

}

#endif