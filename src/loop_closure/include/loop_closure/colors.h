#pragma once
#include <array>
#include <std_msgs/ColorRGBA.h>

enum colors {
  BLACK = 0,
  BROWN = 1,
  BLUE = 2,
  CYAN = 3,
  GREY = 4,
  DARK_GREY = 5,
  GREEN = 6,
  LIME_GREEN = 7,
  MAGENTA = 8,
  ORANGE = 9,
  PURPLE = 10,
  RED = 11,
  PINK = 12,
  WHITE = 13,
  YELLOW = 14,
  TRANSLUCENT = 15,
  TRANSLUCENT_LIGHT = 16,
  TRANSLUCENT_DARK = 17,

};

const std::array<colors, 14> ALL_COLORS = {colors::BLUE,
                                           colors::GREY,
                                           colors::ORANGE,
                                           colors::PINK,
                                           colors::YELLOW,
                                           colors::LIME_GREEN,
                                           colors::PURPLE,
                                           colors::WHITE,
                                           colors::CYAN,
                                           colors::BROWN,
                                           colors::DARK_GREY,
                                           colors::MAGENTA,
                                           colors::RED,
                                           colors::GREEN};

std_msgs::ColorRGBA getColor(colors color) {
  std_msgs::ColorRGBA result;
  float alpha_ = 0.5;

  switch (color) {
  case RED:
    result.r = 0.8;
    result.g = 0.1;
    result.b = 0.1;
    result.a = alpha_;
    break;
  case colors::GREEN:
    result.r = 0.1;
    result.g = 0.8;
    result.b = 0.1;
    result.a = alpha_;
    break;
  case colors::GREY:
    result.r = 0.9;
    result.g = 0.9;
    result.b = 0.9;
    result.a = alpha_;
    break;
  case colors::DARK_GREY:
    result.r = 0.6;
    result.g = 0.6;
    result.b = 0.6;
    result.a = alpha_;
    break;
  case colors::WHITE:
    result.r = 1.0;
    result.g = 1.0;
    result.b = 1.0;
    result.a = alpha_;
    break;
  case colors::ORANGE:
    result.r = 1.0;
    result.g = 0.5;
    result.b = 0.0;
    result.a = alpha_;
    break;
  case colors::TRANSLUCENT_LIGHT:
    result.r = 0.1;
    result.g = 0.1;
    result.b = 0.1;
    result.a = 0.1;
    break;
  case colors::TRANSLUCENT:
    result.r = 0.1;
    result.g = 0.1;
    result.b = 0.1;
    result.a = 0.25;
    break;
  case colors::TRANSLUCENT_DARK:
    result.r = 0.1;
    result.g = 0.1;
    result.b = 0.1;
    result.a = 0.5;
    break;
  case colors::BLACK:
    result.r = 0.0;
    result.g = 0.0;
    result.b = 0.0;
    result.a = alpha_;
    break;
  case colors::YELLOW:
    result.r = 1.0;
    result.g = 1.0;
    result.b = 0.0;
    result.a = alpha_;
    break;
  case colors::BROWN:
    result.r = 0.597;
    result.g = 0.296;
    result.b = 0.0;
    result.a = alpha_;
    break;
  case colors::PINK:
    result.r = 1.0;
    result.g = 0.4;
    result.b = 1;
    result.a = alpha_;
    break;
  case colors::LIME_GREEN:
    result.r = 0.6;
    result.g = 1.0;
    result.b = 0.2;
    result.a = alpha_;
    break;
  case colors::PURPLE:
    result.r = 0.597;
    result.g = 0.0;
    result.b = 0.597;
    result.a = alpha_;
    break;
  case colors::CYAN:
    result.r = 0.0;
    result.g = 1.0;
    result.b = 1.0;
    result.a = alpha_;
    break;
  case colors::MAGENTA:
    result.r = 1.0;
    result.g = 0.0;
    result.b = 1.0;
    result.a = alpha_;
    break;
  case colors::BLUE:
  default:
    result.r = 0.1;
    result.g = 0.1;
    result.b = 0.8;
    result.a = alpha_;
    break;
  }

  return result;
}

std_msgs::ColorRGBA getColorByIndex(int i) {
  int i_norm = i % ALL_COLORS.size();
  return getColor(ALL_COLORS[i_norm]);
}

namespace pc {
enum PRINT_COLOR {
  BLACK,
  RED,
  GREEN,
  YELLOW,
  BLUE,
  MAGENTA,
  CYAN,
  WHITE,
  ENDCOLOR
};

std::ostream& operator<<(std::ostream& os, PRINT_COLOR c) {
  switch (c) {
  case BLACK:
    os << "\033[1;30m";
    break;
  case RED:
    os << "\033[1;31m";
    break;
  case GREEN:
    os << "\033[1;32m";
    break;
  case YELLOW:
    os << "\033[1;33m";
    break;
  case BLUE:
    os << "\033[1;34m";
    break;
  case MAGENTA:
    os << "\033[1;35m";
    break;
  case CYAN:
    os << "\033[1;36m";
    break;
  case WHITE:
    os << "\033[1;37m";
    break;
  case ENDCOLOR:
    os << "\033[0m";
    break;
  default:
    os << "\033[1;37m";
  }
  return os;
}
} // namespace pc

#define ROS_BLACK_STREAM(x) ROS_INFO_STREAM(pc::BLACK << x << pc::ENDCOLOR)
#define ROS_RED_STREAM(x) ROS_INFO_STREAM(pc::RED << x << pc::ENDCOLOR)
#define ROS_GREEN_STREAM(x) ROS_INFO_STREAM(pc::GREEN << x << pc::ENDCOLOR)
#define ROS_YELLOW_STREAM(x) ROS_INFO_STREAM(pc::YELLOW << x << pc::ENDCOLOR)
#define ROS_BLUE_STREAM(x) ROS_INFO_STREAM(pc::BLUE << x << pc::ENDCOLOR)
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM(pc::MAGENTA << x << pc::ENDCOLOR)
#define ROS_CYAN_STREAM(x) ROS_INFO_STREAM(pc::CYAN << x << pc::ENDCOLOR)

#define ROS_BLACK_STREAM_COND(c, x)                                            \
  ROS_INFO_STREAM_COND(c, pc::BLACK << x << pc::ENDCOLOR)
#define ROS_RED_STREAM_COND(c, x)                                              \
  ROS_INFO_STREAM_COND(c, pc::RED << x << pc::ENDCOLOR)
#define ROS_GREEN_STREAM_COND(c, x)                                            \
  ROS_INFO_STREAM_COND(c, pc::GREEN << x << pc::ENDCOLOR)
#define ROS_YELLOW_STREAM_COND(c, x)                                           \
  ROS_INFO_STREAM_COND(c, pc::YELLOW << x << pc::ENDCOLOR)
#define ROS_BLUE_STREAM_COND(c, x)                                             \
  ROS_INFO_STREAM_COND(c, pc::BLUE << x << pc::ENDCOLOR)
#define ROS_MAGENTA_STREAM_COND(c, x)                                          \
  ROS_INFO_STREAM_COND(c, pc::MAGENTA << x << pc::ENDCOLOR)
#define ROS_CYAN_STREAM_COND(c, x)                                             \
  ROS_INFO_STREAM_COND(c, pc::CYAN << x << pc::ENDCOLOR)
