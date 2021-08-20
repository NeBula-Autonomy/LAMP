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
                                           colors::DARK_GREY,
                                           colors::WHITE,
                                           colors::ORANGE,
                                           colors::YELLOW,
                                           colors::BROWN,
                                           colors::PINK,
                                           colors::LIME_GREEN,
                                           colors::PURPLE,
                                           colors::CYAN,
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
