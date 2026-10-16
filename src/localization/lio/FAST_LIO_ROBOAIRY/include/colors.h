// ---- ANSI 颜色宏（终端输出着色） ----
// 用法: std::cout << CLR_GRN "[INFO] Success!" CLR_RST << std::endl;
#ifndef FAST_LIO_COLORS_H
#define FAST_LIO_COLORS_H

#define CLR_RST  "\033[0m"        // 重置
#define CLR_GRN  "\033[0;32m"     // 绿色：成功
#define CLR_YEL  "\033[1;33m"     // 黄色：警告
#define CLR_RED  "\033[0;31m"     // 红色：错误
#define CLR_CYA  "\033[0;36m"     // 青色：次要信息

#endif // FAST_LIO_COLORS_H
