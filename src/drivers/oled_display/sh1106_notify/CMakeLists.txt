px4_add_module(
  MODULE      drivers__oled_display__sh1106_notify
  MAIN        sh1106_notify
  SRCS
    sh1106.cpp
    sh1106_notify.cpp
    font6x8.cpp
  DEPENDS
    #modules__uORB
    px4_work_queue
)
