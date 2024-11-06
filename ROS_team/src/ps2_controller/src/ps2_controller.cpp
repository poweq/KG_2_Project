// ps2_controller.cpp

#include "ps2_controller/ps2_controller.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <linux/joystick.h>
#include <string.h>

// 함수 구현

int ps2_open(const char *file_name)
{
    int ps2_fd;

    ps2_fd = open(file_name, O_RDONLY);
    if (ps2_fd < 0)
    {
        perror("open");
        return -1;
    }

    return ps2_fd;
}

int ps2_map_read(int ps2_fd, ps2_map_t *map)
{
    int len;
    struct js_event js;

    len = read(ps2_fd, &js, sizeof(struct js_event));
    if (len < 0)
    {
        perror("read");
        return -1;
    }

    int type = js.type & ~JS_EVENT_INIT; // 초기화 이벤트 제거
    int number = js.number;
    int value = js.value;

    map->time = js.time;

    if (type == JS_EVENT_BUTTON)
    {
        // 버튼 이벤트 처리
        // ...
    }
    else if (type == JS_EVENT_AXIS)
    {
        // 축 이벤트 처리
        // ...
        switch(number)
        {
            case PS2_AXIS_LX:
                map->lx = value;
                break;

            case PS2_AXIS_LY:
                map->ly = value;
                break;

            case PS2_AXIS_RX:
                map->rx = value;
                break;

            case PS2_AXIS_RY:
                map->ry = value;
                break;

            // 나머지 축 처리
            // ...
        }
    }

    return len;
}

void ps2_close(int ps2_fd)
{
    close(ps2_fd);
}
