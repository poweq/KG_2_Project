// ps2_controller/src/ps2_controller.cpp

#include "ps2_controller/ps2_controller.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <linux/joystick.h>
#include <string.h>
#include <errno.h>

// 함수 구현

int ps2_open(const char *file_name)
{
    int ps2_fd;

    // O_NONBLOCK 플래그를 추가하여 비차단 모드로 열기
    ps2_fd = open(file_name, O_RDONLY | O_NONBLOCK);
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

    // 모든 이벤트를 읽기 위해 루프 사용
    while ((len = read(ps2_fd, &js, sizeof(struct js_event))) == sizeof(struct js_event))
    {
        int type = js.type & ~JS_EVENT_INIT; // 초기화 이벤트 제거
        int number = js.number;
        int value = js.value;

        map->time = js.time;

        if (type == JS_EVENT_BUTTON)
        {
            // 버튼 이벤트 처리
            switch(number)
            {
                case 0: // 'a' 버튼
                    map->a = value;
                    break;
                case 1: // 'b' 버튼
                    map->b = value;
                    break;
                case 4: // L1 버튼
                    map->l1 = value;
                    break;
                case 5: // R1 버튼
                    map->r1 = value;
                    break;
                // 필요한 다른 버튼 처리
                default:
                    break;
            }
        }
        else if (type == JS_EVENT_AXIS)
        {
            // 축 이벤트 처리
            switch(number)
            {
                case 0: // LX
                    map->lx = value;
                    break;
                case 1: // LY
                    map->ly = value;
                    break;
                case 3: // RX
                    map->rx = value;
                    break;
                case 4: // RY
                    map->ry = value;
                    break;
                // 나머지 축 처리
                default:
                    break;
            }
        }
    }

    if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
        perror("read");
        return -1;
    }

    return 0;
}

void ps2_close(int ps2_fd)
{
    close(ps2_fd);
}
