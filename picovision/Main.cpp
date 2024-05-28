#include "Display.h"

int main()
{
    init_display();

    while(true)
    {
        if(display_render_needed())
        {
            uint16_t buf[640];
            for(int x = 0; x < 640; x++)
                buf[x] = x / 20;

            for(int y = 0; y < 200; y++)
            {
                write_display(0, y, 640, buf);
            }

            update_display();
        }
    }

    return 0;
}