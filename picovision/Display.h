#pragma once
#include <cstdint>

namespace pimoroni
{
    class APS6404;
}

void init_display();

void set_display_size(int w, int h);

void write_display(int x, int y, int count, uint8_t *data);
void update_display();

bool display_render_needed();

void display_wait_for_frame();

pimoroni::APS6404 &display_get_ram();
