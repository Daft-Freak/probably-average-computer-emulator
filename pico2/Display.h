#pragma once
#include <cstdint>

void init_display();

void set_display_size(int w, int h);

void write_display(int x, int y, int count, uint8_t *data);
void update_display();

bool display_render_needed();

uint8_t *display_get_framebuffer();