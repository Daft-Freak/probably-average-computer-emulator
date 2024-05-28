#pragma once
#include <cstdint>

void init_display();

void write_display(int x, int y, int count, uint8_t *data);
void update_display();

bool display_render_needed();