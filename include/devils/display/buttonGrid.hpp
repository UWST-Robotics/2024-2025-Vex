#pragma once

#include "liblvgl/lvgl.h"
#include <vector>
#include <string>

namespace devils
{
    class ButtonGrid
    {
    public:
        struct ButtonData
        {
            std::string title;
            lv_event_cb_t callback;
        };

        ButtonGrid(uint8_t rows, uint8_t cols, std::vector<ButtonData> buttonData)
            : rows(rows),
              cols(cols)
        {
            root = lv_obj_create(NULL);
            lv_scr_load(root);

            grid = lv_obj_create(root);
            lv_obj_set_size(grid, 240, 240);
            lv_obj_align(grid, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_pos(grid, 0, 0);
            lv_obj_set_style_radius(grid, 0, 0);
            lv_obj_set_style_border_width(grid, 0, 0);
            lv_obj_set_style_bg_color(grid, lv_color_make(3, 36, 53), 0);
            lv_obj_set_scrollbar_mode(grid, LV_SCROLLBAR_MODE_OFF);

            buttons.reserve(buttonData.size());

            uint8_t row = 0;
            uint8_t col = 0;

            for (uint16_t i = 0; i < buttonData.size(); i++)
            {
                // Fetch Data
                ButtonData data = buttonData[i];

                // Create Button
                lv_obj_t *button = lv_btn_create(grid);
                lv_obj_set_size(button, 60, 60);
                lv_obj_align(button, LV_ALIGN_CENTER, 0, 0);
                lv_obj_set_pos(button, col * 60, row * 60);
                lv_obj_set_style_radius(button, 0, 0);
                lv_obj_set_style_border_width(button, 0, 0);
                lv_obj_set_style_bg_color(button, lv_color_make(255, 255, 255), 0);
                lv_obj_add_event_cb(button, data.callback, LV_EVENT_CLICKED, NULL);
                buttons.push_back(button);

                // Create Label
                lv_obj_t *label = lv_label_create(button);
                lv_label_set_text(label, data.title.c_str());
                lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

                // Update Row and Column
                col++;
                if (col >= cols)
                {
                    col = 0;
                    row++;
                }
            }
        }
        ~ButtonGrid()
        {
            lv_obj_del(root);
        }

    private:
        lv_obj_t *root;
        lv_obj_t *grid;

        std::vector<lv_obj_t *> buttons;

        uint8_t rows;
        uint8_t cols;
    };
}