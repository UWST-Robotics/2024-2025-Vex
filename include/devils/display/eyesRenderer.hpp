#pragma once

#include "liblvgl/lvgl.h"

namespace devils
{
    class EyesRenderer : Runnable
    {
    public:
        EyesRenderer() : Runnable(50)
        {
            // root = lv_obj_create(NULL);
            // lv_scr_load(root);

            // eyesGroup = lv_obj_create(root);
            // lv_obj_set_size(eyesGroup, 340, 200);
            // lv_obj_align(eyesGroup, LV_ALIGN_CENTER, 0, 0);
            // lv_obj_set_pos(eyesGroup, 0, 0);
            // lv_obj_set_style_radius(eyesGroup, 0, 0);
            // lv_obj_set_style_border_width(eyesGroup, 0, 0);
            // lv_obj_set_style_bg_color(eyesGroup, lv_color_make(3, 36, 53), 0);
            // lv_obj_set_scrollbar_mode(eyesGroup, LV_SCROLLBAR_MODE_OFF);

            // leftEye = makeEye(-100, 0, 120);
            // rightEye = makeEye(100, 0, 120);
            // leftEyebrow = makeEyebrow(-100, -85, 0, 130);
            // rightEyebrow = makeEyebrow(100, -110, 0, 130);

            // lv_obj_set_style_bg_color(root, lv_color_make(3, 36, 53), 0);

            lv_obj_t * btn = lv_btn_create(lv_scr_act());     
            lv_obj_set_pos(btn, 10, 10);                            
            lv_obj_set_size(btn, 120, 50);                          
            lv_obj_add_event_cb(btn, btnEventCallback, LV_EVENT_ALL, NULL);

            lv_obj_t * label = lv_label_create(btn);
            lv_label_set_text(label, clickCount == 0 ? "Click me!" : std::to_string(clickCount).c_str());
            lv_obj_center(label);

            runAsync();
        }
        ~EyesRenderer()
        {
            lv_obj_del(root);
        }

        void onUpdate() override
        {
            static int16_t t = 0;
            t++;

            lv_obj_set_pos(
                eyesGroup,
                std::cos(t * 0.1) * 20,
                0);
        }

        static void btnEventCallback(lv_event_t *e)
        {
            lv_obj_t *btn = lv_event_get_target(e);
            lv_event_code_t code = lv_event_get_code(e);

            if (code == LV_EVENT_CLICKED)
            {
                lv_obj_set_style_bg_color(btn, lv_color_make(255, 0, 0), 0);
                lv_obj_set_style_bg_color(btn, lv_color_make(3, 36, 53), 0);
            }
        }

    private:
        lv_obj_t *makeEye(int16_t x, int16_t y, int16_t size)
        {
            // Object
            lv_obj_t *eye = lv_obj_create(eyesGroup);
            lv_obj_set_size(eye, size, size);
            lv_obj_align(eye, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_pos(eye, x, y);

            // Style
            lv_obj_set_style_bg_color(eye, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_radius(eye, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(eye, 0, 0);

            return eye;
        }

        lv_obj_t *makeEyebrow(int16_t x, int16_t y, int16_t angle, int16_t size)
        {
            // Object
            lv_obj_t *eyebrow = lv_obj_create(eyesGroup);
            lv_obj_set_size(eyebrow, size, size);

            // Style
            lv_obj_set_style_bg_color(eyebrow, lv_color_make(3, 36, 53), 0);
            lv_obj_set_style_radius(eyebrow, 0, 0);
            lv_obj_set_style_border_width(eyebrow, 0, 0);
            lv_obj_set_style_transform_pivot_x(eyebrow, size / 2, 0);
            lv_obj_set_style_transform_pivot_y(eyebrow, size, 0);
            lv_obj_set_style_transform_angle(eyebrow, angle, 0);

            lv_obj_align(eyebrow, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_pos(eyebrow, x, y);

            return eyebrow;
        }

        lv_obj_t *root;
        lv_obj_t *eyesGroup;
        lv_obj_t *leftEye;
        lv_obj_t *rightEye;
        lv_obj_t *leftEyebrow;
        lv_obj_t *rightEyebrow;
        unsigned int clickCount = 0;
    };
}