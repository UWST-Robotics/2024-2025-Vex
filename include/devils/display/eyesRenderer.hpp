#pragma once

#include "liblvgl/lvgl.h"

namespace devils
{
    class EyesRenderer : Runnable
    {
    public:
        EyesRenderer(lv_obj_t *parent) : Runnable(50)
        {
            this->parent = parent;

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

        void render() {
            fullScreenContainer = lv_obj_create(parent);
            lv_obj_set_size(fullScreenContainer, lv_pct(100), lv_pct(100));
            eyesGroup = lv_obj_create(fullScreenContainer);
            lv_obj_set_size(eyesGroup, 340, 200);
            lv_obj_align(eyesGroup, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_pos(eyesGroup, 0, 0);
            lv_obj_set_style_radius(eyesGroup, 0, 0);
            lv_obj_set_style_border_width(eyesGroup, 0, 0);
            lv_obj_set_style_bg_color(fullScreenContainer, lv_color_make(3, 36, 53), 0);
            lv_obj_set_style_bg_opa(eyesGroup, 0, 0);
            lv_obj_set_scrollbar_mode(eyesGroup, LV_SCROLLBAR_MODE_OFF);
            

            leftEye = makeEye(-100, 0, 120);
            rightEye = makeEye(100, 0, 120);
            leftEyebrow = makeEyebrow(-100, -85, 0, 130);
            rightEyebrow = makeEyebrow(100, -110, 0, 130);

            lv_obj_set_style_bg_color(parent, lv_color_make(3, 36, 53), 0);

            lv_obj_add_event_cb(fullScreenContainer, handleDestroy, LV_EVENT_LONG_PRESSED, this);
            lv_obj_add_event_cb(eyesGroup, handleDestroy, LV_EVENT_LONG_PRESSED, this);
            lv_obj_add_event_cb(leftEye, handleDestroy, LV_EVENT_LONG_PRESSED, this);
            lv_obj_add_event_cb(rightEye, handleDestroy, LV_EVENT_LONG_PRESSED, this);
            lv_obj_add_event_cb(leftEyebrow, handleDestroy, LV_EVENT_LONG_PRESSED, this);
            lv_obj_add_event_cb(rightEyebrow, handleDestroy, LV_EVENT_LONG_PRESSED, this);



            runAsync();

        }

        void destroy()
        {
            stop();
            lv_obj_del(fullScreenContainer);
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

        static void handleDestroy(lv_event_t *e)
        {
            // get target object
            EyesRenderer *renderer = static_cast<EyesRenderer *>(lv_event_get_user_data(e));
            renderer->destroy();

            
        }

        static void removeObject(lv_obj_t *obj)
        {
            if (obj != nullptr)
            {
                lv_obj_del(obj);
                obj = nullptr;
            }
        }

        lv_obj_t *parent;
        lv_obj_t *fullScreenContainer;
        lv_obj_t *eyesGroup;
        lv_obj_t *leftEye;
        lv_obj_t *rightEye;
        lv_obj_t *leftEyebrow;
        lv_obj_t *rightEyebrow;

    };
    
}