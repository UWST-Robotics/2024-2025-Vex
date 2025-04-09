#pragma once

#include "liblvgl/lvgl.h"
#include <vector>
#include <string>
#include <functional>

// radio button component
static lv_style_t style_radio;
static lv_style_t style_radio_chk;

void set_styles()
{
    lv_style_init(&style_radio);
    lv_style_set_radius(&style_radio, LV_RADIUS_CIRCLE);

    lv_style_init(&style_radio_chk);
    lv_style_set_bg_img_src(&style_radio_chk, NULL);
}
namespace devils
{
    class Radio
    {
    public:
        Radio(lv_obj_t *parent, const char *label, lv_event_cb_t event_cb = nullptr, bool group = false)
        {
            // create radio button
            radio = lv_checkbox_create(parent);
            lv_checkbox_set_text(radio, label);
            if (event_cb != nullptr)
            {
                lv_obj_add_event_cb(radio, event_cb, LV_EVENT_VALUE_CHANGED, NULL);
            }
            if (group)
            {
                lv_obj_add_flag(radio, LV_OBJ_FLAG_EVENT_BUBBLE);
            }
            else set_styles();
            lv_obj_add_style(radio, &style_radio, LV_PART_INDICATOR);
            lv_obj_add_style(radio, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);
        }
        

    private:
        lv_obj_t *radio;
    };

    class RadioGroup
    {
    public:
        RadioGroup(lv_obj_t *parent, const std::vector<std::string> &labels, void (*event_cb)(std::string) = nullptr)
        {
            // create radio group
            radio_group = lv_obj_create(parent);
            this->event_cb = event_cb;
            lv_obj_set_layout(radio_group, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(radio_group, LV_FLEX_FLOW_COLUMN);

            set_styles();

            for (const auto &label : labels)
            {
                auto radio = Radio(radio_group, label.c_str(), nullptr, true);
            }
            // set the first radio button as checked
            lv_obj_t *first_radio = lv_obj_get_child(radio_group, 0);
            lv_obj_add_state(first_radio, LV_STATE_CHECKED); 

            lv_obj_add_event_cb(
                radio_group, 
                handleCheck,
                LV_EVENT_VALUE_CHANGED, 
                this
            );
        };

        lv_obj_t* getRadioGroup() { return radio_group; }

    private:
        lv_obj_t *radio_group;
        void (*event_cb)(std::string) = nullptr;
        uint32_t selected_index = 0;

        static void handleCheck(lv_event_t *e) {
            // get instance of the RadioGroup class
            RadioGroup *instance = static_cast<RadioGroup *>(lv_event_get_user_data(e));
            lv_obj_t * cont = lv_event_get_current_target(e);
            lv_obj_t * act_cb = lv_event_get_target(e);
            lv_obj_t * old_cb = lv_obj_get_child(cont, instance->selected_index);

            if (act_cb == cont) return;

            lv_obj_clear_state(old_cb, LV_STATE_CHECKED);   /*Uncheck the previous radio button*/
            lv_obj_add_state(act_cb, LV_STATE_CHECKED);     /*Uncheck the current radio button*/

            instance->selected_index = lv_obj_get_index(act_cb);

            lv_obj_t * label = lv_obj_get_child(act_cb, NULL);
            if (label != NULL)
            {
                const char *text = lv_label_get_text(label);
                std::string selected_routine = text;
                if (instance->event_cb != nullptr)
                {
                    std::cout << "Changing routine to: " << selected_routine << std::endl;
                    instance->event_cb(selected_routine); 
                }
            }
        }
    };
}
