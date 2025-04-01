#pragma once 

#include "liblvgl/lvgl.h"
#include "./components/radio.hpp"


namespace devils
{
    const std::map<AllianceColor, lv_color_t> color_map = {
        {RED_ALLIANCE, lv_color_make(255, 0, 0)},
        {BLUE_ALLIANCE, lv_color_make(0, 0, 255)},
        {NONE_ALLIANCE, lv_color_make(124, 124, 124)}
    };   
    const std::map<AllianceColor, std::string> color_name_map = {
        {RED_ALLIANCE, "Red"},
        {BLUE_ALLIANCE, "Blue"},
        {NONE_ALLIANCE, "None"}
    };
        
    class OptionsRenderer : Runnable
    {
    public:
        OptionsRenderer(const char* bot_name, const std::vector<std::string>& routine_names, RobotAutoOptions *options) : Runnable(50)
        {
            OptionsRenderer::options = options;
            initializeRoot();
            createOptionsDisplayContainer(bot_name, routine_names);
        }

        ~OptionsRenderer()
        {
            lv_obj_del(root);
        }
    
    private:
        lv_obj_t *root;

        void initializeRoot()
        {
            root = lv_obj_create(NULL);
            lv_scr_load(root);
        }

        void createOptionsDisplayContainer(const char* bot_name, const std::vector<std::string>& routine_names)
        {
            auto fullscreen_container = lv_obj_create(root);
            lv_obj_set_size(fullscreen_container, lv_pct(100), lv_pct(100));
            lv_obj_set_layout(fullscreen_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(fullscreen_container, LV_FLEX_FLOW_COLUMN);
            // center
            lv_obj_set_flex_align(fullscreen_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

            lv_obj_t *title = lv_label_create(fullscreen_container);
            lv_label_set_text(title, bot_name);
            lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(title, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_text_font(title, &lv_font_unscii_16, 0);

            auto options_display_container = lv_obj_create(fullscreen_container);
            lv_obj_set_style_pad_all(options_display_container, 0, 0);
            lv_obj_set_flex_grow(options_display_container, 1);
            lv_obj_set_style_border_width(options_display_container, 0, 0);
            lv_obj_set_width(options_display_container, lv_pct(100));
            lv_obj_clear_flag(options_display_container, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_layout(options_display_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(options_display_container, LV_FLEX_FLOW_ROW);
            lv_obj_set_style_pad_row(options_display_container, 0, 0);

            createAllianceColorContainer(options_display_container);
            createRoutineContainer(options_display_container, routine_names);
        }

        void createAllianceColorContainer(lv_obj_t *parent)
        {
            auto alliance_color_container = lv_obj_create(parent);
            lv_obj_set_size(alliance_color_container, lv_pct(30), lv_pct(100));
            lv_obj_set_layout(alliance_color_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(alliance_color_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(alliance_color_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

            lv_obj_t *alliance_title = lv_label_create(alliance_color_container);
            lv_label_set_text(alliance_title, "Alliance");
            lv_obj_set_style_text_align(alliance_title, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(alliance_title, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_text_font(alliance_title, &lv_font_montserrat_16, 0);

            lv_obj_t *alliance_color_button = lv_btn_create(alliance_color_container);
            lv_obj_set_size(alliance_color_button, lv_pct(100), 100);
            lv_obj_set_flex_grow(alliance_color_button, 1);
            lv_obj_set_style_bg_color(alliance_color_button, color_map.at(options->allianceColor), 0);
            lv_obj_add_event_cb(alliance_color_button, handleAllianceColorChange, LV_EVENT_CLICKED, NULL);

            lv_obj_t *alliance_color_label = lv_label_create(alliance_color_button);
            lv_label_set_text(alliance_color_label, color_name_map.at(options->allianceColor).c_str());
            lv_obj_center(alliance_color_label);
        }

        void createRoutineContainer(lv_obj_t *parent, const std::vector<std::string>& routine_names)
        {
            auto routine_container = lv_obj_create(parent);
            lv_obj_set_size(routine_container, lv_pct(70), lv_pct(100));
            lv_obj_set_layout(routine_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(routine_container, LV_FLEX_FLOW_COLUMN);

            // title
            lv_obj_t *routine_title = lv_label_create(routine_container);
            lv_label_set_text(routine_title, "Select Routine");
            lv_obj_set_style_text_align(routine_title, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(routine_title, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_text_font(routine_title, &lv_font_montserrat_16, 0);


            RadioGroup *radio_group = new RadioGroup(routine_container, routine_names, options->routine);
            lv_obj_t *radio_group_obj = radio_group->getRadioGroup();
            lv_obj_set_width(radio_group_obj, lv_pct(100));
            lv_obj_set_flex_grow(radio_group_obj, 1);
            lv_obj_set_style_pad_all(radio_group_obj, 0, 0);
            lv_obj_set_style_border_width(radio_group_obj, 0, 0);
        
        }

        void addButton(lv_obj_t *container, const char *label, lv_color_t color = lv_color_make(255, 0, 0), lv_event_cb_t event_cb = nullptr)
        {
            lv_obj_t *btn = lv_btn_create(container);
            if (event_cb != nullptr)
            {
                lv_obj_add_event_cb(btn, event_cb, LV_EVENT_CLICKED, NULL);
            }

            lv_obj_set_style_bg_color(btn, color, 0);

            lv_obj_t *label_obj = lv_label_create(btn);
            lv_label_set_text(label_obj, label);
            lv_obj_center(label_obj);
        }

        static void handleAllianceColorChange(lv_event_t *e)
        {
            lv_obj_t *btn = lv_event_get_target(e);
            lv_obj_t *label = lv_obj_get_child(btn, NULL);
            if (label != NULL)
            {
                const char *text = lv_label_get_text(label);
                AllianceColor current_color = getCurrentAllianceColor(text);

                auto next_color = getNextAllianceColor(current_color);
                lv_obj_set_style_bg_color(btn, color_map.at(next_color), 0);
                lv_label_set_text(label, color_name_map.at(next_color).c_str());
                options->allianceColor = next_color;
            }
        }

        static void handleRoutineChange(std::string selected_routine)
        {
            options->routine = selected_routine;
        }

    
        
        static AllianceColor getCurrentAllianceColor(const char *text)
        {
            for (const auto& pair : color_name_map)
            {
                if (pair.second == text)
                {
                    return pair.first;
                }
            }
            return NONE_ALLIANCE;
        }

        static AllianceColor getNextAllianceColor(AllianceColor current_color)
        {
            auto it = color_map.find(current_color);
            auto next_it = std::next(it);
            if (next_it == color_map.end())
            {
                next_it = color_map.begin();
            }
            return next_it->first;
        }

        static RobotAutoOptions *options;
    };

    RobotAutoOptions *OptionsRenderer::options = nullptr;
}