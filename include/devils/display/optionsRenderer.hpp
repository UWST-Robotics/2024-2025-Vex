#pragma once 

#include "liblvgl/lvgl.h"
#include "./components/radio.hpp"
#include "./eyesRenderer.hpp"


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
        OptionsRenderer(const char* bot_name, const std::vector<Routine>& routines, RobotAutoOptions *options) : Runnable(50)
        {
            OptionsRenderer::options = options;
            options->routine = routines[0];
            initializeRoot();
            createOptionsDisplayContainer(bot_name, routines);
        }

        ~OptionsRenderer()
        {
            lv_obj_del(root);
        }
    
    private:
        static lv_obj_t *root;

        void initializeRoot()
        {
            root = lv_obj_create(NULL);
            eyesRenderer = new EyesRenderer(root);
            lv_scr_load(root);
        }

        void createOptionsDisplayContainer(const char* bot_name, const std::vector<Routine>& routines)
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
            createRoutineContainer(options_display_container, routines);
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

        void createRoutineContainer(lv_obj_t *parent, const std::vector<Routine>& routines)
        {
            auto right_container = lv_obj_create(parent);
            lv_obj_set_size(right_container, lv_pct(70), lv_pct(100));
            lv_obj_set_layout(right_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(right_container, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(right_container, LV_OBJ_FLAG_SCROLLABLE);

            auto routine_container = lv_obj_create(right_container);
            lv_obj_set_size(routine_container, lv_pct(70), lv_pct(100));
            lv_obj_set_layout(routine_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(routine_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_border_width(routine_container, 0, 0);
            lv_obj_set_style_pad_all(routine_container, 0, 0);

            // title
            lv_obj_t *routine_title = lv_label_create(routine_container);
            lv_label_set_text(routine_title, "Select Routine");
            lv_obj_set_style_text_align(routine_title, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(routine_title, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_text_font(routine_title, &lv_font_montserrat_16, 0);

            std::vector<std::string> routine_names;
            for (const auto& routine : routines)
            {
                routine_names.push_back(routine.displayName);
            }

            RadioGroup *radio_group = new RadioGroup(routine_container, routine_names, handleRoutineChange);
            lv_obj_t *radio_group_obj = radio_group->getRadioGroup();
            lv_obj_set_width(radio_group_obj, lv_pct(100));
            lv_obj_set_flex_grow(radio_group_obj, 1);
            lv_obj_set_style_pad_all(radio_group_obj, 0, 0);
            lv_obj_set_style_border_width(radio_group_obj, 0, 0);

            auto screen_saver_toggle_container = lv_obj_create(right_container);
            lv_obj_set_size(screen_saver_toggle_container, lv_pct(30), lv_pct(100));
            lv_obj_set_layout(screen_saver_toggle_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(screen_saver_toggle_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(screen_saver_toggle_container, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
            lv_obj_set_style_pad_all(screen_saver_toggle_container, 0, 0);
            lv_obj_set_style_border_width(screen_saver_toggle_container, 0, 0);
            
            auto screen_saver_toggle_button = lv_btn_create(screen_saver_toggle_container);
            lv_obj_set_size(screen_saver_toggle_button, lv_pct(100), 50);
            auto screen_saver_toggle_label = lv_label_create(screen_saver_toggle_button);
            lv_label_set_text(screen_saver_toggle_label, "Save");  
            lv_obj_center(screen_saver_toggle_label);
            lv_obj_add_event_cb(screen_saver_toggle_button, handleScreenSaverToggle, LV_EVENT_CLICKED, NULL);      
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
            Routine new_routine;
            for (const auto& routine : routines)
            {
                if (routine.displayName == selected_routine)
                {
                    new_routine = routine;
                    break;
                }
            }
            options->routine = new_routine;
        }

        static void handleScreenSaverToggle(lv_event_t *e)
        {
            std::cout << options->routine.requiresAllianceColor << std::endl;
            std::cout << options->routine.displayName << std::endl;
            std::cout << "Screen Saver Toggled" << std::endl;
            if (options->routine.requiresAllianceColor && options->allianceColor == NONE_ALLIANCE)
            {
                // show error message
                lv_obj_t *message_box = lv_msgbox_create(root, "Error", "Selected routine can't be used when Alliance is set to 'None'.", {}, true);
                // center the message box
                // shadow
                lv_obj_set_style_shadow_color(message_box, lv_color_make(0, 0, 0), 0);
                lv_obj_set_style_shadow_width(message_box, 10, 0);
                lv_obj_set_style_shadow_opa(message_box, LV_OPA_COVER, 0);
                lv_obj_set_style_shadow_ofs_x(message_box, 4, 0);
                lv_obj_set_style_shadow_ofs_y(message_box, 4, 0);
                // border

                lv_obj_center(message_box);
                return;
            }
            eyesRenderer->render();
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
        static std::vector<Routine> routines;
        static EyesRenderer *eyesRenderer;
        static bool isScreenSaverEnabled;
    };

    RobotAutoOptions *OptionsRenderer::options = nullptr;
    EyesRenderer *OptionsRenderer::eyesRenderer = nullptr;
    bool OptionsRenderer::isScreenSaverEnabled = false;
    std::vector<Routine> OptionsRenderer::routines = {};
    lv_obj_t *OptionsRenderer::root = nullptr;
}