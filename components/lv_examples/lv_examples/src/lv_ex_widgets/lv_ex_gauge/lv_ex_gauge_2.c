#include "../../../lv_examples.h"
#if LV_USE_GAUGE


void lv_ex_gauge_2(void)
{
    /*Describe the color for the needles*/
    static lv_color_t needle_colors[3];
    needle_colors[0] = LV_COLOR_BLUE;
    needle_colors[1] = LV_COLOR_ORANGE;
    needle_colors[2] = LV_COLOR_PURPLE;

    LV_IMG_DECLARE(img_hand);

    /*Create a gauge*/
    lv_obj_t * gauge1 = lv_gauge_create(lv_scr_act(), NULL);
    lv_gauge_set_needle_count(gauge1, 1, needle_colors);
    lv_obj_set_size(gauge1, 230, 230);
    lv_obj_align(gauge1, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_gauge_set_needle_img(gauge1, &img_hand, 4, 4);

    /*Set the values*/
 //   lv_gauge_set_value(gauge1, 0, 10);
 //   lv_gauge_set_value(gauge1, 1, 20);
    lv_gauge_set_value(gauge1, 0, 62);
}

#endif
