//
// Created by RM UI Designer
//

#include "ui_g_sentry_5.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 5
#define OBJ_NUM 3
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_g_sentry_5;
ui_interface_round_t *ui_g_sentry_minipc_state = (ui_interface_round_t *)&(ui_g_sentry_5.data[0]);
ui_interface_round_t *ui_g_sentry_running = (ui_interface_round_t *)&(ui_g_sentry_5.data[1]);
ui_interface_number_t *ui_g_sentry_supercap_v = (ui_interface_number_t *)&(ui_g_sentry_5.data[2]);

void _ui_init_g_sentry_5() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_sentry_5.data[i].figure_name[0] = FRAME_ID;
        ui_g_sentry_5.data[i].figure_name[1] = GROUP_ID;
        ui_g_sentry_5.data[i].figure_name[2] = i + START_ID;
        ui_g_sentry_5.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_g_sentry_5.data[i].operate_tpyel = 0;
    }

    ui_g_sentry_minipc_state->figure_tpye = 2;
    ui_g_sentry_minipc_state->layer = 0;
    ui_g_sentry_minipc_state->r = 23;
    ui_g_sentry_minipc_state->start_x = 1828;
    ui_g_sentry_minipc_state->start_y = 323;
    ui_g_sentry_minipc_state->color = 5;
    ui_g_sentry_minipc_state->width = 10;

    ui_g_sentry_running->figure_tpye = 2;
    ui_g_sentry_running->layer = 0;
    ui_g_sentry_running->r = 10;
    ui_g_sentry_running->start_x = 1682;
    ui_g_sentry_running->start_y = 628;
    ui_g_sentry_running->color = 2;
    ui_g_sentry_running->width = 10;

    ui_g_sentry_supercap_v->figure_tpye = 5;
    ui_g_sentry_supercap_v->layer = 0;
    ui_g_sentry_supercap_v->font_size = 20;
    ui_g_sentry_supercap_v->start_x = 66;
    ui_g_sentry_supercap_v->start_y = 670;
    ui_g_sentry_supercap_v->color = 3;
    ui_g_sentry_supercap_v->number = 12345;
    ui_g_sentry_supercap_v->width = 2;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_sentry_5);
    SEND_MESSAGE((uint8_t *) &ui_g_sentry_5, sizeof(ui_g_sentry_5));
}

void _ui_update_g_sentry_5() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_sentry_5.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_sentry_5);
    SEND_MESSAGE((uint8_t *) &ui_g_sentry_5, sizeof(ui_g_sentry_5));
}

void _ui_remove_g_sentry_5() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_sentry_5.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_sentry_5);
    SEND_MESSAGE((uint8_t *) &ui_g_sentry_5, sizeof(ui_g_sentry_5));
}
