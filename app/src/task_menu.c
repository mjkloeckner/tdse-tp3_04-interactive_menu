/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   : task_menu.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes */
#include "main.h"

/* Demo includes */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes */
#include "board.h"
#include "app.h"
#include "task_menu_attribute.h"
#include "task_menu_interface.h"
#include "display.h"

/********************** macros and definitions *******************************/
#define G_TASK_MEN_CNT_INI          0ul
#define G_TASK_MEN_TICK_CNT_INI     0ul

#define DEL_MEN_XX_MIN              0ul
#define DEL_MEN_XX_MED             50ul
#define DEL_MEN_XX_MAX            200ul

/********************** internal data declaration ****************************/
task_menu_dta_t task_menu_dta =
	{DEL_MEN_XX_MIN, ST_MEN_MOTOR_SELECT, EV_MEN_ENT_IDLE, false};

#define MENU_DTA_QTY	(sizeof(task_menu_dta)/sizeof(task_menu_dta_t))

typedef struct {
    uint8_t power, speed, spin;
} motor_data_t;

/********************** internal functions declaration ***********************/
void task_menu_statechart(void);

/********************** internal data definition *****************************/
const char *p_task_menu = "Task Menu (Interactive Menu)";
const char *p_task_menu_ = "Non-Blocking & Update By Time Code";
bool g_display_update = false;

#define MOTOR_QTY 2

motor_data_t g_motor_data[MOTOR_QTY];

uint8_t g_selected_motor = 0;
uint8_t g_new_sel_motor_spin;
uint8_t g_new_sel_motor_power;
uint8_t g_new_sel_motor_speed;
task_menu_st_t g_menu_prev_state;

/********************** external data declaration ****************************/
uint32_t g_task_menu_cnt;
volatile uint32_t g_task_menu_tick_cnt;

/********************** external functions definition ************************/
void task_menu_init(void *parameters)
{
	task_menu_dta_t *p_task_menu_dta;
	task_menu_st_t	state;
	task_menu_ev_t	event;
	bool b_event;

	/* Print out: Task Initialized */
	LOGGER_INFO(" ");
	LOGGER_INFO("  %s is running - %s", GET_NAME(task_menu_init), p_task_menu);
	LOGGER_INFO("  %s is a %s", GET_NAME(task_menu), p_task_menu_);

	/* Init & Print out: Task execution counter */
	g_task_menu_cnt = G_TASK_MEN_CNT_INI;
	LOGGER_INFO("   %s = %lu", GET_NAME(g_task_menu_cnt), g_task_menu_cnt);

	init_queue_event_task_menu();

	/* Update Task Actuator Configuration & Data Pointer */
	p_task_menu_dta = &task_menu_dta;

	/* Init & Print out: Task execution FSM */
	state = ST_MEN_MOTOR_SELECT;
	p_task_menu_dta->state = state;

    g_menu_prev_state = state;
    g_selected_motor = 1;

	event = EV_MEN_ENT_IDLE;
	p_task_menu_dta->event = event;

	b_event = false;
	p_task_menu_dta->flag = b_event;

	/* Init & Print out: LCD Display */
	displayInit(DISPLAY_CONNECTION_GPIO_4BITS);
    g_display_update = true;
}

void task_menu_update(void *parameters)
{
    bool b_time_update_required = false;

    /* Protect shared resource */
    __asm("CPSID i");	/* disable interrupts */
    if (G_TASK_MEN_TICK_CNT_INI < g_task_menu_tick_cnt)
    {
        /* Update Tick Counter */
        g_task_menu_tick_cnt--;
        b_time_update_required = true;
    }
    __asm("CPSIE i");	/* enable interrupts */

    while (b_time_update_required)
    {
        /* Update Task Counter */
        g_task_menu_cnt++;

        /* Run Task Menu Statechart */
        task_menu_statechart();

        /* Protect shared resource */
        __asm("CPSID i");	/* disable interrupts */
        if (G_TASK_MEN_TICK_CNT_INI < g_task_menu_tick_cnt)
        {
            /* Update Tick Counter */
            g_task_menu_tick_cnt--;
            b_time_update_required = true;
        }
        else
        {
            b_time_update_required = false;
        }
        __asm("CPSIE i");	/* enable interrupts */
    }
}

void menu_set_state(task_menu_st_t new_state)
{
    task_menu_dta_t *p_task_menu_dta;
    p_task_menu_dta = &task_menu_dta;

    g_menu_prev_state = p_task_menu_dta->state;
    p_task_menu_dta->state = new_state;

    g_display_update = true;
}

void task_menu_statechart(void)
{
    task_menu_dta_t *p_task_menu_dta;
    char menu_row_str[16 + 4];

    /* Update Task Menu Data Pointer */
    p_task_menu_dta = &task_menu_dta;

    if (true == any_event_task_menu())
    {
        p_task_menu_dta->flag = true;
        p_task_menu_dta->event = get_event_task_menu();
    }

    switch (p_task_menu_dta->state)
    {
        case ST_MEN_MOTOR_SELECT:

            if (g_display_update)
            {
                g_display_update = false;
                displayCharPositionWrite(0, 0);
                sprintf(menu_row_str, "> Motor %d     ", g_selected_motor);
                displayStringWrite(menu_row_str);
            }

            if (true == p_task_menu_dta->flag)
            {
                p_task_menu_dta->flag = false;
                switch (p_task_menu_dta->event)
                {
                    case EV_MEN_ENT_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_POWER);
                    break;
                    case EV_MEN_NEX_ACTIVE:
                        g_selected_motor = (g_selected_motor % MOTOR_QTY) + 1;
                        g_display_update = true;
                    break;
                    case EV_MEN_ESC_ACTIVE:
                    break;
                    default:
                    break;
                }
            }

        break;

        case ST_MEN_MOTOR_POWER:

            if (g_display_update)
            {
                g_display_update = false;
                displayCharPositionWrite(0, 0);
                sprintf(menu_row_str, "%1d> Power        ", g_selected_motor);
                displayStringWrite(menu_row_str);
            }

            if (true == p_task_menu_dta->flag)
            {
                p_task_menu_dta->flag = false;
                switch (p_task_menu_dta->event)
                {
                    case EV_MEN_ENT_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_POWER_SEL);
                    break;
                    case EV_MEN_NEX_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_SPEED);
                    break;
                    case EV_MEN_ESC_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_SELECT);
                    break;
                    default:
                    break;
                }
            }

        break;

        case ST_MEN_MOTOR_POWER_SEL:

            if (g_menu_prev_state != p_task_menu_dta->state)
            {
                g_menu_prev_state = p_task_menu_dta->state;
                g_new_sel_motor_power = g_motor_data[g_selected_motor - 1].power;
            }

            if (g_display_update)
            {
                g_display_update = false;
                displayCharPositionWrite(0, 0);
                sprintf(menu_row_str, "%1d> Power    %3d%%", g_selected_motor,
                        g_new_sel_motor_power);
                displayStringWrite(menu_row_str);
            }

            if (true == p_task_menu_dta->flag)
            {
                p_task_menu_dta->flag = false;
                switch (p_task_menu_dta->event)
                {
                    case EV_MEN_ENT_ACTIVE:
                        g_motor_data[g_selected_motor - 1].power = g_new_sel_motor_power;
                        menu_set_state(ST_MEN_MOTOR_POWER);
                    break;
                    case EV_MEN_NEX_ACTIVE:
                        g_new_sel_motor_power = ((g_new_sel_motor_power + 10) % 110);
                        g_display_update = true;
                    break;
                    case EV_MEN_ESC_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_POWER);
                    break;
                    default:
                    break;
                }
            }

        break;

        case ST_MEN_MOTOR_SPEED:

            if (g_display_update)
            {
                g_display_update = false;
                displayCharPositionWrite(0, 0);
                sprintf(menu_row_str, "%1d> Speed        ", g_selected_motor);
                displayStringWrite(menu_row_str);
            }

            if (true == p_task_menu_dta->flag)
            {
                p_task_menu_dta->flag = false;
                switch (p_task_menu_dta->event)
                {
                    case EV_MEN_NEX_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_SPIN);
                    break;
                    case EV_MEN_ESC_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_SELECT);
                    break;
                    default:
                    break;
                }
            }

        break;

        case ST_MEN_MOTOR_SPIN:

            if (g_display_update)
            {
                g_display_update = false;
                displayCharPositionWrite(0, 0);
                sprintf(menu_row_str, "%1d> Spin         ", g_selected_motor);
                displayStringWrite(menu_row_str);
            }

            if (true == p_task_menu_dta->flag)
            {
                p_task_menu_dta->flag = false;
                switch (p_task_menu_dta->event)
                {
                    case EV_MEN_NEX_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_POWER);
                    break;

                    case EV_MEN_ESC_ACTIVE:
                        menu_set_state(ST_MEN_MOTOR_SELECT);
                    break;
                    default:
                    break;
                }
            }

        break;

        default:

            p_task_menu_dta->tick  = DEL_MEN_XX_MIN;
            p_task_menu_dta->state = ST_MEN_MOTOR_SELECT;
            p_task_menu_dta->event = EV_MEN_ENT_IDLE;
            p_task_menu_dta->flag  = false;
            g_display_update = false;

            break;
    }
}

/********************** end of file ******************************************/
