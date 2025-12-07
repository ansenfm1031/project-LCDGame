#ifndef __HEADERS_H__
#define __HEADERS_H__

// typedef
typedef enum
{
	SCENE_CLEAR,
	SCENE_MENU,
	SCENE_WAITING,
	SCENE_SHOOTING,
	SCENE_RESULT,
} E_SCENE_INDEX;

typedef struct
{
	int8_t	x;
	int8_t	y;
} POS;

typedef struct
{
	POS		pos;
	uint8_t	dir;
} OBJECT;

// define
#define SERIAL_DEBUG_MODE	1

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
//#define abs(a) ((a) < 0 ? -(a) : (a))

#define MAP_SIZE	160

#define LCD_MIN_X	0
#define LCD_MAX_X	80
#define LCD_MIN_Y	0
#define LCD_MAX_Y	16

#define PLAYABLE_MIN_X	36
#define PLAYABLE_MAX_X	43
#define PLAYABLE_MIN_Y	1
#define PLAYABLE_MAX_Y	14

#define DIR_UP			0x00
#define DIR_RIGHT		0x01
#define DIR_DOWN		0x03
#define DIR_LEFT		0x02
#define DIR_RUNNING		0x04

#define MENU_COUNT	5

#define BULLET_MAX_COUNT	16
#define WALL_MAX_COUNT		8

// global
uint8_t map[MAP_SIZE];			// 5 * 8 * 16 * 2 pixel => 8 bit * 160
uint8_t display_index[32];		// custom character index (uint8_t) for 16 * 2 character
uint32_t last_display_index;	// check displayed before (bit) for 16 * 2 character

uint8_t		g_flag;
uint16_t	g_tick;

E_SCENE_INDEX g_prv_scene = SCENE_CLEAR;
E_SCENE_INDEX g_cur_scene = SCENE_CLEAR;
E_SCENE_INDEX g_nxt_scene = SCENE_CLEAR;

int8_t	g_input_x;
int8_t	g_input_y;
uint8_t	g_input_sw;

#endif // __HEADERS_H__
