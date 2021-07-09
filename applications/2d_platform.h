#ifndef __2D_PLATFORM_H__
#define __2D_PLATFORM_H__

#ifdef __cplusplus
extern "C" {
#endif
enum move_result {
	MOVE_OK,
	MOVE_ERROR,
	MOVE_OUT_RANGE,
	MOVE_OVERLOAD
};

enum move_result 2d_platform_posit(int x, y);
enum move_result z_axis_posit(int p);

#ifdef __cplusplus
}
#endif

#endif
