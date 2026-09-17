/**
 * @file ai_detect.h
 * @brief X5 BPU 人形检测算法封装:启动/停止/结果吐出 + DP 上报
 *
 * DP 映射(产品 qhivvyqawogv04e4):
 *   DP 1   (bool, 开关):  App 下发 = 算法启停;设备 echo 确认
 *   DP 101 (bool, 自定义): 设备上报 = 有人(true)/无人(false),需平台添加
 *   DP 102 (value,自定义): 设备上报 = 人数,需平台添加
 */

#ifndef AI_DETECT_H_
#define AI_DETECT_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AI_DP_ALGO_SWITCH  1
#define AI_DP_HAS_PERSON   101
#define AI_DP_PERSON_COUNT 102

#define AI_MAX_OBJS  64
#define AI_LABEL_LEN 32

/* 单个目标:对应算法吐出的 struct */
typedef struct {
    int   x1, y1, x2, y2;
    char  label[AI_LABEL_LEN];
    float score;
} ai_obj_t;

/* 一帧的完整结果 */
typedef struct {
    ai_obj_t objs[AI_MAX_OBJS];
    int      obj_count;    /* 全部目标数 */
    int      person_count; /* person 类目标数 */
    bool     has_person;   /* person_count > 0 */
} ai_result_t;

/* 初始化(调一次) */
int ai_detect_init(void);

/* 算法启动:幂等,已在运行则直接返回成功 */
int ai_detect_start(void);

/* 算法关闭:幂等;停止后上报一次 有人=false/人数=0 清云端状态 */
int ai_detect_stop(void);

/* 运行状态查询 */
bool ai_detect_is_running(void);

/* 取最新一帧结果(线程安全,供未来 CLI/联动用) */
int ai_detect_get_latest(ai_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* AI_DETECT_H_ */
