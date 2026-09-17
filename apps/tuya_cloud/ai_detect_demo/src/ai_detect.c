/**
 * @file ai_detect.c
 * @brief X5 BPU 人形检测算法封装实现(TKL-NPU 直调版)
 *
 * worker 线程周期读 NV12 测试帧 -> tkl_npu_detect() 同步推理 ->
 * struct 结果聚合为 有人/无人+人数,变化时上报 DP101/DP102。
 *
 * 板端无摄像头,故轮换两帧预生成的 NV12 模拟 有人<->无人 跳变:
 *   /app/dipa/data/ai_person.nv12 (kite.jpg 1352x900,有 11 人)
 *   /app/dipa/data/ai_empty.nv12  (纯黑 1280x720,0 目标)
 * 有摄像头后把读文件换成抓帧即可,接口与 DP 逻辑不变。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tal_api.h"
#include "tuya_iot.h"
#include "tkl_npu.h"

#include "ai_detect.h"

#define AI_MODEL_PATH "/opt/hobot/model/x5/basic/yolov5s_672x672_nv12.bin"
#define AI_INTERVAL_MS 10000
#define AI_LOG_BOX_MAX 8

typedef struct {
    const char *path;
    int         ori_w, ori_h;
} ai_frame_t;

static const ai_frame_t ai_frames[] = {
    {"/app/dipa/data/ai_person.nv12", 1352, 900},
    {"/app/dipa/data/ai_empty.nv12", 1280, 720},
};

static THREAD_HANDLE s_thread   = NULL;
static MUTEX_HANDLE  s_mutex    = NULL;
static volatile bool s_running  = false;
static volatile bool s_exited   = true;
static ai_result_t   s_latest   = {0};
static int           s_last_hp  = -1; /* -1=未知,强制首次上报 */
static int           s_last_cnt = -1;

static TKL_NPU_HANDLE s_npu     = NULL;
static uint8_t       *s_nv12    = NULL;
static int            s_model_w = 0, s_model_h = 0;

int ai_detect_init(void)
{
    if (s_mutex != NULL) {
        return OPRT_OK;
    }
    if (OPRT_OK != tal_mutex_create_init(&s_mutex)) {
        PR_ERR("[AI] mutex init failed");
        return OPRT_COM_ERROR;
    }
    memset(&s_latest, 0, sizeof(s_latest));
    return OPRT_OK;
}

bool ai_detect_is_running(void)
{
    return s_running;
}

int ai_detect_get_latest(ai_result_t *out)
{
    if (NULL == out) {
        return OPRT_INVALID_PARM;
    }
    tal_mutex_lock(s_mutex);
    *out = s_latest;
    tal_mutex_unlock(s_mutex);
    return OPRT_OK;
}

/* 上报 DP101(有人)+DP102(人数),拼成一个 JSON 一次上报 */
static void ai_report_dp(bool has_person, int count)
{
    char json[64];
    snprintf(json, sizeof(json), "{\"%d\": %s, \"%d\": %d}", AI_DP_HAS_PERSON, has_person ? "true" : "false",
             AI_DP_PERSON_COUNT, count);
    OPERATE_RET rt = tuya_iot_dp_report_json(tuya_iot_client_get(), json);
    PR_INFO("[AI] dp report %s -> %d", json, rt);
}

/* 跑一帧 TKL 推理;成功返回目标数,失败返回 -1 */
static int ai_run_once(const ai_frame_t *frame, ai_result_t *res)
{
    size_t expect = (size_t)s_model_w * (size_t)s_model_h * 3 / 2;
    FILE  *fp     = fopen(frame->path, "rb");
    if (NULL == fp) {
        PR_ERR("[AI] open %s failed", frame->path);
        return -1;
    }
    size_t nread = fread(s_nv12, 1, expect, fp);
    fclose(fp);
    if (nread != expect) {
        PR_ERR("[AI] %s short read %u/%u", frame->path, (unsigned)nread, (unsigned)expect);
        return -1;
    }

    TKL_NPU_IMG_T img = {
        .nv12    = s_nv12,
        .model_w = s_model_w,
        .model_h = s_model_h,
        .ori_w   = frame->ori_w,
        .ori_h   = frame->ori_h,
    };
    TKL_NPU_OBJ_T tobjs[TKL_NPU_MAX_OBJ];
    int           tnum = TKL_NPU_MAX_OBJ;
    if (OPRT_OK != tkl_npu_detect(s_npu, &img, tobjs, &tnum)) {
        PR_ERR("[AI] tkl_npu_detect failed");
        return -1;
    }

    memset(res, 0, sizeof(*res));
    res->obj_count = tnum < AI_MAX_OBJS ? tnum : AI_MAX_OBJS;
    for (int i = 0; i < res->obj_count; i++) {
        res->objs[i].x1 = (int)tobjs[i].x1;
        res->objs[i].y1 = (int)tobjs[i].y1;
        res->objs[i].x2 = (int)tobjs[i].x2;
        res->objs[i].y2 = (int)tobjs[i].y2;
        strncpy(res->objs[i].label, tobjs[i].label, sizeof(res->objs[i].label) - 1);
        res->objs[i].score = tobjs[i].score;
        if (0 == strcmp(res->objs[i].label, "person")) {
            res->person_count++;
        }
    }
    res->has_person = (res->person_count > 0);

    PR_INFO("[AI] img=%s objs=%d persons=%d", frame->path, res->obj_count, res->person_count);
    for (int i = 0; i < res->obj_count && i < AI_LOG_BOX_MAX; i++) {
        PR_DEBUG("[AI]   box x1=%d y1=%d x2=%d y2=%d label=%s score=%.2f", res->objs[i].x1, res->objs[i].y1,
                 res->objs[i].x2, res->objs[i].y2, res->objs[i].label, res->objs[i].score);
    }
    if (res->obj_count > AI_LOG_BOX_MAX) {
        PR_DEBUG("[AI]   ... +%d more", res->obj_count - AI_LOG_BOX_MAX);
    }
    return res->obj_count;
}

static void ai_worker_thread(void *arg)
{
    (void)arg;
    unsigned int round = 0;
    unsigned int nframes = sizeof(ai_frames) / sizeof(ai_frames[0]);

    PR_INFO("[AI] worker thread enter (TKL-NPU direct)");
    while (s_running) {
        /* 无摄像头:轮换测试帧模拟有人/无人跳变 */
        const ai_frame_t *frame = &ai_frames[round % nframes];

        ai_result_t res;
        if (ai_run_once(frame, &res) >= 0) {
            tal_mutex_lock(s_mutex);
            s_latest = res;
            tal_mutex_unlock(s_mutex);

            if ((int)res.has_person != s_last_hp || res.person_count != s_last_cnt) {
                ai_report_dp(res.has_person, res.person_count);
                s_last_hp  = (int)res.has_person;
                s_last_cnt = res.person_count;
            }
        }

        round++;
        for (int slept = 0; slept < AI_INTERVAL_MS && s_running; slept += 500) {
            tal_system_sleep(500);
        }
    }

    /* 停止后清一次云端状态 */
    ai_report_dp(false, 0);
    s_last_hp  = 0;
    s_last_cnt = 0;

    PR_INFO("[AI] worker thread exit");
    s_exited = true;
}

int ai_detect_start(void)
{
    if (s_running) {
        PR_INFO("[AI] already running");
        return OPRT_OK;
    }
    if (NULL == s_mutex && OPRT_OK != ai_detect_init()) {
        return OPRT_COM_ERROR;
    }

    /* 模型常驻:启动时加载,停止时卸载 */
    if (NULL == s_npu) {
        if (OPRT_OK != tkl_npu_load(AI_MODEL_PATH, &s_npu) || NULL == s_npu) {
            PR_ERR("[AI] tkl_npu_load failed");
            return OPRT_COM_ERROR;
        }
        tkl_npu_get_input_spec(s_npu, &s_model_w, &s_model_h);
        size_t nbytes = (size_t)s_model_w * (size_t)s_model_h * 3 / 2;
        s_nv12        = (uint8_t *)tal_malloc(nbytes);
        if (NULL == s_nv12) {
            tkl_npu_unload(s_npu);
            s_npu = NULL;
            return OPRT_MALLOC_FAILED;
        }
        PR_INFO("[AI] npu loaded: %dx%d", s_model_w, s_model_h);
    }

    s_running = true;
    s_exited  = false;
    s_last_hp = -1; /* 强制首轮上报 */
    s_last_cnt = -1;

    THREAD_CFG_T cfg = {
        .stackDepth = 1024 * 8,
        .priority   = THREAD_PRIO_1,
        .thrdname   = "ai_detect",
    };
    if (OPRT_OK != tal_thread_create_and_start(&s_thread, NULL, NULL, ai_worker_thread, NULL, &cfg)) {
        s_running = false;
        s_exited  = true;
        PR_ERR("[AI] thread create failed");
        return OPRT_COM_ERROR;
    }
    PR_INFO("[AI] started");
    return OPRT_OK;
}

int ai_detect_stop(void)
{
    if (!s_running) {
        PR_INFO("[AI] already stopped");
        return OPRT_OK;
    }

    s_running = false;

    /* 等线程退出(推理仅约 60ms,无需杀进程) */
    int waited = 0;
    while (!s_exited && waited < 10000) {
        tal_system_sleep(100);
        waited += 100;
    }
    if (s_thread != NULL) {
        tal_thread_delete(s_thread);
        s_thread = NULL;
    }

    /* 卸载模型,释放资源 */
    if (s_npu != NULL) {
        tkl_npu_unload(s_npu);
        s_npu = NULL;
    }
    if (s_nv12 != NULL) {
        tal_free(s_nv12);
        s_nv12 = NULL;
    }

    PR_INFO("[AI] stopped (exited=%d)", (int)s_exited);
    return OPRT_OK;
}
