# ai_detect_demo：X5 BPU 人形检测 → DP 上报

> 在 switch_demo 底子上加算法封装：App DP1 控制算法启停，
> `tkl_npu_detect()` 同步推理吐 struct，聚合为 有人/无人(bool)+人数(value)
> 上报 DP101/DP102。

## 依赖

- 平台仓 `deepano-com/TuyaOpen-ubuntu@platform_ubuntu` 须包含 `tkl_npu`
  模块（`tuyaos_adapter/{include/npu,src/tkl_npu}`）；板端
  `platform/LINUX` 若仍是旧 commit，需 `git pull` 同步。
- 产品 `qhivvyqawogv04e4` 需添加自定义 DP（见下）。

## DP 映射（产品 `qhivvyqawogv04e4`）

| DP | 类型 | 方向 | 含义 | 状态 |
|----|------|------|------|------|
| 1 | bool | App→设备→echo | 算法启停开关（复用开关 DP） | 产品自带 ✅ |
| 101 | bool | 设备→App | 有人(true)/无人(false) | **需平台添加** |
| 102 | value | 设备→App | 人数 0~100 | **需平台添加** |

平台添加步骤（IoT 平台 → 该产品 → 功能定义 → 自定义功能 → 添加）：
- `101 / 有人状态 / 布尔型`；`102 / 人数 / 数值型（0~100，步长1）`。
- 加完重启设备即生效（schema 随连接同步，无需重新激活）。

## 文件

- `src/ai_detect.h/.c`：封装层。`ai_detect_start/stop` 启停 worker 线程；
  模型常驻（start 时 `tkl_npu_load`），每 10s 读一帧 NV12 调 `tkl_npu_detect`，
  struct 聚合为有人/无人+人数，变化上报。
- `src/tuya_main.c`：switch_demo + 3 个钩子（include / DP1 下发启停 / 开机自启）。
- `src/tuya_config.h`：产品 PID（密钥在板端 `tuya_config_secrets.h`，
  被 `.gitignore` 排除，**永不进仓**）。
- `src/cli_cmd.c`、`src/reset_netcfg.*`：与 switch_demo 同源的原厂文件。
- `tools/ai_worker.py`：官方 BPU 脚本副本，仅作对拍基准与离线 NV12 生成，
  运行时不再调用（原厂 `/app` 文件未动）。
- 板端数据（`/app/dipa/data/`）：`ai_person.nv12`（kite 672x672，有人）、
  `ai_empty.nv12`（纯黑 672x672，无人）。由下述命令预生成。

无摄像头阶段轮换两帧 NV12 模拟 有人↔无人；有摄像头后把读文件换成抓帧即可，
`ai_detect_*` 接口与 DP 逻辑不变。

## NV12 测试帧生成（板端，一次性）

```bash
cp /app/pydev_demo/02_detection_sample/01_ultralytics_yolov5x/kite.jpg /app/dipa/data/ai_person.jpg
python3 -c "import cv2,numpy as np;cv2.imwrite('/app/dipa/data/ai_empty.jpg',np.zeros((720,1280,3),np.uint8))"
python3 - <<'EOF'
import sys; sys.path.append('/app/pydev_demo')
import cv2, numpy as np, utils.preprocess_utils as pre
for src, dst in [('/app/dipa/data/ai_person.jpg', '/app/dipa/data/ai_person.nv12'),
                 ('/app/dipa/data/ai_empty.jpg', '/app/dipa/data/ai_empty.nv12')]:
    img = cv2.imread(src)
    rs = pre.resized_image(img, 672, 672, 0)
    y, uv = pre.bgr_to_nv12_planes(rs)
    np.concatenate((y.reshape(-1).astype(np.uint8),
                    uv.reshape(-1).astype(np.uint8))).tofile(dst)
    print(dst, 'orig=%dx%d' % (img.shape[1], img.shape[0]))
EOF
```

## 板端构建运行

```bash
# 密钥（0600，仅板端，不进仓）：从 switch_demo 复制或按 example 填写
cp ../switch_demo/src/tuya_config_secrets.h src/ && chmod 600 src/tuya_config_secrets.h

cd apps/tuya_cloud/ai_detect_demo && source /app/dipa/tuyaopen/export.sh
tos.py config choice -c Linux.config
printf 'D\n' | tos.py build

# 运行：tos.py build 会清空 dist/ 连带删掉 tuyadb 激活数据，
# 所以 elf 拷贝到构建碰不到的稳定目录跑，tuyadb 常驻于此
mkdir -p /app/dipa/data/ai_run
cp dist/ai_detect_demo_1.0.0/ai_detect_demo_1.0.0.elf /app/dipa/data/ai_run/
cd /app/dipa/data/ai_run
setsid -f ./ai_detect_demo_1.0.0.elf </dev/null >>/app/dipa/logs/ai-detect-run.log 2>&1

# 首次 App 扫码绑定成功后备份激活库（先停进程）：
# cp -r /app/dipa/data/ai_run/tuyadb /app/dipa/data/tuyadb.activated.bak
```

## App 验证

1. 设备上线后拨开关 ON → 日志 `[AI] App cmd: START`，上报交替
   `{"101":true,"102":11}` / `{"101":false,"102":0}`。
2. 拨开关 OFF → 日志 `[AI] App cmd: STOP` + 上报 `{"101":false,"102":0}`，
   推理停止；再拨 ON 恢复。
3. 看 struct 明细：`grep -a "\[AI\]" /app/dipa/logs/ai-detect-run*.log`
   （DEBUG 级逐框打印）。
