# 闲时自动重启功能

## 功能说明

在每天凌晨3点，系统会自动检查是否处于闲时状态，如果是则自动重启PC。

## 闲时定义

- 相机已关闭超过5分钟
- 没有新的相机启动命令

## 执行条件

所有以下条件必须同时满足：

1. ✅ 时间为凌晨3点
2. ✅ 系统正在运行
3. ✅ 相机已关闭
4. ✅ 关闭时间超过5分钟

## 工作流程

```
凌晨3点触发
    ↓
检查系统运行状态 → 未运行退出
    ↓
检查相机状态 → 相机开启退出
    ↓
检查关闭时间 → 不足5分钟退出
    ↓
停止项目进程
    ↓
清理临时文件
    ↓
重启PC系统
```

## 安装和使用

### 1. 安装 Cron 任务

```bash
./install_idle_restart_cron.sh
```

此脚本会将任务添加到 root 用户的 crontab（因为 reboot 需要 root 权限）。

### 2. 查看已安装的任务

```bash
sudo crontab -l
```

### 3. 查看 Cron 日志

```bash
grep CRON /var/log/syslog | grep idle_restart
```

### 4. 查看 IDLE 重启日志

```bash
tail -f /home/bosch/logs/visual.log | grep idle_restart
```

### 5. 测试逻辑（不会真正重启）

```bash
./test_idle_restart.sh
```

### 6. 手动测试重启（危险操作）

⚠️ **警告：此操作会立即重启PC系统！**

```bash
# 设置时间
sudo date -s '03:00:00'

# 创建测试文件
echo $$ > /home/bosch/vChisel_ros2_ws/.vchisel_system.lock
echo 'off' > /tmp/vchisel_camera_status.txt
echo $(($(date +%s) - 360)) > /tmp/vchisel_camera_off_time.txt

# 强制执行重启（需要 root 权限）
sudo FORCE_REBOOT=true /home/bosch/vChisel_ros2_ws/idle_restart.sh

# 测试完成后恢复时间
sudo ntpdate -s time.nist.gov
```

### 7. 卸载 Cron 任务

```bash
./uninstall_idle_restart_cron.sh
```

## 相关文件

| 文件 | 说明 |
|------|------|
| `idle_restart.sh` | 主重启脚本 |
| `install_idle_restart_cron.sh` | Cron 任务安装脚本 |
| `uninstall_idle_restart_cron.sh` | Cron 任务卸载脚本 |
| `test_idle_restart.sh` | 逻辑测试脚本 |
| `/tmp/vchisel_camera_status.txt` | 相机状态文件（on/off） |
| `/tmp/vchisel_camera_off_time.txt` | 相机关闭时间戳 |

## 日志示例

```
[2026-03-06 03:00:00] [INFO] [idle_restart] ========== 闲时重启检查开始 ==========
[2026-03-06 03:00:00] [INFO] [idle_restart] 当前时间为凌晨3点，开始检查闲时状态...
[2026-03-06 03:00:00] [INFO] [idle_restart] 系统正在运行 (PID: 12345)
[2026-03-06 03:00:00] [INFO] [idle_restart] 当前相机状态：off
[2026-03-06 03:00:00] [INFO] [idle_restart] 相机已关闭时间：15分钟（900秒）
[2026-03-06 03:00:00] [INFO] [idle_restart] 检测到闲时状态，准备重启系统...
[2026-03-06 03:00:00] [INFO] [idle_restart] 正在停止项目进程...
[2026-03-06 03:00:05] [INFO] [idle_restart] 项目进程已停止
[2026-03-06 03:00:08] [INFO] [idle_restart] 清理临时文件...
[2026-03-06 03:00:08] [INFO] [idle_restart] 即将重启PC系统
[2026-03-06 03:00:08] [INFO] [idle_restart] 重启原因：闲时自动重启
[2026-03-06 03:00:08] [INFO] [idle_restart] 触发时间：2026-03-06 03:00:08
[2026-03-06 03:00:08] [INFO] [idle_restart] 闲时条件：相机已关闭超过 5 分钟
[2026-03-06 03:00:10] [INFO] [idle_restart] 正在重启系统...
```

## 安全机制

1. **时间限制**：只能在凌晨3点执行
2. **条件检查**：必须满足所有闲时条件
3. **交互保护**：手动运行时需要设置 `FORCE_REBOOT=true`
4. **权限控制**：需要 root 权限才能执行重启
5. **日志记录**：所有操作都有详细日志

## 故障排查

### Cron 任务未执行

检查 cron 服务状态：
```bash
sudo systemctl status cron
```

检查 root crontab：
```bash
sudo crontab -l
```

查看系统日志：
```bash
grep CRON /var/log/syslog | tail -20
```

### 重启未触发

查看日志：
```bash
cat /home/bosch/logs/visual.log | grep idle_restart
```

检查相机状态文件：
```bash
cat /tmp/vchisel_camera_status.txt
cat /tmp/vchisel_camera_off_time.txt
```

手动计算关闭时间：
```bash
current_time=$(date +%s)
off_time=$(cat /tmp/vchisel_camera_off_time.txt)
idle_minutes=$(( (current_time - off_time) / 60 ))
echo "相机已关闭 $idle_minutes 分钟"
```

## 注意事项

1. ⚠️ 任务已添加到 root 用户的 crontab
2. ⚠️ 脚本会检查闲时条件，只有在满足条件时才会重启
3. ⚠️ 重启前会停止项目进程并清理临时文件
4. ⚠️ 重启操作不可逆，请谨慎测试
5. ⚠️ 建议在生产环境使用前进行充分测试
