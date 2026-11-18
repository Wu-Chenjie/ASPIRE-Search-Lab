# Stage67只读监督清点竞态

Stage66第九轮P05-A在30.108sim/125.957057wall被外部监督SIGINT中止，原件494文件79193096字节/PAX及SHA保留，余3未启动，原批关闭。原先八轮完整180sim不改。

已定位的触发原因是stage65-monitor先rglob/is_file获得observer-status.tmp，原observer随后把临时文件原子replace为observer-status.json，第二次p.stat读取大小时抛FileNotFoundError。监督把该异常归为transport/decode并中止。原observer第30行写入/改名和原异常堆栈可对照；不是据此发现新的PX4、DDS或安全故障。中止引发pacer清理exit2原样保留，不能说该轮全链输入完整或全部进程正常。

只读外部修复把文件类型/大小合为一次lstat快照；仅源明确拥有的根路径observer-status.tmp、first-imu.tmp、camera-ready.tmp若在发现后消失，记录volatile_temporaries_renamed后跳过。原health.tmp排除不变。持久证据、未知tmp、同名嵌套路径消失、权限或其他IO错误仍失败，不无限重试。原storage/capture/reserve、心跳新鲜度、identity、安全、超时与停止规则不改。

[真实原子改名组件](audits/ATOMIC-RENAME-COMPONENT.json)在无ROS/Gazebo/PX4条件下同时写改名/读清点：旧实现实际复现异常，新实现无异常；[九轮保存目录](audits/NINE-SAVED-INVENTORIES-EQUAL.json)两实现字节清点完全相同。6项反例覆盖持久证据消失/未知tmp/嵌套同名/权限仍错误，原完整回归另记。

此处只证实和修复原监督堆栈所示竞态，不声称原先DDS漏响应唯一底层根因已定位，也不把之前Pybind11异常改判。Stage66失败原始数据/原判断保持，后续只有新Stage67 ID。
