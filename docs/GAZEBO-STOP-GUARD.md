# Gazebo 8 传感器停止并发兼容补丁

2026-10-18，工程退出修复，不是搜索算法贡献。地面原例中所有外部客户端已静止，服务回执true但进程8秒仍不退；原生栈主线程在SensorsPrivate.Stop的join。输出为普通文件、server和Ruby同一PID，排除本例输出管道和遗留server子进程。去掉第二次SIGINT仍复现。

最小无PX4/规划/检测组件的受控退出调度中，同一SensorsPrivate对象两个Stop对同一pthread目标重叠join的两例都挂起；第三例未重叠且正常。相同Stop入口重叠下，串行调用原始Stop三例正常且无原生join重叠。此机制与原无注入栈一致；未采栈的全部历史故障不能唯一归因。

`native/gz_sensor_stop_guard.cpp`只用独立mutex串行调用原始Stop，保留其内部操作，不修改Gazebo或RACER安装文件、不替换规划器、物理、渲染、传感器或线程join实现。生产库不含调度延迟/采栈/诊断开关，不导出pthread_join。只支持当前校验SHA的aarch64 Gazebo Sensors插件、单world单Sensors系统，内部符号ABI有版本绑定；换版本须重新验证。失败解析直接异常终止，不伪造退出成功。

用`python3 -m tools.flight.build_gazebo_guard /tmp/<unique-build>`构建；只有显式`RACER_GZ_SHUTDOWN_GUARD=<library>`启用，仅注入本轮Gazebo子进程。启动核验源/二进制/实际插件SHA、场景插件数量、无诊断环境，并核验真实Gazebo映射加载。两组用相同库与参数，运行报告记录所有SHA。默认历史行为保持。

原8秒非显示清理预算、PID所有权、独立评测、强杀/异常失败分类和全部飞行安全/时效门不变。先在独立强制调度测试及无注入长地面组件取得稳定证据、完整回归后，才能恢复比较；不能用组件或已跑满但清理失败的数据冒称完整比较成功。
