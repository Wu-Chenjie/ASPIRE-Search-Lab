# 早期诊断握手的证实范围与修复

原Stage65三订阅可见/三次发送后，mapper和gateway各有真实状态，executor0响应，原监督正确停批。不能用“已发布”推断三方已记录；单进程多Node正例不等于真实执行器独立进程路径已验证。没有把缺响应改成成功。

已完成真实三进程正常与保存估计输入正例，受控沉默/暂态只丢诊断请求负例；无Gazebo/PX4或飞行。新协议用有界真实文件响应确认，同一ID有限重发且保留原接收库的去重和全部post查询信用。成功严格8wall、至多80次、实际发送间隔至少0.1wall，外层12/监督10和原控制安全期限保持。无法收齐明确失败，不生成成功报告。

正常旧发送方负载测试也能成功，故未唯一复现原运行的底层漏响应责任。受控0.6s暂态负例能直接证明原三次发布假定不足及新有界重发的作用，但不把夹具说成真实原故障。原主runner的Pybind转换错误和停止后的socket拒绝连接尚未唯一归因；新主入口仅记录堆栈，不忽略异常或改判。

首次夹具输出路径冲突被独占创建阻止，181旧原件逐SHA不变；所有后续输出独立。所有旧批/失败及测试负例保留，当前修复未改变规划器、恢复预算、未知空间处理或任何控制门。新比较不继承性能数据，完整固定六对同版重采。

证据：

- [真实多进程正常/沉默案例](audits/REAL-MULTIPROCESS-ACK-CASES.json)
- [受控暂态诊断丢弃对照](audits/TRANSIENT-DIAGNOSTIC-DROP-CASES.json)
- [保存估计输入负载对照](audits/SAVED-INPUT-MULTIPROCESS-CASES.json)
- [最终发送方/间隔/真实确认验证](audits/FINAL-MULTIPROCESS-ACK-VALIDATION.json)
- [旧原始文件未改变](audits/OLD-RAW-UNCHANGED-AFTER-FIXTURE.json)
