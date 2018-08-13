##文件说明##
###src###
####common####
常用函数

- common: 常用函数, 例如最大值最小值
- fileAdvance: 文件操作, 将采样的数据保存成文件
- printAdvance: 方便输出
####example####
各类测试文件

- 其它: 各类测试代码
- modelData: 模型XML文件
- test.cpp: *主函数*
####ext####
外部文件 - 开源库Eigen
####hardware####
一代控制其中的通信相关的一些代码
####ik####
逆向运动学

- IKSolver: 逆解器基类
- PieperSolver: 符合Pieper准则一类机器人的逆解, 未进行维护
- SiasunSR4CSolver: 新松机器人通用的逆解器, 是IKSolver的派生类
####kinematic####
正向运动学

- Frame: 坐标描述
- State: 机器人关节状态描述
- Trsf: xyzrxryrz位姿表示方法
####math####
数学类

- HTransform3D: 4X4变换矩阵
- Rotation3D: 3X3旋转矩阵
- Vector3D: 3X1向量
- Q: 数组, 一般用来表示机器人关节位置, 速度或加速度
- Quaternion: 单位四元数
- LeastSquare: 最小二乘法
- Integrator: 路径长度采样计算
####model####
模型构建

- Config: 机器人姿态配置
- DHParameters: DH参数(一行)
- DHTable: DH参数表(多行)
- Jacobian: 雅克比矩阵
- Link: 机器人关节
- SerialLink: 串联机器人
- Tool: 工具计算(只完成了TCP计算)
####parse####
文件解析

- 其它: 来自一代控制器的代码(未使用)
- RobotXMLParse: 机器人XML文件解析器
####pathplanner####
路径规划器

- Planner: *标准规划器基类*, 它的派生类指针可以放置到运动堆栈中, 以下为其派生类
- CircularPlanner: 圆弧路径规划器
- LinePlanner: 直线路径规划器
- MultiLineArcBlendPlanner: 连续直线圆弧混合规划器
- JoggingPlanner: 示教规划器
- QtoQPlanner: 点到点规划
- RotationPlanner: 纯旋转运动规划

*其它*

- QBlend: 使用五次多项式混合两个机器人关节状态
- SmoothMotionPlanner: 平滑S型曲线规划器
- SMPlannerEx: 平滑S型曲线规划器拓展
- TimeOptimal: 时间最优规划
- PointToPointPlanner: 非标准 - 点到点规划, 随着速度变化轨迹可能会变化
- ExcessMotionPlanner: 未实现
- MLBBPlanner: 未完成

####simulation####
仿真类

- IterativeSimulation: 积分仿真
- MotionStack: 运动堆栈
- TaskStack: 任务堆栈
####trajectory####
轨迹描述类/插补器

- Interpolator: 插补器基类
- CompositeInterpolator: 复合插补器
- ConvertedInterpolator: 输出转换插补器
- SequenceInterpolator: 基础序列插补器(把多个插补器串联)
- CircularInterpolator: 基础圆弧插补器
- LinearInterpolator: 基础线性插补器(包括直线, 旋转等)
- PolynomialInterpolator: 基础多项式插补器
- RotationInterpolator: 基础(可多圈)姿态旋转插补器
- BezierInterpolator: 基础贝赛尔曲线插补器
- BezierPath: 以长度为索引的基础贝赛尔曲线插补器
- CircularTrajectory: 圆弧路径
- LineTrajectory: 直线路径(也可以描述纯旋转的路径)
- MLABTrajectory: 连续线段圆弧混合的路径
- Sampler: 通用采用工具
- Trajectory: 以路径长度为索引的ikInterpolator, 提供一些算法. 与上面几个名字中带有Trajectory的没有关系
- TwopartBezier: 二段贝塞尔

###doc###
####doxygen####
doxygen文档的输出文件夹, 具体输出由Doxyfile文件配置
####img####
用于doxygen文档的一些内嵌图片
####其它####
- .git: git文件夹, 不用管
- .gitignore: 配置git忽略文件,包括Debug,Release,doxygen输出文件夹和名字中带有"temp"的所有文件
- Doxyfile: doxygen的配置文件
- 其它

##注释文档使用说明##
注释用doxygen格式编写, 安装doxygen, dot和latex的电脑上在robot目录中执行
`doxygen ./Doxyfile`
即可输出doxygen文档. 当前的配置是输出Html文件和Latex文件. 打开`doc/doxygen/html`文件夹下的index.html即可浏览网页版说明文档. Latex文件用于pdf输出, 由于文档中包含中文, 需要对文件进行修改后再调用make输出pdf. "资料整理/程序代码"文件夹中的generatePDF脚本可以辅助这一操作. 打开该文件, 修改robot目录和输出目录, 用命令行运行脚本即可.