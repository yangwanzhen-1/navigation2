# Navfn Planner

The NavfnPlanner is a global planner plugin for the Nav2 Planner server. It implements the Navigation Function planner with either A\* or Dij. expansions. It is largely equivalent to its counterpart in ROS 1 Navigation. The Navfn planner assumes a circular robot (or a robot that can be approximated as circular for the purposes of global path planning) and operates on a weighted costmap.

The `global_planner` package from ROS (1) is a refactor on NavFn to make it more easily understandable, but it lacks in run-time performance and introduces suboptimal behaviors. As NavFn has been extremely stable for about 10 years at the time of porting, the maintainers felt no compelling reason to port over another, largely equivalent (but poorer functioning) planner. 

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-navfn.html) for additional parameter descriptions.


# Navfn Planner

`NavfnPlanner` 是 Nav2 `Planner Server`（规划服务器）中的一个**全局规划器插件（Global Planner Plugin）**。

它实现了 **Navigation Function（导航函数）规划算法**，路径搜索时可以使用：

* **A*** 搜索
* **Dijkstra（迪杰斯特拉）** 搜索

它与 ROS 1 Navigation 中对应的 `NavFn` 规划器基本等价。

Navfn Planner 假设机器人是一个**圆形机器人**，或者在进行全局路径规划时，可以近似看作圆形机器人。它在一个**带权重的 Costmap（代价地图）**上进行路径规划。

ROS 1 中还有一个 `global_planner` 包。这个包实际上是对 `NavFn` 的一次重构，主要目的是让代码结构更加清晰、更容易理解。

但是，与原始的 `NavFn` 相比，`global_planner` 存在以下问题：

* 运行时性能较差；
* 会产生一些次优的规划行为（suboptimal behaviors）。

在将 Navigation Stack 移植到 ROS 2 / Nav2 时，`NavFn` 已经稳定运行了大约 **10 年**。

因此，Nav2 的维护者认为，没有充分的理由再去移植另一个功能基本相同、但实际表现反而更差的规划器。

所以 Nav2 最终选择继续使用并移植：

`NavFn`

而不是 ROS 1 中重构后的：

`global_planner`

有关 Navfn Planner 更多参数说明，可以查看它的：

[Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-navfn.html)

