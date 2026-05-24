# IRBL repository documentation

Документ описывает репозиторий `irbl`, его ROS 2 nodes, библиотечные пакеты,
порядок работы, связи между компонентами и роль локального A* replanner в связке
с `rbl_controller`.

Контекст алгоритма соответствует работе `2504.18840v3.pdf`: базовая идея системы
это distributed Lloyd-based / RBL-подход, где робот строит локальную безопасную
область движения на основе соседей и препятствий, вычисляет центроид этой
области и отправляет его как локальную reference-точку в MRS control stack. В
данном репозитории поверх этой идеи добавлен практический локальный A* replanner,
который помогает в dense forest-сценариях: он дает контроллеру промежуточный
waypoint на безопасном пути, а RBL-часть продолжает выполнять reactive
centroid-based control внутри локально допустимой области.

## 1. Назначение репозитория

`irbl` содержит ROS 2 реализацию локального контроллера для UAV в лесной/плотной
среде:

- получение состояния UAV из MRS estimation/control stack;
- получение локальной карты препятствий как `sensor_msgs/PointCloud2` или
  `octomap_msgs/Octomap`;
- учет соседних UAV как динамических препятствий;
- построение локальной допустимой области движения `cell_A`;
- вычисление целевой точки-центроида по RBL/Lloyd-логике;
- опциональное построение локального A* пути до цели;
- преобразование результата в `mrs_msgs/srv/ReferenceStampedSrv` для
  `control_manager/reference`;
- публикация диагностических visualization topics.

На уровне ROS runtime главным node является composable node
`rbl_controller::WrapperRosRBL`. Остальные основные пакеты являются библиотеками
или вспомогательными executable/script nodes.

## 2. Структура репозитория

```text
irbl/
  ros_packages/
    ciri/
      include/ciri/ciri.h
      src/ciri.cpp
    map_generator/
      src/random_forest_sensing.cpp
    nodelet/
      src/nodelet.cpp
      launch/rbl_controller.launch.py
      config/default.yaml
    rbl_controller_core/
      include/rbl_controller_core/rbl_controller.h
      src/rbl_controller.cpp
    rbl_rc/
      rbl_rc/rbl_rc/rc.py
    rbl_replanner/
      include/rbl_replanner/replanner.h
      src/replanner.cpp
      src/replanner2.cpp
  tmux/
    forest_real/
    forest_easy/
    sim_1_uav/
    sim_2_uav/
    irbl/
    irbl_gps/
```

Практически важные директории:

- `ros_packages/nodelet`: ROS 2 component node, который соединяет MRS topics,
  services и C++ `RBLController`.
- `ros_packages/rbl_controller_core`: основная логика RBL-контроллера.
- `ros_packages/rbl_replanner`: локальный A* replanner и voxel grid.
- `ros_packages/ciri`: построение convex region / half-space constraints для
  безопасной области.
- `ros_packages/map_generator`: генератор искусственного forest point cloud.
- `ros_packages/rbl_rc`: RC-интерфейс для задания цели и параметра `betaD`.
- `tmux/*`: сценарии запуска экспериментов/симуляций.

## 3. Runtime nodes

### 3.1 `rbl_controller` / `WrapperRosRBL`

Файл:

- `ros_packages/nodelet/src/nodelet.cpp`
- launch: `ros_packages/nodelet/launch/rbl_controller.launch.py`
- package name: `rbl_controller_node`
- component plugin: `rbl_controller::WrapperRosRBL`
- registered via `RCLCPP_COMPONENTS_REGISTER_NODE`

Это главный ROS 2 node репозитория. Он не реализует всю математику сам, а
оборачивает библиотеку `RBLController` в ROS-интерфейс.

Основные обязанности:

- загрузить параметры из YAML через `mrs_lib::ParamLoader`;
- создать `RBLController`;
- принять odometry, altitude, point cloud / octomap, group states;
- преобразовать все входы в frame управления;
- вызывать `RBLController::getNextRef()`;
- отправлять полученную reference-точку в MRS `control_manager/reference`;
- публиковать визуализацию cells, centroid, path, local obstacles, waypoint;
- обрабатывать сервисы activation, deactivation, goto, set_betaD.

#### Входные topics

Через launch remapping используются:

```text
~/odom_in          -> estimation_manager/odom_main
~/alt_in           -> estimation_manager/garmin_agl/agl_height
~/pcl_in           -> configurable pcl_topic
~/octomap_in       -> octomap_server/octomap_local_binary
~/group_states_in  -> filter_reflective_uavs/pose_vel
```

`pcl_topic` в `rbl_controller.launch.py` по умолчанию:

```text
/uav2/losos_server/current_submap_pc
```

В `forest_real/session.yml` он также передается явно при запуске
`rbl_controller_node`.

#### Выходные topics

Node публикует:

```text
~/position
~/centroid
~/seed_B
~/target
~/replanner_waypoint
~/cell_s
~/cell_a
~/actively_sensed_A
~/local_obstacles
~/inflated_map
~/cloud
~/path
/benchmark/event
/benchmark/goal
```

Наиболее полезные для отладки:

- `~/path`: A* path в world coordinates;
- `~/replanner_waypoint`: текущий waypoint, выбранный на A* path;
- `~/centroid`: итоговая centroid/reference-точка RBL;
- `~/actively_sensed_A`: часть `cell_A`, видимая текущим limited FOV;
- `~/local_obstacles`: локальные препятствия, использованные при partition;
- `~/cloud`: текущий obstacle cloud после добавления соседей как препятствий.

#### Services

```text
~/control_activation_in    std_srvs/srv/Trigger
~/control_deactivation_in  std_srvs/srv/Trigger
~/goto_out                 mrs_msgs/srv/Vec4
~/set_betaD                mrs_msgs/srv/Float64Srv
~/ref_out                  mrs_msgs/srv/ReferenceStampedSrv client
```

`goto_out` задает цель:

```text
goal: [x, y, z, heading]
```

Внутри callback:

1. записывает benchmark goal;
2. вызывает `rbl_controller_->setGoal()`;
3. публикует `/benchmark/goal`;
4. публикует `/benchmark/event = goal_set`.

`set_betaD` меняет желаемую ширину weighting-функции Lloyd/RBL:

```cpp
rbl_params_.betaD = req->value;
rbl_controller_->setBetaD(req->value);
```

#### Таймер `cbTmSetRef`

`cbTmSetRef()` это основной runtime цикл управления. Его частота задается
параметром:

```yaml
rate:
  timer_set_ref: 20
```

Порядок работы:

1. Если node еще не initialized или нет odometry, выход.
2. Берет `nav_msgs/Odometry`.
3. Трансформирует position и velocity в `control_frame`.
4. Вычисляет roll/pitch/yaw из quaternion odometry.
5. Если пришла altitude, передает ее в контроллер.
6. Если пришли group states, обновляет состояния соседей.
7. Если включен `octomap_msg`, преобразует Octomap occupied leaves в PCL cloud.
8. Иначе читает `sensor_msgs/PointCloud2` и конвертирует в PCL.
9. Если включено `group_odoms/add_to_pcl`, добавляет соседние UAV в PCL как
   сферические occupied voxel clusters.
10. Публикует debug cloud.
11. Если controller не activated, не двигает UAV.
12. Вызывает `RBLController::getNextRef()`.
13. Отправляет reference в `control_manager/reference` через service client.

Это важный момент: `WrapperRosRBL` отправляет в MRS не A* path напрямую, а одну
локальную reference-точку, вычисленную контроллером. A* влияет на нее через
waypoint/destination.

#### Таймер `cbTmDiagnostics`

Частота:

```yaml
rate:
  timer_diagnostics: 10
```

Публикует visualization markers/path/clouds. Также проверяет benchmark goal:
если distance до goal меньше `0.3 m`, публикует `/benchmark/event = goal_reached`.

#### Добавление других UAV в obstacle cloud

`WrapperRosRBL::addAgents2PCL()` берет `group_states` и добавляет вокруг каждого
соседа сферический набор PCL points с intensity `1.0`. Радиус задается
`encumbrance`, voxel spacing - `voxel_size`.

Зачем это нужно:

- A* и partitionCellA получают соседей как реальные occupied voxels;
- локальный replanner не прокладывает путь сквозь другого UAV;
- RBL/CWVD-часть получает дополнительные constraints от `neighbors`.

### 3.2 `random_forest`

Файл:

- `ros_packages/map_generator/src/random_forest_sensing.cpp`
- package: `map_generator`
- executable: `random_forest`

Это standalone ROS 2 node для генерации искусственной карты леса/препятствий.
В текущем `forest_real/session.yml` запуск `ros2 run map_generator random_forest`
закомментирован, вместо него используется playback bag, но node остается
полезным для симуляций.

Публикует:

```text
/map_generator/local_cloud
/map_generator/global_cloud
/pcl_render_node/local_map
```

Подписывается:

```text
odometry
```

Основные функции:

- `RandomMapGenerate()`: генерирует прямоугольные/полярные obstacle columns и
  ring-like obstacles.
- `RandomMapGenerateCylinder()`: генерирует цилиндрические препятствия с
  минимальной дистанцией между центрами.
- `rcvOdometryCallback()`: получает odometry, извлекает локальную часть карты
  вокруг текущей позиции и публикует local cloud.

Параметры:

```text
map/x_size
map/y_size
map/z_size
map/obs_num
map/resolution
map/circle_num
ObstacleShape/lower_rad
ObstacleShape/upper_rad
ObstacleShape/lower_hei
ObstacleShape/upper_hei
sensing/radius
sensing/rate
min_distance
```

### 3.3 `rc_goal_controller`

Файл:

- `ros_packages/rbl_rc/rbl_rc/rbl_rc/rc.py`
- package: `rbl_rc`
- node name: `rc_goal_controller`

Python node для управления goal и `betaD` с RC transmitter.

Подписки:

```text
/uav2/mavros/state
/uav2/mavros/rc/in
/uav2/estimation_manager/odom_main
```

Service clients:

```text
/uav2/rbl_controller/goto
/uav2/rbl_controller/set_betaD
/uav2/estimation_manager/change_estimator
```

Логика:

- RC channel 9 выбирает estimator: `point_lio` или `gps_garmin`.
- После смены estimator node ждет свежую pose и отправляет hold-goal в текущей
  позиции, чтобы UAV не прыгнул к старой цели.
- RC channel 2 мапится в `betaD` диапазона `[0.1, 20.0]`.
- RC roll/pitch channels задают goal вокруг текущей позиции в круге радиуса
  `max_distance = 6.0 m`.
- Если стики в deadzone, goal обновляется как hold текущей XY-позиции.

Этот node не участвует в основном A*/RBL вычислении, но удобен для real flight
управления целями и динамической настройки `betaD`.

### 3.4 Внешние nodes, запускаемые tmux-сценариями

В `tmux/forest_real/session.yml` есть несколько процессов, которые важны для
полного эксперимента, но не реализованы в `irbl/ros_packages`.

Они являются runtime-зависимостями сценария:

- `rmw_zenoh_cpp rmw_zenohd`: transport/router для ROS 2 DDS/RMW окружения.
- `mrs_uav_status status.sh`: статус MRS UAV stack.
- `status_proxy.py`: локальный helper из `tmux/forest_real`, который подменяет
  или проксирует статус для сценария.
- `mrs_multirotor_simulator`: симулятор multirotor UAV и hardware API.
- `mrs_uav_autostart automatic_start.launch.py`: автоматический старт UAV.
- `filter_reflective_uavs`: внешний пакет, публикующий относительные poses и
  velocities соседних UAV в `filter_reflective_uavs/pose_vel`.
- `ros2 bag play` через `download_and_play_bag.sh`: источник записанной карты /
  point cloud для forest-сценария.
- `rviz2`: визуализация.

Эти nodes дают окружение, в котором работает `rbl_controller`. Сам IRBL
контроллер ожидает, что MRS core уже поднят, есть odometry, TF, obstacle cloud и
service endpoint `control_manager/reference`.

## 4. Библиотечные пакеты

### 4.1 `rbl_controller_core`

Файлы:

- `ros_packages/rbl_controller_core/include/rbl_controller_core/rbl_controller.h`
- `ros_packages/rbl_controller_core/src/rbl_controller.cpp`

Это главная библиотека управления. Она не является ROS node сама по себе, но
используется `WrapperRosRBL`.

#### `RBLParams`

Основные параметры:

- `radius`: радиус локальной области `cell_S`, максимальный горизонт локального
  планирования RBL;
- `path_lookahead_distance`: расстояние pure-pursuit waypoint на A* path;
- `step_size`: дискретизация точек внутри `cell_S`;
- `encumbrance`: safety radius UAV;
- `betaD`, `beta_min`, `dt`: параметры адаптации weighting function;
- `d1..d7`: пороги rule-based поведения;
- `cwvd_rob`, `cwvd_obs`: коэффициенты weighted Voronoi planes для роботов и
  obstacles;
- `limited_fov`, `lidar_tilt`, `lidar_fov`: ограничение видимой области;
- `ciri`: использовать CIRI convex decomposition или classic partition;
- `replanner`: включить A* replanner;
- `use_garmin_alt`: брать altitude из Garmin AGL или из `agent_pos.z`;
- `voxel_size`: дискретизация obstacle cloud в RBL;
- `inflation_bonus`: дополнительная inflation для replanner occupancy grid.

#### Инициализация `RBLController`

В constructor:

1. `radius_sensing_ = radius + encumbrance + voxel_half_diagonal`;
2. `beta_ = beta_min`;
3. если `params.replanner == true`, создается `RBLReplanner`;
4. если `params.ciri == true`, создается `CIRI`.

Параметры replanner сейчас задаются в коде:

```cpp
map_width = 30.0
map_height = 10.0
weight_safety = 1.0
weight_deviation = 100.0
replanner_vox_size = 0.3
replanner_freq = 1.0 Hz
```

Это означает, что A* работает в локальном voxel grid примерно `30 x 30 x 10 m`
с разрешением `0.3 m`, центрированном на UAV.

#### Входы контроллера

`WrapperRosRBL` обновляет:

- `setCurrentPosition()`;
- `setCurrentVelocity()`;
- `setRollPitchYaw()`;
- `setAltitude()`;
- `setGroupStates()`;
- `setPCL()`;
- `setGoal()`;
- `setBetaD()`.

`setGoal()` дополнительно:

- ставит `has_goal_ = true`;
- ставит `pending_replan_ = true`;
- увеличивает `goal_generation_`;
- очищает `path_` и `inflated_map_`;
- сбрасывает локальные переменные waypoint/centroid.

Это важно для A*: новый goal всегда принудительно запускает replanning.

#### Основной метод `getNextRef()`

`getNextRef()` возвращает `mrs_msgs::msg::Reference`.

Логика:

1. Если цели нет, вернуть hold reference в текущей позиции.
2. Если obstacle cloud еще нет, вернуть hold reference.
3. Если включен replanner:
   - проверить, свободен ли async replanner;
   - если `pending_replan_` или сработал `replanTimer()`, запустить
     `RBLReplanner::plan()` через `std::async`;
   - если async result готов, принять path только если `goal_generation`
     совпадает с текущей целью;
   - если новый path пустой, сохранить старый path;
   - если path уже есть, выбрать waypoint на нем через
     `determineWaypointFixedDistance()`;
   - если path еще нет, вернуть hold reference.
4. Сформировать `group_positions`.
5. Построить `cell_S` и `cell_A` через `createAndPartitionCellA()`.
6. Вычислить `c1_full_`, `c2_`, `c1_no_rot_`.
7. Применить `applyRules()` для адаптации `beta`, `theta`, `phi`.
8. Сдвинуть итоговый centroid в реально sensed cell:
   `c1_ = movePointToCell(c1_full_, sensed_cell_A_)`.
9. Сформировать reference через `determineNextRef()`.

Ключевой вывод: A* не заменяет RBL-controller. A* выбирает локальный waypoint,
а RBL-controller дальше выбирает безопасную reference-точку внутри локальной
допустимой области.

#### `cell_S`, `cell_A`, `sensed_cell_A`

`cell_S` это дискретизированная сфера или круг вокруг UAV:

- 2D mode: `getpointsInsideCircle()`;
- 3D mode: `pointsInsideSphere()`;
- радиус: `params_.radius`;
- шаг: `params_.step_size`.

`cell_A` это безопасная часть `cell_S`, оставшаяся после применения half-space
constraints от:

- соседних UAV;
- obstacles в PCL;
- CIRI planes, если включен `ciri`.

`sensed_cell_A` это часть `cell_A`, которая лежит в текущем поле зрения LiDAR
по `lidar_tilt`, `lidar_fov` и `rpy`.

Дополнительно в `createAndPartitionCellA()` есть near-agent добавление:
около UAV принудительно добавляется локальная область радиуса
`max(encumbrance, 2 * step_size)` в `sensed_cell_A`. Это предотвращает ситуацию,
когда из-за limited FOV или пустой видимой области ближайшие безопасные точки
исчезают полностью.

#### Classic partition

`partitionCellA()` строит half-space constraints:

- для каждого соседа строит CWVD-like разделяющую плоскость с учетом
  `encumbrance` и `cwvd_rob`;
- для локальных obstacle voxels в радиусе `radius_sensing_` строит аналогичные
  planes с учетом obstacle voxel half-diagonal и `cwvd_obs`;
- удаляет из `cell_S` точки, лежащие на запрещенной стороне хотя бы одной
  плоскости.

Для ускорения поиска локальных obstacles используется `pcl::KdTreeFLANN` и
`radiusSearch`.

#### CIRI partition

`partitionCellACiri()` вызывает `CIRI::comvexDecomposition()` для построения
convex polytope вокруг отрезка между `agent_pos` и seed. Из результата CIRI
получаются planes, которые затем используются так же, как classic planes.

Если CIRI не инициализирован, point cloud пустой, decomposition failed или
`cell_A` пустая, код возвращается к classic partition. Это важный fallback:
ошибка CIRI не должна остановить контроллер.

#### Centroid и weighting function

`computeScalarValue()` задает веса точек:

```text
w(q) = exp(-(dist(q, destination) - min_dist) / beta)
```

Это численно устойчивый вариант Laplacian-like weighting из статьи: вместо
`exp(-dist/beta)` используется нормализация по `min_dist`, чтобы избежать
слишком малых весов.

`computeCentroid()` вычисляет weighted centroid по `cell_A` или `cell_S`.
Если веса некорректны или cell пустая, centroid ставится в позицию UAV.

Также есть boundary logic:

- если centroid слишком близко к границе `cell_A` (`boundary_threshold`);
- и UAV реально движется быстрее `boundary_threshold_speed`;
- и `beta < 20`;

то `beta` увеличивается и centroid пересчитывается. Это практическая реализация
идеи из статьи про управление расстоянием от centroid до границы безопасной
области при tracking/measurement uncertainties.

#### `applyRules()`

`applyRules()` реализует rule-based коррекцию:

- `beta` уменьшается, когда centroid `c1` слишком близко к UAV и сильно
  отличается от centroid свободной области `c2`;
- иначе `beta` экспоненциально стремится к `betaD`;
- `theta` поворачивает направление цели влево/вправо, если геометрия ячеек
  показывает необходимость обхода;
- `phi` может корректировать вертикальное направление, если `use_z_rule`;
- есть проверка, что поворот не ухудшает расстояние к `c1_no_rot`.

В статье этому соответствует adaptive control / rule-based Lloyd часть:
параметр `beta` регулирует "greediness" движения к цели, а side-rule помогает
роботам договориться о стороне обхода без коммуникации.

#### `determineNextRef()`

Формирует reference для MRS.

Если `limited_fov == true`, контроллер:

- смотрит, находится ли centroid в переднем секторе относительно текущего yaw;
- если да, отправляет позицию `c1`;
- если centroid вне +/-90 deg, position reference ставится в текущую позицию,
  но heading разворачивается на `c1_full`.

Это важно для не-stuck поведения с ограниченным FOV: UAV может сначала
развернуться к безопасному centroid/области, не пытаясь лететь боком в
невидимую область.

### 4.2 `rbl_replanner`

Файлы:

- `ros_packages/rbl_replanner/include/rbl_replanner/replanner.h`
- `ros_packages/rbl_replanner/src/replanner.cpp`

Это библиотека локального A* replanner.

#### На каком типе данных работает A*

A* работает не по готовой ROS costmap и не по Octomap напрямую.

Pipeline данных такой:

```text
sensor_msgs/PointCloud2 или Octomap
  -> pcl::PointCloud<pcl::PointXYZI>
  -> локальный VoxelGrid occupancy
  -> inflated occupancy grid
  -> clearance grid
  -> A* graph search по voxel indices
```

Основной тип:

```cpp
struct VoxelGrid {
  int X, Y, Z;
  std::vector<int> data;
}
```

`_inflated_grid_` это binary occupancy grid:

- `0`: свободный voxel;
- `1`: occupied/inflated obstacle voxel.

`_clearance_grid_` это не occupancy, а distance-to-nearest-obstacle field в
cell units. Он вычисляется из `_inflated_grid_` и используется как cost term.

Поэтому точная формулировка:

> A* ищет путь по локальной inflated occupancy grid, но стоимость ребер
> дополнена clearance-derived safety cost и deviation cost. Это не полноценная
> costmap как отдельный входной слой; cost field строится внутри replanner из
> occupancy.

#### Конструктор

Конструктор задает размеры voxel grid:

```cpp
_X_ = ceil(map_width / replanner_vox_size)
_Y_ = ceil(map_width / replanner_vox_size)
_Z_ = ceil(map_height / replanner_vox_size)
```

Grid центрируется на текущей позиции UAV. В этой версии `map_length` не
используется, потому что его нет в `ReplannerParams`.

#### Когда вызывается A*

A* вызывается внутри:

```cpp
RBLReplanner::plan()
```

А `plan()` запускается из `RBLController::getNextRef()` асинхронно:

```cpp
replanner_future_ = std::async(... rbl_replanner_->plan())
```

Условия запуска replanner:

- `pending_replan_ == true`, что происходит после `setGoal()`;
- или `rbl_replanner_->replanTimer()` возвращает true;
- и предыдущий async replanner уже завершен.

`replanTimer()` срабатывает:

- на первом планировании;
- далее с периодом `1 / replanner_freq`, сейчас `1 Hz`.

Внутри `RBLReplanner::plan()` A* запускается не всегда. Сначала:

1. обновляется occupancy через `fillAndInflateGrid()`;
2. предыдущий `path_` переводится в grid coordinates;
3. вызывается `shouldReplan()`.

`shouldReplan()` возвращает true если:

- goal изменился;
- path пустой;
- пройдено около `30%` текущего path;
- текущий path blocked новым occupancy grid.

Если `shouldReplan() == false`, `plan()` возвращает старый path.
Если true:

1. считается clearance grid;
2. вызывается `AStarPlan()`;
3. grid path переводится в world path;
4. возвращается raw path без smoothing.

#### `fillAndInflateGrid()`

Берет текущий PCL cloud, переводит points в local voxel indices и заполняет
binary occupancy. Затем выполняет inflation.

Текущая версия использует сферическую инфляцию:

```cpp
dx*dx + dy*dy + dz*dz <= r*r
```

Это математически соответствует safety radius вокруг obstacle. Кубическая
инфляция блокировала бы весь cube `[-r, r]^3`, что более консервативно и может
закрывать диагональные проходы в лесу. Сферическая inflation лучше соответствует
реальной геометрии UAV/encumbrance.

Также слой `z = 0` заполняется как floor obstacle.

#### `calculateClearanceGrid()`

Строит поле расстояний до ближайшего occupied voxel:

1. occupied voxels получают distance `0`;
2. free voxels получают большое значение `INF`;
3. 1D squared distance transform применяется по X;
4. потом по Y;
5. потом по Z;
6. результат сохраняется как `floor(sqrt(distance_squared))`.

Это Felzenszwalb-Huttenlocher style separable Euclidean distance transform.
Преимущество: линейная сложность по числу voxel cells вместо Dijkstra-like
priority queue expansion.

#### `AStarPlan()`

Граф:

- вершины: свободные voxels `(x, y, z)`;
- соседи: 26-connected grid, включая face, edge и corner moves;
- occupied voxels отбрасываются;
- out-of-bounds voxels отбрасываются;
- start/goal сначала корректируются через `closestFreeIdx()`.

Стоимость:

```text
motion_cost = euclideanDistance(parent, child)
clearance = replanner_vox_size * clearance_grid[child]
safety_penalty = weight_safety / (clearance + eps)
deviation_penalty = weight_deviation * deviationPenalty(previous_path, parent, child)
tentative_g = parent.g + motion_cost + safety_penalty + deviation_penalty
h = euclideanDistance(child, goal)
f = g + h
```

`safety_penalty` отталкивает path от obstacles. Чем меньше clearance, тем больше
штраф.

`deviationPenalty()` возвращает `0`, если переход `(parent -> child)` совпадает
с участком предыдущего path, и `1` иначе. При `weight_deviation = 100` это
сильно стабилизирует path и уменьшает дрожание траектории между replanning
итерациями.

Оптимизации:

- `best_g_score` хранит лучший найденный cost для каждого voxel;
- candidate отбрасывается до push в priority queue, если он не лучше;
- closed voxels предотвращают повторное расширение;
- counters `expanded/generated/skipped_*` печатаются для диагностики.

#### Почему raw path возвращается без smoothing

В README `tmux/forest_real/README.md` объяснено, что smoothing мог срезать
несколько A* узлов и стереть obstacle-avoidance структуру. Для леса это опасно:
сглаженный shortcut может пройти ближе к препятствиям, чем исходный voxel path.

Поэтому текущий `plan()` возвращает raw A* world path. Сглаживание остается как
функция, но не используется в активной планировочной ветке.

### 4.3 `ciri`

Файлы:

- `ros_packages/ciri/include/ciri/ciri.h`
- `ros_packages/ciri/src/ciri.cpp`

`ciri` это библиотека построения convex region / polytope вокруг seed segment.
Она используется только если:

```yaml
rbl_controller:
  ciri: true
```

Основные классы:

- `Ellipsoid`: геометрия эллипсоида, преобразования между world/ellipsoid
  frame, расстояния, проверка points inside;
- `CIRI`: iterative decomposition, tangent planes, ellipsoid fitting,
  conversion to plane data.

Главный метод:

```cpp
bool CIRI::comvexDecomposition(bd, pc, a, b)
```

Входы:

- `bd`: boundary planes;
- `pc`: point cloud obstacles как `Eigen::Matrix3Xf`;
- `a`: старт, обычно `agent_pos`;
- `b`: seed point.

Выход:

- через `getPlaneData()` возвращается набор пар `(normal, point_on_plane)`.

Эти planes затем переводятся в `plane_normals` / `plane_points` и используются
для фильтрации `cell_S` в `cell_A`.

### 4.4 `map_generator`

Библиотека/пакет содержит executable `random_forest`. Он полезен для
синтетических тестов без bag playback.

Зависимости:

- `rclcpp`;
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`;
- `PCL`;
- `pcl_conversions`;
- `Eigen3`.

### 4.5 `rbl_rc`

Python package с node `rc_goal_controller`. Используется для real flight
операций, когда goal и `betaD` задаются с пульта, а также для переключения
estimator.

## 5. Как A* связан с `rbl_controller`

Включение A* контролируется YAML:

```yaml
rbl_controller:
  replanner: true
```

В `tmux/forest_real/config/rbl_controller.yaml` replanner выключен:

```yaml
replanner: false
```

В `tmux/forest_real/config/rbl_controller_astar.yaml` replanner включен:

```yaml
replanner: true
```

Порядок работы при включенном replanner:

1. `WrapperRosRBL` получает odometry, point cloud, group states.
2. `WrapperRosRBL` вызывает `RBLController::getNextRef()`.
3. `RBLController` асинхронно запускает `RBLReplanner::plan()`, если:
   - новая цель;
   - прошел replanner timer;
   - replanner не занят.
4. `RBLReplanner` строит local inflated occupancy grid из PCL.
5. `RBLReplanner` считает clearance grid.
6. `RBLReplanner` запускает A* по voxel occupancy grid.
7. A* возвращает path как список voxel indices.
8. Replanner переводит path в world coordinates.
9. `RBLController` принимает path, если он относится к текущему
   `goal_generation_`.
10. `RBLController` выбирает waypoint на path через pure-pursuit lookahead.
11. Этот waypoint становится `destination_` для RBL weighting function.
12. RBL строит `cell_A`, считает centroid и отправляет reference.

То есть A* в системе выполняет роль локального path-shaper. Он говорит RBL,
куда "смотреть" и какую промежуточную цель считать актуальной. Но фактическая
команда в MRS stack остается centroid/reference, безопасная относительно
локальной RBL-ячейки.

## 6. Почему дрон не должен стопориться на месте

В коде есть несколько приемов, которые уменьшают зависание и дрожание.

### 6.1 Асинхронный replanner

A* запускается через `std::async`, а `getNextRef()` не блокирует control loop.
Если planner еще считает, контроллер продолжает использовать предыдущий path.
Это защищает 20 Hz reference loop от тяжелого планирования.

### 6.2 `pending_replan_` и `goal_generation_`

При новой цели:

- `pending_replan_ = true`;
- `goal_generation_++`;
- старый path очищается.

Async result принимается только если generation совпадает. Старый результат от
предыдущей цели не может перезаписать актуальный path.

### 6.3 Сохранение старого path при неудачном replanning

Если replanner вернул пустой path, `RBLController` пишет diagnostic message и
сохраняет предыдущий path. Это лучше, чем мгновенно остановить UAV из-за
единичного failed replanning.

### 6.4 Hold до первого пути

После новой цели, если path еще нет, `getNextRef()` возвращает hold reference:

```text
position = current UAV position
heading = current yaw
```

Это safety behavior. UAV не летит напрямую в goal через препятствия, пока
replanner не построит первый path.

### 6.5 Pure-pursuit lookahead по A* path

`determineWaypointFixedDistance()`:

1. ищет ближайшую точку path к UAV;
2. идет вперед по сегментам path;
3. ищет пересечение path segment со сферой радиуса `path_lookahead_distance`;
4. возвращает эту точку как waypoint.

Это лучше, чем следовать ближайшему следующему voxel, потому что UAV получает
стабильную цель впереди по path и не дергается от клетки к клетке.

### 6.6 Raw A* path без smoothing

Сглаживание отключено. Это сохраняет obstacle-avoidance структуру path. В лесу
это важно: сглаженный shortcut может привести к тому, что RBL waypoint окажется
на другой стороне дерева, и controller будет "упираться" в локальные constraints.

### 6.7 `best_g_score` и deviation penalty

`best_g_score` уменьшает лишние queue entries, поэтому A* быстрее завершает
поиск.

`deviationPenalty` стабилизирует path между replanning итерациями. Без него path
может прыгать между несколькими почти равными проходами, что проявляется как
дрожание waypoint и centroid.

### 6.8 Clearance penalty

A* не просто ищет кратчайший путь по свободным voxel. Он штрафует малый
clearance:

```text
safety_penalty = weight_safety / (clearance + eps)
```

Это отводит path от деревьев/препятствий и снижает риск, что RBL controller
получит waypoint слишком близко к границе `cell_A`.

### 6.9 `closestFreeIdx()`

Если start или goal попали в occupied voxel, replanner ищет ближайший свободный
voxel BFS-like обходом. Это помогает не провалиться сразу при шумной карте или
когда цель/позиция оказалась внутри inflated obstacle.

### 6.10 Добавление near-agent sensed cell

Даже при limited FOV в `createAndPartitionCellA()` в `sensed_cell_A` добавляется
локальная область вокруг UAV. Это снижает вероятность пустой actively sensed
cell и ситуации, когда centroid невозможно выбрать.

### 6.11 CIRI fallback

Если CIRI не построил polytope или вернул пустой `cell_A`, контроллер
переходит к classic partition. Это предотвращает остановку из-за неудачного
convex decomposition.

### 6.12 Boundary beta adaptation

Если centroid близко к границе safe cell, `beta` увеличивается и centroid
пересчитывается. Это отталкивает reference от границы и компенсирует tracking
errors / sensor uncertainty, как обсуждается в статье.

### 6.13 Limited FOV heading behavior

Если centroid оказался вне переднего сектора, controller не отправляет UAV
лететь боком. Он удерживает позицию, но задает heading на `c1_full`. Это дает
сенсору возможность увидеть релевантную область и затем продолжить движение.

## 7. Порядок запуска в `forest_real`

Основной сценарий:

```text
tmux/forest_real/session.yml
```

Важные windows:

- `router`: запускает `rmw_zenohd`;
- `core`: запускает `core_without_status.launch.py`;
- `status` и `status_proxy`: MRS status;
- `rbl_controller`: запускает `rbl_controller.launch.py`;
- `activation`: готовая команда activation service;
- `goto`: готовая команда goto service;
- `filter_reflective_uavs`: публикует соседние UAV states;
- `simulator`, `takeoff`, `hw_api`: MRS simulator / startup;
- `map_generator`: в текущей версии играет bag и запускает map TF helper;
- `rviz`: визуализация.

Команда rbl controller:

```text
ros2 launch rbl_controller_node rbl_controller.launch.py \
  custom_config:=$RBL_CONTROLLER_CONFIG \
  pcl_topic:=/uav2/losos_server/current_submap_pc
```

Чтобы включить A*, нужно выбрать config:

```bash
export RBL_CONTROLLER_CONFIG=./config/rbl_controller_astar.yaml
```

или передать этот файл в launch напрямую.

## 8. Основные зависимости

Внутренние зависимости:

```text
rbl_controller_node
  -> rbl_controller_core
  -> rbl_replanner
  -> ciri
```

`rbl_controller_node` также зависит от:

- `mrs_lib`;
- `mrs_msgs`;
- `rclcpp`, `rclcpp_components`;
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`;
- `std_msgs`, `std_srvs`;
- `octomap`, `octomap_msgs`;
- `PCL`, `pcl_conversions`;
- `filter_reflective_uavs`;
- `ouster_sensor_msgs`.

`rbl_controller_core` зависит от:

- `Eigen`;
- `PCL`;
- `mrs_msgs`;
- `rbl_replanner`;
- `ciri`.

`rbl_replanner` зависит от:

- `Eigen`;
- `PCL`.

`ciri` зависит от:

- `Eigen`.

`map_generator` зависит от:

- `rclcpp`;
- `std_msgs`;
- `geometry_msgs`;
- `nav_msgs`;
- `PCL`;
- `pcl_conversions`;
- `Eigen3`.

`rbl_rc` зависит от:

- `rclpy`;
- runtime ROS interfaces `mavros_msgs`, `mrs_msgs`, `nav_msgs`.

## 9. Важные параметры

### RBL controller

```yaml
rbl_controller:
  limited_fov: true
  ciri: true
  replanner: true/false
  radius: 5.0
  path_lookahead_distance: 3.0
  step_size: 0.2
  encumbrance: 0.6
  betaD: 0.3
  beta_min: 0.1
  boundary_threshold: 0.2
  boundary_threshold_speed: 0.01
  lidar_tilt: 20.0
  lidar_fov: 59.0
  pcl:
    downsample: true
    voxel_size: 0.2
```

`radius` и `path_lookahead_distance` не одно и то же:

- `radius` ограничивает локальную область RBL cells;
- `path_lookahead_distance` выбирает waypoint впереди по A* path.

### Replanner

```yaml
replanner:
  inflation_bonus: 0.4
```

Остальные replanner параметры сейчас задаются в constructor `RBLController`.
Для большей настраиваемости их стоит вынести в YAML.

## 10. Отличие текущей реализации от чистой статьи

Статья `2504.18840v3.pdf` описывает базовый distributed Lloyd-based алгоритм:

- безопасная cell geometry через CWVD;
- centroid-based control;
- адаптация `beta`;
- учет tracking error и uncertainty;
- flocking/proximity constraints;
- работа без explicit communication.

Репозиторий `irbl` сохраняет эту основу, но добавляет инженерные компоненты:

- 3D point cloud / octomap ingestion;
- CIRI convex decomposition;
- local A* replanner;
- async replanning;
- pure-pursuit lookahead waypoint;
- MRS service integration;
- visualization и benchmark events;
- real/simulation tmux launch infrastructure.

Локальный A* можно понимать как high-level replanner, который статья сама
называет полезным направлением для улучшения performance в сложной среде. Он не
заменяет safety proof RBL-ячейки, а дает более удачную промежуточную цель для
центроидного контроллера.

## 11. Типовой цикл работы системы

1. `tmux/session.yml` запускает MRS core, simulator/bag, controller и TF.
2. `rbl_controller` node загружает параметры.
3. Пользователь вызывает activation.
4. Пользователь задает goal через `/uav1/rbl_controller/goto`.
5. `WrapperRosRBL` в 20 Hz loop обновляет state и cloud.
6. `RBLController` запускает A* async, если replanner включен.
7. A* строит local path по inflated occupancy grid.
8. `RBLController` выбирает lookahead waypoint на path.
9. По waypoint строится RBL active cell.
10. Из active cell считается centroid.
11. Reference отправляется в MRS control manager.
12. Diagnostics публикуют path/cell/centroid/goal.
13. При движении path периодически пересчитывается или сохраняется, если
    replanning не требуется.

## 12. Практические замечания по отладке

Если UAV стоит на месте:

- проверить activation service;
- проверить, пришел ли PCL / Octomap;
- проверить, установлен ли goal;
- если replanner включен, проверить `~/path`;
- если `~/path` пустой, смотреть A* diagnostics: `No path found`,
  `skipped_occupied`, `skipped_oob`;
- проверить `inflation_bonus` и `encumbrance`: слишком большая inflation может
  закрыть все проходы;
- проверить frame `control_frame` и корректность transform odometry/cloud;
- проверить `limited_fov`: UAV может hold position, если centroid вне FOV, пока
  разворачивается heading;
- проверить `cell_A` и `actively_sensed_A`: пустая safe/sensed cell приведет к
  hold или centroid в текущей позиции;
- если CIRI часто падает, временно выключить `ciri` или смотреть fallback logs.

Если UAV дергается:

- увеличить `path_lookahead_distance`;
- проверить `weight_deviation` в replanner;
- убедиться, что path smoothing не включен обратно;
- проверить частоту replanner и стабильность cloud;
- проверить noisy group states: соседние UAV добавляются как obstacles.

Если UAV идет слишком близко к препятствиям:

- увеличить `inflation_bonus`;
- увеличить `weight_safety`;
- уменьшить `betaD`;
- проверить, что clearance grid считается и A* path не проходит по inflated
  cells.

## 13. Краткая карта функций

### `WrapperRosRBL`

- `initialize()`: параметры, pubs/subs/services/timers, создание controller.
- `cbTmSetRef()`: основной loop state -> cloud -> controller -> MRS reference.
- `cbTmDiagnostics()`: визуализация и benchmark goal reached.
- `cbSrvActivateControl()`: включает движение.
- `cbSrvDeactivateControl()`: отключает движение.
- `cbSrvGotoPosition()`: задает goal.
- `cbSrvSetBetaD()`: задает `betaD`.
- `updateGroupStates()`: переводит `PoseVelocityArray` в `State`.
- `addAgents2PCL()`: добавляет соседей в cloud как occupied points.

### `RBLController`

- `getNextRef()`: главный метод управления.
- `createAndPartitionCellA()`: строит `cell_S`, `cell_A`, `sensed_cell_A`.
- `partitionCellA()`: classic CWVD-like half-space partition.
- `partitionCellACiri()`: CIRI-based convex partition.
- `computeActivelySensedCell()`: limited-FOV filtering.
- `computeCentroid()`: weighted centroid.
- `computeScalarValue()`: weights around destination.
- `applyRules()`: adaptive `beta`, `theta`, `phi`.
- `determineWaypointFixedDistance()`: pure-pursuit waypoint on A* path.
- `determineNextRef()`: centroid/heading -> `mrs_msgs::msg::Reference`.

### `RBLReplanner`

- `plan()`: update grid, decide replan, run A*, return path.
- `shouldReplan()`: goal changed, path empty, progress, blocked path.
- `fillAndInflateGrid()`: PCL -> inflated occupancy voxel grid.
- `calculateClearanceGrid()`: occupancy -> clearance distance field.
- `AStarPlan()`: 26-connected voxel A* with clearance/deviation costs.
- `closestFreeIdx()`: recover start/goal if inside occupied voxel.
- `gridIdxToWorldCoords()` / `worldCoordsToGridIdx()`: coordinate conversion.

### `CIRI`

- `comvexDecomposition()`: build convex polytope around seed segment.
- `findEllipsoid()`: fit ellipsoid inside obstacle constraints.
- `findTangentPlaneOfSphere()`: produce separating tangent planes.
- `getPlaneData()`: expose planes to controller.

### `random_forest`

- `RandomMapGenerate()`: random box/ring obstacle generation.
- `RandomMapGenerateCylinder()`: random cylindrical forest generation.
- `rcvOdometryCallback()`: local cloud extraction around UAV.

### `rc_goal_controller`

- `rc_cb()`: RC channel processing.
- `handle_position()`: RC sticks -> local goal.
- `handle_beta()`: RC channel -> `betaD`.
- `start_estimator_switch()`: estimator switch request.
- `handle_post_switch_hold()`: hold after fresh estimator pose.
