# ros虚拟墙插件
借鉴源码：https://github.com/DylanLN/virtual_wall.git

使用rviz插件"Publish Point"点击两个点即可自动连成一条虚拟墙，Publish Point发布话题为/clicked_point

改进：

    增加虚拟墙保存功能 -- rostopic pub /virtual_wall_save std_msgs/Empty {}"
    修改删除功能 -- rostopic pub  /delete_wall std_msgs/Int32 '0'  根据虚拟墙的id号来进行相应虚拟墙的删除 '0'-id


插件安装：

    下载该功能包放入Navigation功能包，使用catkin_make编译工作空间
    使用rospack plugins --attrib=plugin costmap_2d命令，终端出现virtual_wall .../Virtual_wall/costmap_plugins.xml，便说明已经安装成功了，可以作为一个地图插件来使用了

插件使用：

    进入你自己的导航功能包navigation中，找到自己的代价地图参数配置文件夹
    在global_costmap_params.yaml和local_costmap_params.yaml文件末尾，加入插件参数
    plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::VoxelLayer"}
    - {name: virtual_layer, type: "virtual_wall::VirtualWall"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

    PS：virtual_layer的顺序要在inflation_layer膨胀层插件上面
  
使用过程：

    1、启动robot并打开导航
    2、使用rviz插件"Publish Point"点击两个点即可自动连成一条虚拟墙。
    3、发布rostopic pub /virtual_wall_save std_msgs/Empty {}"保存虚拟墙
    4、关闭导航，再次打开即可显示虚拟墙，也可实现避障和导航
    5、根据虚拟墙创建的顺序添加id号， 发布rostopic pub /delete_wall std_msgs/Int32 '0' 命令，根据id自由删除相对应虚拟墙 

仿真环境 -- turtlebot3 
版本 -- ros-noetic

遇到问题：

    一、在初次使用插件的时候，代价地图一直不显示
        1、使用Cartographer定位，在launch文件中把map_server节点关闭掉了，没有加载map.yaml文件
        2、没有把cartographer_occupancy_grid_node节点关闭掉
        3、在确保launch文件和插件的参数格式没有问题后，尝试更改插件顺序试试，要确保virtual_layer插件在inflation_layer上面
        
    二、删除虚拟墙的时候，会有虚拟墙的代价地图残留在地图上，实则是已经删掉了
        原因1：loadmapcallback函数在onInitialize初始化函数中加载
        解决方法：在 onInitialize初始化函数中添加一个定时器，在初始化完成后短暂延时后调用 loadMapCallback 问题解决
        原因2：全局代价地图更新频率太低

    三、ros1的turtlebot3，在cartographer建图节点处把odom话题重映射为了odom_tf，在使用cartographer做重定位的时候，记得把remap注释掉，要保证odom话题一致

    四、局部代价地图外的marker虚拟墙组件一直闪烁 -- 尚未找到原因
        提高全局代价地图更新频率会有效果,但仍然闪烁 
        原因：local_costmap.yaml的global_frame不能是map，得是odom
        
        新问题：
            实车的global_frame修改为odom之后，导航无法前进，局部路径规划坐标系对不上 -- 因为如果是odom的话实车定位效果很差，故还是要用map

        当local_costmap的global_frame为map时：
            修改程序逻辑，将marker与插入坐标点分开单独发布
            修改Savemap函数逻辑，优化虚拟墙ID的准确性与唯一性

















