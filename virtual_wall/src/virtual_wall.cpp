#include <pluginlib/class_list_macros.h>
#include <virtual_wall/virtual_wall.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>

PLUGINLIB_EXPORT_CLASS(virtual_wall::VirtualWall, costmap_2d::Layer)

using namespace std;
 
namespace virtual_wall {
// 初始化变量及虚拟墙坐标点保存路径
VirtualWall::VirtualWall() {
    cout << "VirtualWall()" << endl;
    wallmax_x = 0.0;
    wallmax_y = 0.0;
    wallmin_x = 0.0;
    wallmin_y = 0.0;
    next_wall_id = 0;
    save_file_path_ = ros::package::getPath("virtual_wall") + "/virtual_walls.txt";
}

VirtualWall::~VirtualWall() {
    cout << "~VirtualWall()" << endl;
}

void VirtualWall::onInitialize(){
    boost::unique_lock<boost::recursive_mutex> lock(data_access_);
    ros::NodeHandle g_nh;
    nh = ros::NodeHandle("~/" + name_);
    matchSize();
    current_ = true;
    enabled_ = true;
    add_wall_sub_ = g_nh.subscribe("/clicked_point", 1, &VirtualWall::AddWallCallback, this);
    wall_list_pub = g_nh.advertise<std_msgs::Int32>("/virtual_wall_list", 1);
    delete_wall_sub = g_nh.subscribe("/delete_wall", 1, &VirtualWall::DeleteWallCallback, this);
    wall_maker_pub = g_nh.advertise<visualization_msgs::Marker>("virtual_wall_vis", 1);
    // 初始化定时器，延时调用 loadMapCallback，为了可以让虚拟墙在打开的时候可以准确显示
    load_map_timer_ = nh.createTimer(ros::Duration(0.5), &VirtualWall::triggerLoadMapCallback, this, true);
    save_map_sub = g_nh.subscribe("/virtual_wall_save", 1,  &VirtualWall::SaveMapCallback, this);

}

void VirtualWall::triggerLoadMapCallback(const ros::TimerEvent&) {LoadMapCallback();}

void VirtualWall::matchSize() {
    boost::unique_lock<boost::recursive_mutex> lock(data_access_);
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    resolution = master->getResolution();
    // 获取局部代价地图的global_frame
    global_frame_ = layered_costmap_->getGlobalFrameID();
    cout << "global_frame_ : " << global_frame_ << endl;
    map_frame_ = "map";
}
// 更新代价地图边界
void VirtualWall::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y) {
    *min_x = std::min(wallmin_x, *min_x);
    *min_y = std::min(wallmin_y, *min_y);
    *max_x = std::max(wallmax_x, *max_x);
    *max_y = std::max(wallmax_y, *max_y);

}
// 更新代价地图 -- 只需要给Virtual_wall_point设置代价地图即可，不需要给marker也增加，不然会产生闪烁现象
// 该函数通过给定的区域更新代价地图中的虚拟墙区域，并设置为致命障碍物(LETHAL_OBSTACLE)
void VirtualWall::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    // 锁住数据访问，保证线程安全
    boost::unique_lock<boost::recursive_mutex> lock(data_access_); 
    // 如果虚拟墙使用的是地图框架，直接更新代价地图类型，否则先进行坐标转换再更新代价地图类型
    if (global_frame_ == map_frame_) {
        for (const auto& wall : wallPoint) {
            // 仅更新墙体的中间插值点，避免过多的更新导致闪烁
            for (size_t i = 1; i < wall.polygon.points.size() - 1; ++i) {  
                unsigned int pixle_x, pixle_y;
                // 将墙体的插值点坐标转换为代价地图的栅格坐标
                if (master_grid.worldToMap(wall.polygon.points[i].x, wall.polygon.points[i].y, pixle_x, pixle_y)) {
                    // 设置该位置为致命障碍物
                    master_grid.setCost(pixle_x, pixle_y, costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }
    } else {
        geometry_msgs::TransformStamped transform;
        try {
            // 坐标系变换
            transform = tf_->lookupTransform(global_frame_, map_frame_, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("Failed to get TF transform: %s", ex.what());
            return;
        }
        // 将变换数据转换为 tf2 格式
        tf2::Transform tf2_transform;
        tf2::convert(transform.transform, tf2_transform);

        for (const auto& wall : wallPoint) {
            for (size_t i = 1; i < wall.polygon.points.size() - 1; ++i) {
                // 将每个墙体的插值点坐标转换为世界坐标系下的坐标
                tf2::Vector3 tf_p(wall.polygon.points[i].x, wall.polygon.points[i].y, 0);
                // 使用坐标变换将墙体坐标转换为地图坐标系下的坐标
                tf2::Vector3 map_p = tf2_transform * tf_p;

                unsigned int pixle_x, pixle_y;
                // 将转换后的地图坐标转换为代价地图的栅格坐标
                if (master_grid.worldToMap(map_p.x(), map_p.y(), pixle_x, pixle_y)) {
                    // 设置该位置为致命障碍物
                    master_grid.setCost(pixle_x, pixle_y, costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }
    }

    // marker分开发布
    publishMarkers();
}
void VirtualWall::publishMarkers(){
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = map_frame_;
    node_vis.header.stamp = ros::Time::now();
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;
    node_vis.scale.x = resolution;
    node_vis.scale.y = resolution;
    node_vis.scale.z = 0.05;

    geometry_msgs::Point pt;
    std_msgs::Int32 msg_id;
    pt.z = 0.05;
    for (const auto& wall : wallPoint) {
        msg_id.data = wall.id;
        wall_list_pub.publish(msg_id); // 发布虚拟墙ID
        for (const auto& p : wall.polygon.points) {
            pt.x = p.x;
            pt.y = p.y;
            node_vis.points.push_back(pt);
        }
    }

    wall_maker_pub.publish(node_vis); 
}

void VirtualWall::AddWallCallback(const geometry_msgs::PointStampedConstPtr& msg) {

    std::cout << "AddWallCallback()" << std::endl;
    global_frame_ = layered_costmap_->getGlobalFrameID();
    cout << "global_frame_ : " << global_frame_ << endl;

    geometry_msgs::Point32 point;

    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = msg->point.z;
    wallmax_x = std::max(wallmax_x, msg->point.x);
    wallmax_y = std::max(wallmax_y, msg->point.y);
    wallmin_x = std::min(wallmin_x, msg->point.x);
    wallmin_y = std::min(wallmin_y, msg->point.y);

    if (temp_wall.polygon.points.empty()) {
        temp_wall.polygon.points.push_back(point);
    } else {
        temp_wall.polygon.points.push_back(point);
        // 必须是push两个point才可生成虚拟墙
        if (temp_wall.polygon.points.size() == 2) {
            virtual_wall::Wall wall;
            wall.id = next_wall_id++; // 给虚拟墙分配唯一ID
            wall.polygon.points.push_back(temp_wall.polygon.points[0]);
            wall.polygon.points.push_back(temp_wall.polygon.points[1]);
            GenerateWallBetweenPoints(temp_wall.polygon.points[0], temp_wall.polygon.points[1], wall); // 生成虚拟墙
            v_wall.push_back(wall);
            wallPoint.push_back(wall);
            temp_wall.polygon.points.clear();  // 清空 temp_wall 以便下一次添加新的墙
        }
    }
    updateWallPointsVector();
}
// 根据ID号来删除相应虚拟墙
void VirtualWall::DeleteWallCallback(const std_msgs::Int32ConstPtr& msg) {
    for (size_t i = 0; i < v_wall.size(); i++) {
        if (v_wall[i].id == msg->data) {
            v_wall.erase(v_wall.begin() + i);
            wallPoint.erase(wallPoint.begin() + i);
            break;   
        }
    }
    std::cout << "Delete Virtua_Wall SUCESS!" << endl;
    updateWallPointsVector();
    updateCosts(*layered_costmap_->getCostmap(), 0, 0, layered_costmap_->getCostmap()->getSizeInCellsX(), layered_costmap_->getCostmap()->getSizeInCellsY());
}
// 保存虚拟墙，包括ID，两端的坐标点
void VirtualWall::SaveMapCallback(const std_msgs::EmptyConstPtr& msg) {
    std::ofstream ofs(save_file_path_);
    if (!ofs.is_open()) {
        ROS_ERROR("Failed to open file for saving: %s", save_file_path_.c_str());
        return;
    }
    if (v_wall.empty()) {
        // 如果虚拟墙列表为空，重置 next_wall_id 为 0
        next_wall_id = 0;
        ofs.close();
        ROS_INFO("No virtual walls to save, reset next_wall_id to 0");
        return;
    }

    for (const auto& wall : v_wall) {
        std::cout << "Start to writing virtual walls!" << std::endl;
        ofs << static_cast<int>(wall.id) << " "    // txt 文件只支持UTF-8格式数字,不支持uint8_t,存储时转换一下
            << wall.polygon.points[0].x << " " << wall.polygon.points[0].y << " "
            << wall.polygon.points[1].x << " " << wall.polygon.points[1].y << "\n";
    }
    ofs.close();
    ROS_INFO("Saved virtual walls to %s", save_file_path_.c_str());
}
// 读取Virtual_wall.txt文件，显示Virtual_wall
void VirtualWall::LoadMapCallback() {
    std::ifstream ifs(save_file_path_);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open file for loading: %s", save_file_path_.c_str());
        return;
    }

    wallPointsVector.clear();
    v_wall.clear();
    wallPoint.clear();
    int id;
    int max_id = -1;
    geometry_msgs::Point32 point1, point2;
    while (ifs >> id >> point1.x >> point1.y >> point2.x >> point2.y) {
        virtual_wall::Wall wall;
        wall.id = static_cast<uint8_t>(id); // 读取的时候将格式从 UTF-8 转回 uint8_t
        wall.polygon.points.push_back(point1);
        wall.polygon.points.push_back(point2);
        GenerateWallBetweenPoints(point1, point2, wall);
        v_wall.push_back(wall);
        wallPoint.push_back(wall);
        if (id > max_id) {
            max_id = id; // 更新最大ID
        }
    }
    // 如果加载的虚拟墙列表非空，更新 next_wall_id
    next_wall_id = max_id + 1;
    ifs.close();
    updateWallPointsVector();
    // 更新代价地图
    updateCosts(*layered_costmap_->getCostmap(), 0, 0, layered_costmap_->getCostmap()->getSizeInCellsX(), layered_costmap_->getCostmap()->getSizeInCellsY());
}
// 在click_point两点之间线性插入坐标点生成Virtual_wall
void VirtualWall::GenerateWallBetweenPoints(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2, virtual_wall::Wall& wall) {
    float delta_x = p2.x - p1.x;
    float delta_y = p2.y - p1.y;
    float distance = sqrt(delta_x * delta_x + delta_y * delta_y); //两点之间欧几里德距离
    int steps = static_cast<int>(distance / resolution); // 插值点的数量
    // 计算插值点的坐标
    for (int i = 0; i <= steps; ++i) {
        geometry_msgs::Point32 interpolated_point;
        double t = i / static_cast<double>(steps); // 比例因子t ,从 0 到 1 线性变化
        interpolated_point.x = p1.x + delta_x * t ;  
        interpolated_point.y = p1.y + delta_y * t ;
        wall.polygon.points.push_back(interpolated_point);
    }

}
// 更新存储坐标点的容器
void VirtualWall::updateWallPointsVector() {
    wallPointsVector.clear();
    for (const auto& wall : wallPoint) {
        for (const auto& point : wall.polygon.points) {
            wallPointsVector.push_back(point);
        }
    }
    updateCosts(*layered_costmap_->getCostmap(), 0, 0, layered_costmap_->getCostmap()->getSizeInCellsX(), layered_costmap_->getCostmap()->getSizeInCellsY());
}

}  // namespace virtual_wall
