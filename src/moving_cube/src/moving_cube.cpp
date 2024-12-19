#include <iostream>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

class fixed_obstacle 
{
public:
    fixed_obstacle()
    {
        genFixedCircleCloudMap();
    };
    ~fixed_obstacle(){};
    
    pcl::PointCloud<pcl::PointXYZ> fixed_cloud_map;

    void genFixedCircleCloudMap()
    {
        pcl::PointXYZ pt_random;
        double _resolution{0.1};

        for (int i = 0; i < 4; ++i)
        {
            double x, y, z, theta;
            switch (i)
            {
                case 0:
                    x = 0; y = 8; z = 2;
                    theta = 0;
                    break;
                case 1:
                    x = 2; y = 6; z = 1;
                    theta = 1.5;
                    break;
                case 2:
                    x = 0; y = -8; z = 2;
                    theta = 0;
                    break;
                case 3:
                    x = -8; y = 0; z = 1;
                    theta = 1.5;
                    break;
                default:
                    break;
            }

            x = floor(x / _resolution) * _resolution + _resolution / 2.0;
            y = floor(y / _resolution) * _resolution + _resolution / 2.0;
            z = floor(z / _resolution) * _resolution + _resolution / 2.0;
            Eigen::Vector3d translate(x, y, z);
            Eigen::Matrix3d rotate;
            rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;
            double radius1 = 0.5;
            double radius2 = 0.5;

            // draw a circle centered at (x,y,z)
            Eigen::Vector3d cpt;
            for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
            {
                cpt(0) = 0.0;
                cpt(1) = radius1 * cos(angle);
                cpt(2) = radius2 * sin(angle);

                // inflate
                Eigen::Vector3d cpt_if;
                for (int ifx = -0; ifx <= 0; ++ifx)
                    for (int ify = -0; ify <= 0; ++ify)
                    for (int ifz = -0; ifz <= 0; ++ifz)
                    {
                        cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution, ifz * _resolution);
                        cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                        pt_random.x = cpt_if(0);
                        pt_random.y = cpt_if(1);
                        pt_random.z = cpt_if(2);
                        fixed_cloud_map.push_back(pt_random);
                    }
            }
        }
        fixed_cloud_map.width = fixed_cloud_map.points.size();
        fixed_cloud_map.height = 1;
        fixed_cloud_map.is_dense = true;
    }
};

class moving_cube
{
public:
    enum moving_type{
        STOP, 
        LINE,
        CIRCLE
    };

private:
    Eigen::Vector2d pos_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d vel_{Eigen::Vector2d::Zero()};
    ros::Time t_last_update_{ros::Time(0)}, t_last_change_;
    bool line_last_state_{0};
    double t_duration{1.0};
    moving_type move_type_{STOP};
    Eigen::Vector2d init_point1_, init_point2_;
    double _resolution;

    pcl::PointCloud<pcl::PointXYZ> cloudMap;

public:
    moving_cube(Eigen::Vector2d initPoint1, Eigen::Vector2d initPoint2, moving_type moveType):
    init_point1_(initPoint1),
    init_point2_(initPoint2),
    move_type_(moveType)
    {
        pos_ = init_point2_;
        cloudGenerate();
    };
    ~moving_cube(){};

    void cloudGenerate()
    {
        pcl::PointXYZ pt_random;
        double x{pos_[0]},y{pos_[1]};
        double w{1.0};
        _resolution = 0.05;

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        int widNum = ceil(w / _resolution);
        double radius = w / 2;

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
        {
            double h = 3.0;
            int heiNum = ceil(h / _resolution);
            for (int t = -10; t < heiNum; t++)
            {
                double temp_x = x + (r + 0.5) * _resolution + 1e-2;
                double temp_y = y + (s + 0.5) * _resolution + 1e-2;
                double temp_z = (t + 0.5) * _resolution + 1e-2;
                if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
                {
                    pt_random.x = temp_x;
                    pt_random.y = temp_y;
                    pt_random.z = temp_z;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }
        cloudMap.width = cloudMap.points.size();
        cloudMap.height = 1;
        cloudMap.is_dense = true;
    }
    
    void updateCallback()
    {
        cloudMap.points.clear();
        ros::Time t_now = ros::Time::now();
        if(t_last_update_ == ros::Time(0))
        {
            t_last_update_ = t_now;
            t_last_change_ = t_now;
        }
        double delta_t = (t_now - t_last_update_).toSec();
        
        // ROS_INFO("delta_t : %f", delta_t);
        vel_update(delta_t);
        pos_update(delta_t);
        cloudGenerate();
        t_last_update_ = t_now;
    }

    void vel_update(double delta_t)
    {
        switch(move_type_)
        {
            case STOP:
                break;
            case LINE:
            {   
                Eigen::Vector2d vel = (init_point1_ - init_point2_) / t_duration;
                ros::Time t_now = ros::Time::now();
                if ((t_now - t_last_change_).toSec() > t_duration)
                {
                    if(line_last_state_) line_last_state_ = false;
                    else line_last_state_ = true;
                    t_last_change_ = t_now;
                }
                if (line_last_state_) vel_ = -vel;
                else vel_ = vel;
                break;
            }
            case CIRCLE:
            {
                
                double radius = (init_point1_ - init_point2_).norm(); 
                double angular_velocity = 1 / t_duration; 
                
                
                if(!line_last_state_) 
                {
                    /* vel 单位向量 */
                    Eigen::Vector2d vel;
                    vel = (init_point1_ - init_point2_) / (init_point1_ - init_point2_).norm();
                    /* vel 逆时针旋转90度 */
                    double d = vel[0];
                    vel[0] = vel[1];
                    vel[1] = - d;
                    /* vel_ 实际速度 */
                    vel_ = vel * angular_velocity * radius;
                    line_last_state_ = true;
                }
                
                double angle = angular_velocity * delta_t;  
                Eigen::Matrix2d rotation_matrix;

                /* 逆时针旋转矩阵 */
                /*  cos  -sin
                    sin   cos   */
                rotation_matrix << cos(angle), -sin(angle), sin(angle), cos(angle); 
                vel_ = rotation_matrix * vel_;  
            }
            default:
                break;
        }
    }
    void pos_update(double delta_t)
    {
        pos_ += vel_ * delta_t;
    }


    inline const pcl::PointCloud<pcl::PointXYZ>& getCloudMap() const
    {
        return cloudMap;
    }
};

class obstacles
{
private:
    fixed_obstacle fixed_circle;
    std::vector<moving_cube> cube_vec;
    int cub_num;
    pcl::PointCloud<pcl::PointXYZ> cloud_map, fixed_cloud;
    
    sensor_msgs::PointCloud2 globalMap_pcd;
    sensor_msgs::PointCloud2 localMap_pcd;
    ros::Timer update_timer;
    ros::Publisher _local_map_pub;
    ros::Publisher _all_map_pub;
    int _map_ok{false};
public:
    obstacles(int cubNum, ros::NodeHandle &nh):
    cub_num(cubNum)
    {
        fixed_cloud.points.insert(fixed_cloud.points.end(), 
                                fixed_circle.fixed_cloud_map.points.begin(), fixed_circle.fixed_cloud_map.points.end());
        cube_vec.reserve(cubNum);
        update_timer = nh.createTimer(ros::Duration(0.01), &obstacles::updateCallback, this);
        _local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
        _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
    };
    ~obstacles(){};
    void updateCallback(const ros::TimerEvent&)
    {
        cloud_map.points.clear();
        cloud_map.points.insert(cloud_map.points.end(), fixed_cloud.begin(), fixed_cloud.end());
        for(int i = 0; i < cub_num; i++)
        {
            cube_vec[i].updateCallback();
            cloud_map.points.insert(cloud_map.points.end(), cube_vec[i].getCloudMap().points.begin(), cube_vec[i].getCloudMap().points.end());
        }
        cloud_map.width = cloud_map.points.size();
        cloud_map.height = 1;
        cloud_map.is_dense = true;
        _map_ok = true;
    }

    void pubPoints()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            if(_map_ok) break;
        }
        pcl::toROSMsg(cloud_map, globalMap_pcd);
        globalMap_pcd.header.frame_id = "world";
        _all_map_pub.publish(globalMap_pcd);
    }

    void push_back(moving_cube &cube)
    {
        cube_vec.push_back(cube);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "moving_cube");
    ros::NodeHandle nh("~");
    obstacles cubes(3, nh);
    //LINE   : 1:goal              2:   init pos
    //CIRCLE : 1:center of circle  2：  init pos 
    moving_cube cube1(Eigen::Vector2d {-4, 4},Eigen::Vector2d {-6, 6}, moving_cube::moving_type::LINE);
    moving_cube cube2(Eigen::Vector2d {1, 0},Eigen::Vector2d {3, 0}, moving_cube::moving_type::LINE);
    moving_cube cube3(Eigen::Vector2d {-4, -4},Eigen::Vector2d {-5, -5}, moving_cube::moving_type::LINE);
    // moving_cube cube4(Eigen::Vector2d {0, 8}, Eigen::Vector2d {-2, 8}, moving_cube::moving_type::CIRCLE);
    cubes.push_back(cube1);
    cubes.push_back(cube2);
    cubes.push_back(cube3);
    // cubes.push_back(cube4);
    while (ros::ok())
    {
        cubes.pubPoints();
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
}