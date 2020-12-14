#include <cstdio>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include <linux/videodev2.h>

#include "indemind_driver/DriverInterface.h"

// TODO: Add QoS

struct IndemindDriverNode : public rclcpp::Node {
    IndemindDriverNode(const std::string &name)
         : Node(name, rclcpp::NodeOptions()){}
};

std::shared_ptr<IndemindDriverNode> node;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf;

void publish_static_transforms();
void camera_callback(indem::cameraData *data);
void imu_callback(indem::IMUData *data);
void hmd_hotplug_callback(bool bArrive);

void print_module_flash(ModuleParamInFlash<1> module_param);
void print_module_parameters(ModuleParameters param);
void print_camera_parameters(CameraParameter param);
void print_imu_parameters(IMUParameter param);
void print_module_info(ModuleInfo info);

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::SingleThreadedExecutor executor;
    // auto node = std::make_shared<IndemindDriverNode>("indemind_test");
    node = std::make_shared<IndemindDriverNode>("indemind_driver_node");

    int ret = 0;
    int width = 1280;     //单图像宽度 640 1280
    int height = 800;     //单图像高  400 800
    int fps = 100;        //帧率 25 50 100 200
    int imufreq = 1000;   //imu频率
    int version = 255;    //代码固件版本号，初始值任意
    size_t info_size = 0; //获取到的参数总长度
    unsigned char *module_info = new unsigned char[FLASH_MAX_SIZE];
    ModuleParamInFlash<1> module_flash_data = {0}; //标定参数等信息等
    indem::IDriverInterface *driver = DriverFactory();
    enum indem::IMAGE_RESOLUTION plan = indem::RESOLUTION_1280;
    //获取标定参数等信息等
    ret = driver->GetModuleParams(version, module_info, info_size);
    if (ret != true) {
        printf("Get params faild\n");
    } else {
        memcpy(&module_flash_data, module_info, info_size);
    }
    print_module_flash(module_flash_data);

    //打开设备
    ret = driver->Open(imufreq, fps, plan);
    ret = 0;
    if (ret < 0) {
        printf("Open device err\n");
    } else {
        publish_static_transforms();

        camera_pub = node->create_publisher<sensor_msgs::msg::Image>("/indemind/camera/image_raw", 100);
        // printf("address of pub: 0x%" PRIXPTR "\n",
        //   reinterpret_cast<std::uintptr_t>(camera_pub.get()));
        driver->SetCameraCallback(camera_callback);

        imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/indemind/imu", 1000);
        driver->SetIMUCallback(imu_callback);
        
        SetHotplugCallback(hmd_hotplug_callback);
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void print_module_flash(ModuleParamInFlash<1> module_param)
{
    ModuleParameters params = module_param._parent;
    int img_frequency = module_param._imgFrequency;
    int img_resolution = module_param._imgResolution;
    int imu_frequency = module_param._imuFrequency;

    printf("#### Current Settings #####\n")
    printf("img_frequency\t%d\n", img_frequency);
    printf("img_resolution\t%d\n", img_resolution);
    printf("imu_frequency\t%d\n", imu_frequency);
    print_module_parameters(params);
    
    return;
}

void print_module_parameters(ModuleParameters param)
{
    CameraParameter left_cam_param  = param._camera[0];
    CameraParameter right_cam_param = param._camera[1];
    IMUParameter    imu_param       = param._imu;
    ModuleInfo      module_info     = param._device;
    SlamParameter   slam_param      = param._slam;

    printf("\n#### Left Camera Parameters ####\n");
    print_camera_parameters(left_cam_param);
    printf("\n#### Right Camera Parameters ####\n");
    print_camera_parameters(right_cam_param);
    printf("\n#### IMU Parameters ####\n");
    print_imu_parameters(imu_param);
    printf("\n#### Module Info ####\n");
    print_module_info(module_info);

    return;
}

void print_camera_parameters(CameraParameter param)
{
    printf("Matrix T_SC:\n");
    for (size_t i = 0; i<16; i++) {
        printf("%f\t", param._TSC[i]);
        if (3==i || 7==i || 11==i) {
            printf("\n");
        }
    }

    printf("\n\nResolution:\n%d x %d\n\n", param._width, param._height);

    printf("Distortion type: equidistant\n\n");

    printf("fx, fy:\n(%f, %f)\n\n", param._focal_length[0], param._focal_length[1]);

    printf("Rotation Matrix R:\n");
    for (size_t i = 0; i<9; i++) {
        printf("%f\t", param._R[i]);
        if (2==i || 5==i) {
            printf("\n");
        }
    }

    printf("\n\nProjection Matrix P:\n");
    for (size_t i = 0; i<12; i++) {
        printf("%f\t", param._P[i]);
        if (3==i || 7==i) {
            printf("\n");
        }
    }

    printf("\n\nCamera Matrix K:\n");
    for (size_t i = 0; i<9; i++) {
        printf("%f\t", param._K[i]);
        if (2==i || 5==i) {
            printf("\n");
        }
    }

    printf("\n\nDistortion Vector D:\n");
    for (size_t i = 0; i<4; i++) {
        printf("%f\t", param._D[i]);
    }
    printf("\n\n");
    return;
}

void print_imu_parameters(IMUParameter param)
{
    printf("a_max\t%f\ng_max\t%f\n", param._a_max, param._g_max);
    printf("simga_a_c\t%f\nsigma_g_c\t%f\n", param._sigma_a_c, param._sigma_g_c);
    printf("sigma_ba\t%f\nsigma_bg\t%f\n", param._sigma_ba, param._sigma_bg);
    printf("sigma_aw_c\t%f\nsigma_gw_c\t%f\n", param._sigma_aw_c, param._sigma_gw_c);
    printf("tau\t%f\n", param._tau);
    printf("g\t%f\n", param._g);
    printf("a0\t%f\t%f\t%f\t%f\n", param._a0[0], param._a0[1], param._a0[2], param._a0[3]);
    printf("T_BS:\n");
    for (size_t i = 0; i<16; i++) {
        printf("%f\t", param._T_BS[i]);
        if (3==i || 7==i || 11==i) {
            printf("\n");
        }
    }
    printf("\nAcc:\n");
    for (size_t i = 0; i<12; i++) {
        printf("%f\t", param._Acc[i]);
    }
    printf("\nGyr:\n");
    for (size_t i = 0; i<12; i++) {
        printf("%f\t", param._Gyr[i]);
    }
    return;
}

void print_module_info(ModuleInfo info)
{
    printf("\nid:\t");
    printf(info._id);
    printf("\ndesigner:\t");
    printf(info._designer);
    printf("\nfireware version:\t");
    printf(info._fireware_version);
    printf("\nhardware version:\t");
    printf(info._hardware_version);
    printf("\ncamera module:\t");
    printf(info._lens);
    printf("\nIMU module:\t");
    printf(info._imu);
    printf("\nviewing angle:\t");
    printf(info._viewing_angle);
    printf("\nbaseline length:\t");
    printf(info._baseline);
    printf("\n");
}

rclcpp::Time hardware_time_to_software_time(double hardware_time)
{
    static bool is_software_time_initialized = false;
    static rclcpp::Time software_time_begin;
    static double hardware_time_begin(0);

    if (false == is_software_time_initialized) {
      software_time_begin = node->get_clock()->now();
      hardware_time_begin = hardware_time;
      is_software_time_initialized = true;
    }
    double runtime_milisec = hardware_time - hardware_time_begin;     

    return software_time_begin + rclcpp::duration<int64_t, std::nano>(
                                rclcpp::nanoseconds(
                                static_cast<int64_t>(runtime_milisec*1e6)));
}

void publish_static_transforms()
{   
    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(node);

    rclcpp::Time tf_time = node->get_clock()->now();
    geometry_msgs::msg::TransformStamped base_to_cam;
    base_to_cam.header.stamp    = tf_time;
    base_to_cam.header.frame_id = "indemind_link";
    base_to_cam.child_frame_id  = "indemind_cam";

    static_tf_broadcaster.sendTransform(base_to_cam);
    return;
}

void camera_callback(indem::cameraData *data)
{
    auto msg = sensor_msgs::msg::Image();
    // assert(data->_height * data->_width * data->_channel == data->_size);
    msg.header.stamp    = hardware_time_to_software_time(data->_timeStamp);
    msg.header.frame_id = "indemind_cam";
    
    msg.height  = data->_height;
    msg.width   = data->_width;
    msg.data    = std::vector<unsigned char>(data->_image, data->_image + data->_size);
    msg.step    = data->_width;
    msg.encoding = sensor_msgs::image_encodings::MONO8;

    // assert(camera_pub);
    // printf("address of pub in cb: 0x%" PRIXPTR "\n",
    //       reinterpret_cast<std::uintptr_t>(camera_pub.get()));
    camera_pub->publish(msg);
    // printf("Published one frame\n");
    return;
}

void imu_callback(indem::IMUData *data)
{
    auto msg = sensor_msgs::msg::Imu();

    msg.header.frame_id = "indemind_imu_frame";  
    msg.header.stamp = hardware_time_to_software_time(data->_timeStamp);
    msg.angular_velocity.x = data->_gyr[0];
    msg.angular_velocity.y = data->_gyr[1];
    msg.angular_velocity.z = data->_gyr[2];
    msg.linear_acceleration.x = data->_acc[0];
    msg.linear_acceleration.y = data->_acc[1];
    msg.linear_acceleration.z = data->_acc[2];
    
    imu_pub->publish(msg);
    return;
}

void hmd_hotplug_callback(bool bArrive)
{
    if (bArrive) {
        printf("device connected\n");
    } else {
        printf("device disconnected\n");
    }
    return;
}
