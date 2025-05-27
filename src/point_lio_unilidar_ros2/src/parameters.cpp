#include "parameters.h"

bool is_first_frame = true;

double lidar_end_time = 0.0;
double first_lidar_time = 0.0;
double time_con = 0.0;

double last_timestamp_lidar = -1.0;
double last_timestamp_imu = -1.0;

int pcd_index = 0;

std::string lid_topic;
std::string imu_topic;

bool prop_at_freq_of_imu;
bool check_satu;
bool con_frame;
bool cut_frame;

bool use_imu_as_input;
bool space_down_sample;
bool publish_odometry_without_downsample;

int init_map_size;
int con_frame_num;

double match_s;
double satu_acc;
double satu_gyro;
double cut_frame_time_interval;

float plane_thr;

double filter_size_surf_min;
double filter_size_map_min;
double fov_deg;

double cube_len;
float DET_RANGE;

bool imu_en;
bool gravity_align;
bool non_station_start;

double imu_time_inte;

double laser_point_cov;
double acc_norm;

double vel_cov;
double acc_cov_input;
double gyr_cov_input;

double gyr_cov_output;
double acc_cov_output;
double b_gyr_cov;
double b_acc_cov;

double imu_meas_acc_cov;
double imu_meas_omg_cov;

int lidar_type;
int pcd_save_interval;

std::vector<double> gravity_init;
std::vector<double> gravity;

std::vector<double> extrinT;
std::vector<double> extrinR;

bool runtime_pos_log;
bool pcd_save_en;
bool path_en;
bool extrinsic_est_en = true;

bool scan_pub_en;
bool scan_body_pub_en;

shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;

void readParameters(const std::shared_ptr<rclcpp::Node> &node)
{
  p_pre = std::make_shared<Preprocess>();

  node->declare_parameter("prop_at_freq_of_imu", true);
  node->get_parameter("prop_at_freq_of_imu", prop_at_freq_of_imu);

  node->declare_parameter("use_imu_as_input", true);
  node->get_parameter("use_imu_as_input", use_imu_as_input);

  node->declare_parameter("check_satu", true);
  node->get_parameter("check_satu", check_satu);

  node->declare_parameter("init_map_size", 100);
  node->get_parameter("init_map_size", init_map_size);

  node->declare_parameter("space_down_sample", true);
  node->get_parameter("space_down_sample", space_down_sample);

  node->declare_parameter("mapping.satu_acc", 3.0);
  node->get_parameter("mapping.satu_acc", satu_acc);

  node->declare_parameter("mapping.satu_gyro", 35.0);
  node->get_parameter("mapping.satu_gyro", satu_gyro);

  node->declare_parameter("mapping.acc_norm", 1.0);
  node->get_parameter("mapping.acc_norm", acc_norm);

  node->declare_parameter("mapping.plane_thr", 0.05);
  node->get_parameter("mapping.plane_thr", plane_thr);

  node->declare_parameter("point_filter_num", 2);
  node->get_parameter("point_filter_num", p_pre->point_filter_num);

  node->declare_parameter("common.lid_topic", std::string("/livox/lidar"));
  node->get_parameter("common.lid_topic", lid_topic);

  node->declare_parameter("common.imu_topic", std::string("/livox/imu"));
  node->get_parameter("common.imu_topic", imu_topic);

  node->declare_parameter("common.con_frame", false);
  node->get_parameter("common.con_frame", con_frame);

  node->declare_parameter("common.con_frame_num", 1);
  node->get_parameter("common.con_frame_num", con_frame_num);

  node->declare_parameter("common.cut_frame", false);
  node->get_parameter("common.cut_frame", cut_frame);

  node->declare_parameter("common.cut_frame_time_interval", 0.1);
  node->get_parameter("common.cut_frame_time_interval", cut_frame_time_interval);

  node->declare_parameter("common.time_lag_imu_to_lidar", 0.0);
  node->get_parameter("common.time_lag_imu_to_lidar", time_lag_imu_to_lidar);

  node->declare_parameter("filter_size_surf", 0.5);
  node->get_parameter("filter_size_surf", filter_size_surf_min);

  node->declare_parameter("filter_size_map", 0.5);
  node->get_parameter("filter_size_map", filter_size_map_min);

  node->declare_parameter("cube_side_length", 200.0);
  node->get_parameter("cube_side_length", cube_len);

  node->declare_parameter("mapping.det_range", 300.0);
  node->get_parameter("mapping.det_range", DET_RANGE);

  node->declare_parameter("mapping.fov_degree", 180.0);
  node->get_parameter("mapping.fov_degree", fov_deg);

  node->declare_parameter("mapping.imu_en", true);
  node->get_parameter("mapping.imu_en", imu_en);

  node->declare_parameter("mapping.start_in_aggressive_motion", false);
  node->get_parameter("mapping.start_in_aggressive_motion", non_station_start);

  node->declare_parameter("mapping.extrinsic_est_en", true);
  node->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);

  node->declare_parameter("mapping.imu_time_inte", 0.005);
  node->get_parameter("mapping.imu_time_inte", imu_time_inte);

  node->declare_parameter("mapping.lidar_meas_cov", 0.1);
  node->get_parameter("mapping.lidar_meas_cov", laser_point_cov);

  node->declare_parameter("mapping.acc_cov_input", 0.1);
  node->get_parameter("mapping.acc_cov_input", acc_cov_input);

  node->declare_parameter("mapping.vel_cov", 20.0);
  node->get_parameter("mapping.vel_cov", vel_cov);

  node->declare_parameter("mapping.gyr_cov_input", 0.1);
  node->get_parameter("mapping.gyr_cov_input", gyr_cov_input);

  node->declare_parameter("mapping.gyr_cov_output", 0.1);
  node->get_parameter("mapping.gyr_cov_output", gyr_cov_output);

  node->declare_parameter("mapping.acc_cov_output", 0.1);
  node->get_parameter("mapping.acc_cov_output", acc_cov_output);

  node->declare_parameter("mapping.b_gyr_cov", 0.0001);
  node->get_parameter("mapping.b_gyr_cov", b_gyr_cov);

  node->declare_parameter("mapping.b_acc_cov", 0.0001);
  node->get_parameter("mapping.b_acc_cov", b_acc_cov);

  node->declare_parameter("mapping.imu_meas_acc_cov", 0.1);
  node->get_parameter("mapping.imu_meas_acc_cov", imu_meas_acc_cov);

  node->declare_parameter("mapping.imu_meas_omg_cov", 0.1);
  node->get_parameter("mapping.imu_meas_omg_cov", imu_meas_omg_cov);

  node->declare_parameter("preprocess.blind", 1.0);
  node->get_parameter("preprocess.blind", p_pre->blind);

  node->declare_parameter("preprocess.lidar_type", 1);
  node->get_parameter("preprocess.lidar_type", lidar_type);

  node->declare_parameter("preprocess.scan_line", 16);
  node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);

  node->declare_parameter("preprocess.scan_rate", 10);
  node->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);

  node->declare_parameter("preprocess.timestamp_unit", 1);
  node->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);

  node->declare_parameter("mapping.match_s", 81.0);
  node->get_parameter("mapping.match_s", match_s);

  node->declare_parameter("mapping.gravity_align", true);
  node->get_parameter("mapping.gravity_align", gravity_align);

  node->declare_parameter("mapping.gravity", std::vector<double>{});
  node->get_parameter("mapping.gravity", gravity);

  node->declare_parameter("mapping.gravity_init", std::vector<double>{});
  node->get_parameter("mapping.gravity_init", gravity_init);

  node->declare_parameter("mapping.extrinsic_T", std::vector<double>{});
  node->get_parameter("mapping.extrinsic_T", extrinT);

  node->declare_parameter("mapping.extrinsic_R", std::vector<double>{});
  node->get_parameter("mapping.extrinsic_R", extrinR);

  node->declare_parameter("odometry.publish_odometry_without_downsample", false);
  node->get_parameter("odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);

  node->declare_parameter("publish.path_en", true);
  node->get_parameter("publish.path_en", path_en);

  node->declare_parameter("publish.scan_publish_en", true);
  node->get_parameter("publish.scan_publish_en", scan_pub_en);

  node->declare_parameter("publish.scan_bodyframe_pub_en", true);
  node->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);

  node->declare_parameter("runtime_pos_log_enable", false);
  node->get_parameter("runtime_pos_log_enable", runtime_pos_log);

  node->declare_parameter("pcd_save.pcd_save_en", false);
  node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);

  node->declare_parameter("pcd_save.interval", -1);
  node->get_parameter("pcd_save.interval", pcd_save_interval);

} // end of readParameters
