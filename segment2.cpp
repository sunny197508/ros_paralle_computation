#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <Eigen/Dense>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <dynamic_reconfigure/server.h>
#include <pts/DynamicParamsConfig.h>
unsigned int message_process_num=0;

#include <unistd.h>
#include <queue>

#define DEBUG 1
#define MAX_CPU_NUM 32
#define MAX_OUT_NUM 10 //Maximum output message of one thread
typedef void* (*pFunction)(void*); 
class cloudHandler;

struct mix_param {
	std::queue<sensor_msgs::PointCloud2>* q_in;
	std::queue<sensor_msgs::PointCloud2>* (*q_out)[MAX_CPU_NUM];
	int cpu_num;
	};

struct mix_out_param {
	cloudHandler* ch;
	int num;
	}; 

void* para_process(void* arg) ;
void* pPublish(void* arg);
//typedef int (*pPub)(const int num);

using namespace std;
	
	double pass_through_min_ = 0.0;
	double pass_through_max_ = 4.0;
	int outlier_removal_meank_ = 10;
	double outlier_removal_stddev_mul_thresh_ = 0.2;
	double voxel_filter_leaf_x_ = 0.01;
	double voxel_filter_leaf_y_ = 0.01;
	double voxel_filter_leaf_z_ = 0.01;
	int ransac_max_iter_num_ = 1000;
	double ransac_dist_threshold_ = 0.05;
	int normal_estim_ksearch_ = 50;
	int region_growing_min_cluster_size_ = 50;
	int region_growing_max_cluster_size_ = 1000000;
	int region_growing_neighb_num_ = 30;
	double region_growing_smooth_degree_threshold_ = 3.0;
	double region_growing_curv_threshold_ = 1.0;
	int min_red_ = 46;
	int max_red_ = 255;
	int min_green_ = 0;
	int max_green_ = 30;
	int min_blue_ = 0;
	int max_blue_ = 30;
    //pcl::visualization::CloudViewer viewer("viewer");
    
class par_exec
{
public:
	  par_exec(pFunction pf,int output_num){
	  	  int i,j;
	  	  int res;
	  	  cur_cpu = 0;
	      //cpu_num = sysconf(_SC_NPROCESSORS_ONLN);
	      //cout << "CPU number is " << cpu_num << endl;
	      cpu_num = 4;
	      struct mix_param mp[cpu_num];
	      for(i=0;i<cpu_num;i++)
	          in_queue[i] = new std::queue<sensor_msgs::PointCloud2>;
	      for(i=0;i<output_num;i++)
		      for(j=0;j<cpu_num;j++)
	              out_queue[i][j] = new std::queue<sensor_msgs::PointCloud2>;	 
	      for(i=0;i<cpu_num;i++){
	      	  mp[i].q_in  = in_queue[i];
	      	  mp[i].q_out = out_queue;
	      	  mp[i].cpu_num = i;
	          //if(pthread_create(&tidp[i],NULL,pf,(void*)in_queue[i]) == -1) {
	          res = pthread_create(&tidp[i],NULL,pf,(void*)&mp[i]);
	          cout << "res is " << res << endl;
	          if(res != 0) 
	              cout << "Create thread " << i << " failed" << endl; 
	          else
	          	  cout << "Create thread " << i << " success" << endl;
	      }
	                	
	  }
	  
	  int in_proc(const sensor_msgs::PointCloud2& msg) {
	  	  static int i = 0;
	  	  if(cur_cpu == cpu_num)
	  	  	  cur_cpu = 0;
	  	  in_queue[cur_cpu]->push(msg);
	  	  
	  	  cur_cpu++;
	  	  #ifdef DEBUG
          i++;
	  	  cout << "Total number of input package is " << i << endl;
	  	  #endif
	  }	

    sensor_msgs::PointCloud2 out_proc(int number) {
    	  int i;
    	  int bitmap = 0;
    	  int early_frame = 0;
    	  ros::Time stamp = ros::Time::now();
    	  sensor_msgs::PointCloud2 msg;
    	  
    	  while(1) {
            for(i=0;i<cpu_num;i++){
                if(out_queue[number][i] -> empty() != 1){
                	  bitmap |= 1<<i;
                      if(out_queue[number][i] -> front().header.stamp < stamp )	{
                      	  stamp = out_queue[number][i] -> front().header.stamp;  
                          early_frame = i;	 
                      }
                }
            }
            if(bitmap != 0)
                break;  
            usleep(10);
        }
        msg = (sensor_msgs::PointCloud2&)out_queue[number][early_frame]->front();
        out_queue[number][early_frame]->pop();
        return msg;
        
    }
   
    int cpu_num;
    int cur_cpu;
	  pthread_t tidp[MAX_CPU_NUM];
	  std::queue<sensor_msgs::PointCloud2>* in_queue[MAX_CPU_NUM];
	  std::queue<sensor_msgs::PointCloud2>* out_queue[MAX_OUT_NUM][MAX_CPU_NUM];

private:	  	
	  	
};

class cloudHandler
{
public:
		//par_exec *parallel((cloudHandler*)this,para_process,3); 
    cloudHandler(){
    int num;
		
		param_init();
        pcl_sub = nh.subscribe("/camera/depth/color/points", 10, &cloudHandler::cloudCB_filter, this);
        
        foregroud_pub = nh.advertise<sensor_msgs::PointCloud2>("foregroud_pub", 1);
        voxel_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_filter_pub", 1);
        pass_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("pass_filter_pub", 1);
        stat_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("stat_filter_pub", 1);
        color_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("color_filter_pub", 1);
        ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
        coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
		parallel = new par_exec(para_process,3);    
		//par_exec parallel((pFunction)para_process,(int)3); 
		
		mop[0].num = 0;
		mop[0].ch = this;
		pthread_create(&pot[0],NULL,pPublish,(void*)&mop[0]);
		mop[1].num = 1; 
		mop[1].ch = this;
		pthread_create(&pot[1],NULL,pPublish,(void*)&mop[1]);
		mop[2].num = 2;
		mop[2].ch = this;
		pthread_create(&pot[2],NULL,pPublish,(void*)&mop[2]);
		//
		//m_timer = nh.createTimer(ros::Duration(1), &cloudHandler::OnTimerOut,  this);//0.1s
    
    }
	
	void param_init()
	{
		pass_through_min_ = 0.0;
		pass_through_max_ = 2.5;
		outlier_removal_meank_ = 10;
		outlier_removal_stddev_mul_thresh_ = 0.2;
		voxel_filter_leaf_x_ = 0.01;
		voxel_filter_leaf_y_ = 0.01;
		voxel_filter_leaf_z_ = 0.01;
		ransac_max_iter_num_ = 1000;
		ransac_dist_threshold_ = 0.05;
		normal_estim_ksearch_ = 50;
		region_growing_min_cluster_size_ = 50;
		region_growing_max_cluster_size_ = 1000000;
		region_growing_neighb_num_ = 30;
		region_growing_smooth_degree_threshold_ = 3.0;
		region_growing_curv_threshold_ = 1.0;

		viewer_flag = false;
	}

    void cloudCB_filter(const sensor_msgs::PointCloud2& input)
    {
        parallel->in_proc(input);
    }

/*
    void cloudCB_filter(const sensor_msgs::PointCloud2& input)
    {
        struct timespec t1,t2;
		ROS_INFO("\n\n---------------------------");
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&t1);
        printf("The begin time of his CB process is %ds %dns",t1.tv_sec,t1.tv_nsec);
        pcl::PointCloud<pcl::PointXYZ> cloud;
		sensor_msgs::PointCloud2 color_output;
        pcl::PointCloud<pcl::PointXYZ> pass_filtered,cloud_filtered,cloud_downsampled;
	
		color_filter(input, color_output, pass_through_min_, pass_through_max_, min_red_, max_red_, 
					 min_green_, max_green_, min_blue_, max_blue_);

        pcl::fromROSMsg(color_output, cloud);
#if 1
        //passFilter_filter(cloud,cloud);
#endif
        statFilter_filter(cloud,cloud);
        voxelFilter_filter(cloud,cloud);
        
#if 0
        ransac_segment(cloud);
#endif
	message_process_num++;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&t2);
        printf("The end time of his CB process is %ds %dns,last %dns",t2.tv_sec,t2.tv_nsec,t2.tv_nsec - t1.tv_nsec);
        printf("Receive message number is %d\n",message_process_num);
    }
*/  
    
#if 1

	#endif
    void passFilter_filter(const pcl::PointCloud<pcl::PointXYZ> &input,  pcl::PointCloud<pcl::PointXYZ> &output)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (input.makeShared());            
        pass.setFilterFieldName ("z");         
        pass.setFilterLimits (pass_through_min_, pass_through_max_);
		//ROS_INFO("pass_through_min_=%f, pass_through_max_=%f", pass_through_min_, pass_through_max_);
		//pass.setFilterLimits (0.0, 2.5);        
        pass.filter (output);       

		sensor_msgs::PointCloud2 pass_filter_output;
        pcl::toROSMsg(output, pass_filter_output);
        pass_filter_pub.publish(pass_filter_output);
    }
	//离群点去除
	  //void statFilter_filter(const pcl::PointCloud<pcl::PointXYZ> &input,  pcl::PointCloud<pcl::PointXYZ> &output)

	
    //void voxelFilter_filter(const pcl::PointCloud<pcl::PointXYZ> &input,  pcl::PointCloud<pcl::PointXYZ> &output)

    void ransac_segment(const  pcl::PointCloud<pcl::PointXYZ>  &cloud)
    {
      
        pcl::PointCloud<pcl::PointXYZ> cloud_segmented_backgroud,cloud_segmented_foregroud;
        pcl::ModelCoefficients coefficients;//
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setOptimizeCoefficients(true);//ke
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
		segmentation.setMaxIterations(ransac_max_iter_num_);
        //segmentation.setMaxIterations(1000);
		segmentation.setDistanceThreshold(ransac_dist_threshold_);
        //segmentation.setDistanceThreshold(0.05);
        segmentation.setInputCloud(cloud.makeShared());
        segmentation.segment(*inliers, coefficients);


        pcl_conversions::fromPCL(coefficients, ros_coefficients);
        coef_pub.publish(ros_coefficients);

        pcl_msgs::PointIndices ros_inliers;
        pcl_conversions::fromPCL(*inliers, ros_inliers);
        ind_pub.publish(ros_inliers);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        
        extract.setNegative(true);
        extract.filter(cloud_segmented_foregroud);


        //statFilter_filter(cloud_segmented_foregroud,cloud_segmented_foregroud);
        //region_growing(cloud_segmented_foregroud);//cloud_segmented_foregroud

        sensor_msgs::PointCloud2 fore_output;
        pcl::toROSMsg(cloud_segmented_foregroud, fore_output);
        foregroud_pub.publish(fore_output);
       

    }

    void region_growing(pcl::PointCloud<pcl::PointXYZ> cloud)
    {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud.makeShared());
		
		normal_estimator.setKSearch (normal_estim_ksearch_);
        //normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);
	
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize (region_growing_min_cluster_size_);
        //reg.setMinClusterSize (50);
		reg.setMaxClusterSize (region_growing_max_cluster_size_);
        //reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (region_growing_neighb_num_);
        //reg.setNumberOfNeighbours (30);
        reg.setInputCloud (cloud.makeShared());
        //reg.setIndices (indices);
        reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (region_growing_smooth_degree_threshold_ / 180.0 * M_PI);
        //reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
		reg.setCurvatureThreshold (region_growing_curv_threshold_);
        //reg.setCurvatureThreshold (1.0);
        
        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
		reg_ = reg;
		viewer_flag = true;
    }
#if 0
	void OnTimerOut(const ros::TimerEvent& event) 
	{
		ROS_INFO("in OnTimerOut");
		if(viewer_flag)
		{
        pcl::PointCloud <pcl::PointXYZ>::Ptr colored_cloud = reg_.getColoredCloud();
        viewer.showCloud(colored_cloud);
		}
     	#if 0
	 	while (!viewer.wasStopped ())
        {
            reg_ = reg;
        }
		#endif
	}
#endif
	par_exec *parallel; 
	ros::Publisher foregroud_pub,ind_pub, coef_pub, voxel_filter_pub, stat_filter_pub, pass_filter_pub, color_filter_pub;
  struct mix_out_param mop[3];
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    
    pcl_msgs::ModelCoefficients ros_coefficients;
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg_;
	ros::Timer m_timer;
	bool viewer_flag;    
       
	pthread_t pot[3];
        //unsigned int message_process_num=0;
};

void callback(pts::DynamicParamsConfig &config) 
{
#if 0
  		ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
			//g_int_param=config.int_param;
#endif
		ROS_INFO("in callback");
		pass_through_min_ = config.pass_through_min;
		pass_through_max_ = config.pass_through_max;
		outlier_removal_meank_ = config.outlier_removal_meank;
		outlier_removal_stddev_mul_thresh_ = config.outlier_removal_stddev_mul_thresh;
		voxel_filter_leaf_x_ = config.voxel_filter_leaf_x;
		voxel_filter_leaf_y_ = config.voxel_filter_leaf_y;
		voxel_filter_leaf_z_ = config.voxel_filter_leaf_z;
		ransac_max_iter_num_ = config.ransac_max_iter_num;
		ransac_dist_threshold_ = config.ransac_dist_threshold;
		normal_estim_ksearch_ = config.normal_estim_ksearch;
		region_growing_min_cluster_size_ = config.region_growing_min_cluster_size;
		region_growing_max_cluster_size_ = config.region_growing_max_cluster_size;
		region_growing_neighb_num_ = config.region_growing_neighb_num;
		region_growing_smooth_degree_threshold_ = config.region_growing_smooth_degree_threshold;
		region_growing_curv_threshold_ = config.region_growing_curv_threshold;
		min_red_ = config.min_red;
		max_red_ = config.max_red;
		min_green_ = config.min_green;
		max_green_ = config.max_green;
		min_blue_ = config.min_blue;
		max_blue_ = config.max_blue;
}

void color_filter(const sensor_msgs::PointCloud2& input,
	              sensor_msgs::PointCloud2 &color_output,				
				  int min_distance, int max_distance,
				  int min_red, int max_red, 
				  int min_green, int max_green, 
				  int min_blue, int max_blue)
{
	unsigned int data_size = input.data.size();
	unsigned int data_width = input.width;
	int unit_size = data_size / data_width;
	pcl::PointIndices color_indices;
	std::vector<int> index;
	for(unsigned int i = 0; i < data_width; ++i)
	{
		float distance = 0.0f;
		unsigned char* p = (unsigned char*)&distance;
		*(p + 0) = (unsigned char)input.data[unit_size * i + 8];
		*(p + 1) = (unsigned char)input.data[unit_size * i + 9];
		*(p + 2) = (unsigned char)input.data[unit_size * i + 10];
		*(p + 3) = (unsigned char)input.data[unit_size * i + 11];
		if(distance >= min_distance && distance <= max_distance)
		{
			int red = (unsigned char)input.data[unit_size * i + 18];
			int green = (unsigned char)input.data[unit_size * i + 17];
			int blue = (unsigned char)input.data[unit_size * i + 16];
			int a = (unsigned char)input.data[unit_size * i + 19];
			if(red >= min_red && red <= max_red &&
		   	green >= min_green && green <= max_green &&
		   	blue >= min_blue && blue <= max_blue
			&& red > green && red > blue)
			{
				//ROS_INFO("id:%d, red:%d, green:%d, blue:%d, a:%d", i, red, green, blue, a);
				index.push_back(i);
			}
		}
	}
      pcl::PointCloud<pcl::PointXYZ> cloud, output;
      pcl::fromROSMsg(input, cloud);
	
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud.makeShared());

	boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
	extract.setIndices(index_ptr);
	extract.setNegative(false);
	extract.filter(output);
	
      pcl::toROSMsg(output, color_output);
      //color_filter_pub.publish(color_output);
}

sensor_msgs::PointCloud2  statFilter_filter(const pcl::PointCloud<pcl::PointXYZ> &input,  pcl::PointCloud<pcl::PointXYZ> &output)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    statFilter.setInputCloud(input.makeShared());
statFilter.setMeanK(outlier_removal_meank_);
    //statFilter.setMeanK(10);
    //statFilter.setStddevMulThresh(0.2);
statFilter.setStddevMulThresh(outlier_removal_stddev_mul_thresh_);
    statFilter.filter(output);

sensor_msgs::PointCloud2 stat_filter_output;
    pcl::toROSMsg(output, stat_filter_output);
    return stat_filter_output;
    //stat_filter_pub.publish(stat_filter_output);
}

sensor_msgs::PointCloud2 voxelFilter_filter(const pcl::PointCloud<pcl::PointXYZ> &input,  pcl::PointCloud<pcl::PointXYZ> &output)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(input.makeShared());
voxelSampler.setLeafSize(voxel_filter_leaf_x_, voxel_filter_leaf_y_, voxel_filter_leaf_z_);
    //voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelSampler.filter(output);
sensor_msgs::PointCloud2 voxel_filter_output;
    pcl::toROSMsg(output, voxel_filter_output);
    return voxel_filter_output;
    //voxel_filter_pub.publish(voxel_filter_output);
}
   
//int para_process(queue<sensor_msgs::PointCloud2>* pq) {
void* para_process(void* arg) {	
	int i;
    int thread_num = 0;
	struct mix_param* mp;
	std::queue<sensor_msgs::PointCloud2>* q_in;
	std::queue<sensor_msgs::PointCloud2>* (*q_out)[MAX_CPU_NUM];
	int cpu_num;
	pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 color_output;
    sensor_msgs::PointCloud2 stat_filter_output;
    sensor_msgs::PointCloud2 voxel_filter_output;
    pcl::PointCloud<pcl::PointXYZ> pass_filtered,cloud_filtered,cloud_downsampled; 
    mp = (struct mix_param*)arg;
    q_in  = mp->q_in;
    q_out = mp->q_out;
    cpu_num = mp->cpu_num;
    int found = 0;
    //pq = (queue<sensor_msgs::PointCloud2>*)arg;
	cout << "Begin the " << cpu_num <<"th thread process " << endl;
   
    while(1) {
        while(q_in->empty() == 1)
            usleep(20);	
            
        const sensor_msgs::PointCloud2 input = q_in->front();
        q_in->pop();           
        color_filter(input, color_output, pass_through_min_, pass_through_max_, min_red_, max_red_, 
		    	 min_green_, max_green_, min_blue_, max_blue_);
        q_out[0][cpu_num]->push(color_output);
        pcl::fromROSMsg(color_output, cloud);
        stat_filter_output = statFilter_filter(cloud,cloud);
        q_out[1][cpu_num]->push(stat_filter_output);
        voxel_filter_output = voxelFilter_filter(cloud,cloud);
        q_out[2][cpu_num]->push(voxel_filter_output);    
    }
} 

void* pPublish(void* arg)
{   
	struct mix_out_param* mop;
	mop = (struct mix_out_param*)arg;
	int num;
	cloudHandler* ch;
	num = mop->num;
	ch = mop->ch;
	sensor_msgs::PointCloud2 msg;
	int pkts = 0;
    while(1) {
        msg = ch->parallel->out_proc(num);
		#ifdef DEBUG
		pkts++;
        cout << "Total packages send by channel " << num << " is " << pkts << endl;
		#endif
        if(num == 0)
            ch->color_filter_pub.publish(msg);
        else if(num == 1)
        	ch->stat_filter_pub.publish(msg);	
        else if(num == 2)
        	ch->voxel_filter_pub.publish(msg);
    }	
	
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segment2");

#if 0
    dynamic_reconfigure::Server<pts::DynamicParamsConfig> server;
    dynamic_reconfigure::Server<pts::DynamicParamsConfig>::CallbackType f;

    f = boost::bind(&callback, _1);
    server.setCallback(f);
#endif
	cloudHandler *handler = new cloudHandler();

   	ros::Rate loop_rate(10);
	while(ros::ok()) 
   	{
		ros::spin();
		loop_rate.sleep();
	}
    return 0;
}