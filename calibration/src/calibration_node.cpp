#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <group4_msgs/PointCloudPose.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/features/normal_3d.h>

const double epsilon = 1e-6;
const double maxCorrespondanceDistance = 0.01;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointXYZ PointT;

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

Eigen::Matrix4f get_ICP_align(pcl::PointCloud<PointT>& source, pcl::PointCloud<PointT>& target, const double& epsilon, const double& maxCorrespondanceDistance)
{
    // http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration

    pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalSource (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalTarget (new pcl::PointCloud<pcl::PointNormal>);



    pcl::NormalEstimation<PointT, pcl::PointNormal> normalEstimate;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    normalEstimate.setSearchMethod(tree);
    normalEstimate.setKSearch(30);

    normalEstimate.setInputCloud(source.makeShared());
    normalEstimate.compute(*pointsWithNormalSource);
    pcl::copyPointCloud(source, *pointsWithNormalSource);

    normalEstimate.setInputCloud (target.makeShared());
    normalEstimate.compute(*pointsWithNormalTarget);
    pcl::copyPointCloud (target, *pointsWithNormalTarget);

    //	Instantiate custom point representation
    MyPointRepresentation pointNormal;
    //	... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    pointNormal.setRescaleValues(alpha);

    //	Align
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> registration;
    registration.setTransformationEpsilon(epsilon);
    //	Set the maximum distance between two correspondences
    registration.setMaxCorrespondenceDistance(maxCorrespondanceDistance);
    //	Set the point representation
    registration.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (pointNormal));

    registration.setInputSource(pointsWithNormalSource);
    registration.setInputTarget(pointsWithNormalTarget);
    registration.setMaximumIterations(30);

    PCL_ERROR("Source size: %d  --  Target size: %d\n", (int)pointsWithNormalSource.get()->size(), (int)pointsWithNormalTarget.get()->size());

    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointNormal>::Ptr regResult = pointsWithNormalSource;

    PCL_ERROR("Stitching ... ");
    registration.align(*regResult);
    PCL_ERROR("Done!\n");

    tf = registration.getFinalTransformation().inverse();

    return tf;
}


void receive_pointcloud(const sensor_msgs::PointCloud2 pc)
{
    static bool init = 0;
    static pcl::PointCloud<PointT> last_p;

    pcl::PointCloud<PointT> cur_p;
    pcl::fromROSMsg(pc, cur_p);

    if (0 == init)
    {
        last_p = cur_p;
        return;
    }
    std::cout << "Starting ICP ...";
    Eigen::Matrix4f tf = get_ICP_align(last_p, cur_p, epsilon, maxCorrespondanceDistance);
    std::cout << tf;

}

void data_callback(const group4_msgs::PointCloudPosePtr data)
{
    receive_pointcloud(data->carmine_pointcloud);
    //pcl::fromROSMsg(*data, this->input.points);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle node_handle("~");

    //ros::Publisher cameratate_pub = node_handle.advertise<group4_msgs::PointCloudPose>("/robot_rx60b/camerapose", 5);
    ros::Subscriber data_sub = node_handle.subscribe("/robot_rx60b/bumblebee_pose",1,&data_callback);

    ros::spin();

    return 0;
}
