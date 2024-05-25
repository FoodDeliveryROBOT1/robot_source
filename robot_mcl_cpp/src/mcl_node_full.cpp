#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_broadcaster.h"  
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tf2/convert.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <robot_mcl_cpp/Pose.h>
#include <robot_mcl_cpp/Particle.h>

using namespace std::chrono_literals;


namespace robot_mcl_cpp {
class MCL_Node : public rclcpp::Node {
public :
    // inline setting functions
    inline void setCanUpdateScan(bool canUpdateScan) { canUpdateScan_ = canUpdateScan; }
    inline void setDeltaX(double deltaX) { deltaX_ = deltaX; }
    inline void setDeltaY(double deltaY) { deltaY_ = deltaY; }
    inline void setDeltaDist(double deltaDist) { deltaDist_ = deltaDist; }
    inline void setDeltaYaw(double deltaYaw) { deltaYaw_ = deltaYaw; }
    inline void setDeltaXSum(double deltaXSum) { deltaXSum_ = deltaXSum; }
    inline void setDeltaYSum(double deltaYSum) { deltaYSum_ = deltaYSum; }
    inline void setDeltaDistSum(double deltaDistSum) { deltaDistSum_ = deltaDistSum; }
    inline void setDeltaYawSum(double deltaYawSum) { deltaYawSum_ = deltaYawSum; }
    inline void setParticlePose(int i, double x, double y, double yaw) { particles_[i].setPose(x, y, yaw); }
    inline void setParticleW(int i, double w) { particles_[i].setW(w); }
    inline void setUseOdomMsg(bool useOdomMsg) { useOdomMsg_ = useOdomMsg; }
    inline void setTotalLikelihood(double totalLikelihood) { totalLikelihood_ = totalLikelihood; }
    inline void setAverageLikelihood(double averageLikelihood) { averageLikelihood_ = averageLikelihood; }
    inline void setMaxLikelihood(double maxLikelihood) { maxLikelihood_ = maxLikelihood; }
    inline void setMaxLikelihoodParticleIdx(int maxLikelihoodParticleIdx) { maxLikelihoodParticleIdx_ = maxLikelihoodParticleIdx; }

    // inline getting functions
    inline double getLocalizationHz(void) { return localizationHz_; }
    inline double getDeltaX(void) { return deltaX_; }
    inline double getDeltaY(void) { return deltaY_; }
    inline double getDeltaDist(void) { return deltaDist_; }
    inline double getDeltaYaw(void) { return deltaYaw_; }
    inline double getDeltaXSum(void) { return deltaXSum_; }
    inline double getDeltaYSum(void) { return deltaYSum_; }
    inline double getDeltaDistSum(void) { return deltaDistSum_; }
    inline double getDeltaYawSum(void) { return deltaYawSum_; }

    inline std::vector<double> getOdomNoiseDDM(void) { return odomNoiseDDM_; }
    inline Pose getMCLPose(void) { return mclPose_; }
    inline Pose getBaseLink2Laser(void) { return baseLink2Laser_; }
    inline int getParticlesNum(void) { return particlesNum_; }
    inline Pose getParticlePose(int i) { return particles_[i].getPose(); }
    inline double getParticleW(int i) { return particles_[i].getW(); }
    inline std::vector<int> getResampleIndices(void) { return resampleIndices_; }
    inline double getResampleThresholdESS(void) { return resampleThresholdESS_; }
    inline double getEffectiveSampleSize(void) { return effectiveSampleSize_; }
    
    
    MCL_Node(): 
        Node("MCL_Node_Run"),

        scanName_("scan"),
        odomName_("/odom_ekf"),
        mapName_("/map"),
        poseName_("/mcl_pose"),
        particlesName_("/mcl_particles"),
        unknownScanName_("/unknown_scan"),
        residualErrorsName_("/residual_errors"),
        laserFrame_("laser"),
        baseLinkFrame_("base_footprint"),
        mapFrame_("map"),
        odomFrame_("odom"),
        useOdomFrame_(true),

        initialPose_({0.0, 0.0, 0.0}),
        particlesNum_(3000),
        initialNoise_({0.2, 0.2, 0.02}),
        useAugmentedMCL_(false),
        addRandomParticlesInResampling_(true),
        randomParticlesRate_(0.1),
        randomParticlesNoise_({0.1, 0.1, 0.01}),

        odomNoiseDDM_({0.4, 0.2, 0.4, 0.2}),

        deltaXSum_(0.0),
        deltaYSum_(0.0),
        deltaDistSum_(0.0),
        deltaYawSum_(0.0),
        deltaTimeSum_(0.0),
        resampleThresholds_({-1.0, -1.0, -1.0, -1.0, -1.0}),


        useOdomMsg_(true),
        use_cmd_vel_(true),

        measurementModelType_(0),
        pKnownPrior_(0.5),
        pUnknownPrior_(0.5),
        scanStep_(10),
        zHit_(0.9),
        zShort_(0.2),
        zMax_(0.05),
        zRand_(0.05),
        varHit_(0.1),
        lambdaShort_(3.0),  // change
        lambdaUnknown_(1.0),
        alphaSlow_(0.001),
        alphaFast_(0.9),
        omegaSlow_(0.0),
        omegaFast_(0.0),
        rejectUnknownScan_(false),
        publishUnknownScan_(false),
        publishResidualErrors_(false),
        resampleThresholdESS_(0.5),
        maxResidualError_(1.0f),
        minValidResidualErrorNum_(10),
        meanAbsoluteErrorThreshold_(0.5f),
        failureCntThreshold_(10),
        useEstimationResetting_(false),
        localizationHz_(10.0),
        performGlobalLocalization_(false),   // test   // every paramete here need to tune!!!!
        broadcastTF_(true), 
        gotMap_(false),
        gotScan_(false),
        isInitialized_(true),
        canUpdateScan_(true),
        initial_mcl_ready_(false)
    {
        // set seed for the random values based on the current time
        srand((unsigned int)time(NULL));
        pUnknownPrior_ = 1.0 - pKnownPrior_;

        // broad_cast
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Define custom QoS
        rclcpp::QoS custom_qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
        custom_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);  // Set reliability to Reliable
        custom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        // Set up subscribers
        scanSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scanName_, custom_qos, std::bind(&MCL_Node::scanCB, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odomName_, 10, std::bind(&MCL_Node::odomCB, this, std::placeholders::_1));
        mapSub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            mapName_, 20, std::bind(&MCL_Node::mapCB, this, std::placeholders::_1));
        initialPoseSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1, std::bind(&MCL_Node::initialPoseCB, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MCL_Node::cmd_vel_CB, this, std::placeholders::_1));

        // Set up publishers
        posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            poseName_, 1);
        particlesPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            particlesName_, 1); 
        unknownScanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            unknownScanName_, 1);
        residualErrorsPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            residualErrorsName_, 1);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = create_wall_timer(100ms, std::bind(&MCL_Node::timer_callback, this));

                // set initial pose
        mclPose_.setPose(initialPose_[0], initialPose_[1], initialPose_[2]);
        prevMCLPose_ = mclPose_;
        odomPose_.setPose(0.0, 0.0, 0.0);
        deltaX_ = deltaY_ = deltaDist_ = deltaYaw_ = 0.0;

        geometry_msgs::msg::TransformStamped transform_msg;
        rclcpp::Rate loop_rate(10);  // 10 Hz loop rate
        while (rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for 100 milliseconds before publishing again
            auto now = rclcpp::Clock().now();
            try {
                // bool success = tf_buffer_->canTransform(
                //     baseLinkFrame_, laserFrame_, now);
                if ( true) {
                    transform_msg = tf_buffer_->lookupTransform(laserFrame_,baseLinkFrame_, now ,10s);
                    break;  // Exit loop on successful retrieval
                }
            } catch (const tf2::LookupException& ex) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Lookup transform error: " << ex.what());
                RCLCPP_ERROR_STREAM(this->get_logger(), "Did you set the static transform publisher between " << baseLinkFrame_ << " to " << laserFrame_ << "?");
            } catch (const tf2::ConnectivityException& ex) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Connectivity error: " << ex.what());
            } catch (const tf2::ExtrapolationException& ex) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Extrapolation error: " << ex.what());
            }
        }
        RCLCPP_INFO(this->get_logger(), "got TF of laser");
        geometry_msgs::msg::Quaternion quat = transform_msg.transform.rotation;
        tf2::Quaternion ros_quat(quat.w, quat.x, quat.y, quat.z);  // Convert ROS msg to tf2 Quaternion

        // Create a tf2::Matrix3x3 to store rotation matrix
        tf2::Matrix3x3 rot_mat(ros_quat);

        // Extract roll, pitch, yaw angles from the rotation matrix
        double roll, pitch, yaw;
        rot_mat.getRPY(roll, pitch, yaw);

        // Extract position data from transform message
        double x = transform_msg.transform.translation.x;
        double y = transform_msg.transform.translation.y;

        // Assuming 'baseLink2Laser_' is a custom message or variable to store the data
        baseLink2Laser_.setX(x);
        baseLink2Laser_.setY(y);
        baseLink2Laser_.setYaw(yaw);
    }
    double normalize(double z)
    {
        return atan2(sin(z), cos(z));
    }
    double pf_ran_gaussian(double sigma){
        double x1, x2, w, r;

        do {
            do {
            r = drand48();
            } while (r == 0.0);
            x1 = 2.0 * r - 1.0;
            do {
            r = drand48();
            } while (r == 0.0);
            x2 = 2.0 * r - 1.0;
            w = x1 * x1 + x2 * x2;
        } while (w > 1.0 || w == 0.0);

        return sigma * x2 * sqrt(-2.0 * log(w) / w);
    }

    double angle_diff(double a, double b)
    {
        a = normalize(a);
        b = normalize(b);
        double d1 = a - b;
        double d2 = 2 * M_PI - fabs(d1);
        if (d1 > 0) {
            d2 *= -1.0;
        }
        if (fabs(d1) < fabs(d2)) {
            return d1;
        } else {
            return d2;
        }
    }

    void updateParticlesByMotionModel(void) {
        double deltaX, deltaY, deltaDist, deltaYaw;

        double dx, dy;
        if (useOdomMsg_) {

            deltaDist = deltaDist_ * (1/localizationHz_ ) ;
            deltaYaw = deltaYaw_ * (1/localizationHz_ ) ;

            double old_yaw = mclPose_.getYaw();

            dx = deltaDist * cos(old_yaw);
            dy = deltaDist * sin(old_yaw);
            
            // reset
            deltaDist_ = deltaYaw_ = deltaDist = 0.0;
        } else {
            // estimate robot's moving using the linear interpolation of the estimated pose
            dx = mclPose_.getX() - prevMCLPose_.getX();
            dy = mclPose_.getY() - prevMCLPose_.getY();
            double dyaw = mclPose_.getYaw() - prevMCLPose_.getYaw();
            while (dyaw < -M_PI)
                dyaw += 2.0 * M_PI;
            while (dyaw > M_PI)
                dyaw -= 2.0 * M_PI;
            double t = -(atan2(dy, dx) - prevMCLPose_.getYaw());
            deltaX = dx * cos(t) - dy * sin(t);
            deltaY = dx * sin(t) + dy * cos(t);
            deltaYaw = dyaw;
            deltaDist = sqrt(dx * dx + dy * dy);
            if (dx < 0.0)
                deltaDist *= -1.0;   
            prevMCLPose_ = mclPose_;

            // calculate odometry
            t = odomPose_.getYaw() + deltaYaw / 2.0;
            double x = odomPose_.getX() + deltaDist * cos(t);
            double y = odomPose_.getY() + deltaDist * sin(t);
            double yaw = odomPose_.getYaw() + deltaYaw;
            odomPose_.setPose(x, y, yaw);
        }

        // if (cmd_vel_){
        //     double linear_velocity = cmd_vel_->linear.x;  // Assuming motion in x direction
        //     double angular_velocity = cmd_vel_->angular.z;  // Assuming yaw rotation around z-axis

        //     // Compute deltaDist and deltaYaw
        //     double delta_t = 0.1;  // Example time step, adjust based on your update rate
        //     deltaX_ = linear_velocity * delta_t;
        //     deltaYaw_ = angular_velocity * delta_t;

        // }

        deltaDistSum_ += fabs(deltaDist);
        deltaYawSum_ += fabs(deltaYaw);


          // Implement sample_motion_odometry (Prob Rob p 136)
        double alpha1_ = 0.2;
        double alpha2_ = 0.2;
        double alpha3_ = 0.2;
        double alpha4_ = 0.2; 

        double delta_rot1, delta_trans, delta_rot2;
        double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
        double delta_rot1_noise, delta_rot2_noise;

        double old_yaw = mclPose_.getYaw();

          // Avoid computing a bearing from two poses that are extremely near each
        // other (happens on in-place rotation).
        if (sqrt(
            dx * dx +
            dy * dy) < 0.01)
        {
            delta_rot1 = 0.0;
        } else {
            delta_rot1 = angle_diff(
            atan2(dx, dy),
            old_yaw);
        }
        delta_trans = sqrt(
            dx * dx +
            dy * dy);
        delta_rot2 = angle_diff(deltaYaw, delta_rot1);

        // We want to treat backward and forward motion symmetrically for the
        // noise model to be applied below.  The standard model seems to assume
        // forward motion.
        delta_rot1_noise = std::min(
            fabs(angle_diff(delta_rot1, 0.0)),
            fabs(angle_diff(delta_rot1, M_PI)));
        delta_rot2_noise = std::min(
            fabs(angle_diff(delta_rot2, 0.0)),
            fabs(angle_diff(delta_rot2, M_PI)));

        for (int i = 0; i < particlesNum_; ++i) {

            // Sample pose differences
            delta_rot1_hat = angle_diff(
            delta_rot1,
            pf_ran_gaussian(
                sqrt(
                alpha1_ * delta_rot1_noise * delta_rot1_noise +
                alpha2_ * delta_trans * delta_trans)));
            delta_trans_hat = delta_trans -
            pf_ran_gaussian(
            sqrt(
                alpha3_ * delta_trans * delta_trans +
                alpha4_ * delta_rot1_noise * delta_rot1_noise +
                alpha4_ * delta_rot2_noise * delta_rot2_noise));
            delta_rot2_hat = angle_diff(
            delta_rot2,
            pf_ran_gaussian(
                sqrt(
                alpha1_ * delta_rot2_noise * delta_rot2_noise +
                alpha2_ * delta_trans * delta_trans)));

            double x = particles_[i].getX() + delta_trans_hat *
                cos(particles_[i].getYaw() + delta_rot1_hat);
            double y = particles_[i].getY() + delta_trans_hat *
                sin(particles_[i].getYaw() + delta_rot1_hat);
            double yaw = particles_[i].getYaw() + delta_rot1_hat + delta_rot2_hat;


            particles_[i].setPose(x, y, yaw);
        }
        // differential drive model
        // double dist2 = deltaDist * deltaDist;
        // double yaw2 = deltaYaw * deltaYaw;
        // double distRandVal = dist2 * odomNoiseDDM_[0] + yaw2 * odomNoiseDDM_[1];
        // double yawRandVal = dist2 * odomNoiseDDM_[2] + yaw2 * odomNoiseDDM_[3];
        // for (int i = 0; i < particlesNum_; ++i) {
        //     double ddist = deltaDist + nrand(distRandVal);
        //     double dyaw = deltaYaw + nrand(yawRandVal);
        //     double yaw = particles_[i].getYaw();
        //     double t = yaw + dyaw / 2.0;
        //     double x = particles_[i].getX() + ddist * cos(t);
        //     double y = particles_[i].getY() + ddist * sin(t);
        //     yaw += dyaw;
        //     particles_[i].setPose(x, y, yaw);
        // }
    }
    


    void calculateLikelihoodsByMeasurementModel(void) {
        if (rejectUnknownScan_ && (measurementModelType_ == 0 || measurementModelType_ == 1))
            rejectUnknownScan();

        // this->get_clock()->now() = scan_.header.stamp;
        double xo = baseLink2Laser_.getX();
        double yo = baseLink2Laser_.getY();
        double yawo = baseLink2Laser_.getYaw();
        std::vector<Pose> sensorPoses(particlesNum_);
        for (int i = 0; i < particlesNum_; ++i) {
            double yaw = particles_[i].getYaw();
            double sensorX = xo * cos(yaw) - yo * sin(yaw) + particles_[i].getX();
            double sensorY = xo * sin(yaw) + yo * cos(yaw) + particles_[i].getY();
            double sensorYaw = yawo + yaw;
            Pose sensorPose(sensorX, sensorY, sensorYaw);
            sensorPoses[i] = sensorPose;
            particles_[i].setW(0.0);
        }

        for (int i = 0; i < (int)scan_->ranges.size(); i += scanStep_) {
            double range = scan_->ranges[i];
            double rangeAngle = (double)i * scan_->angle_increment + scan_->angle_min;
            double max;
            for (int j = 0; j < particlesNum_; ++j) {
                double p;
                if (measurementModelType_ == 0)
                    p = calculateLikelihoodFieldModel(sensorPoses[j], range, rangeAngle);
                else if (measurementModelType_ == 1)
                    p = calculateBeamModel(sensorPoses[j], range, rangeAngle);
                else
                    p = calculateClassConditionalMeasurementModel(sensorPoses[j], range, rangeAngle);
                double w = particles_[j].getW();
                w += log(p);
                particles_[j].setW(w);
                if (j == 0) {
                    max = w;
                } else {
                    if (max < w)
                        max = w;
                }
            }

            // Too small values cannot be calculated.
            // The log sum values are shifted if the maximum value is less than threshold.
            if (max < -300.0) {
                for (int j = 0; j < particlesNum_; ++j) {
                    double w = particles_[j].getW() + 300.0;
                    particles_[j].setW(w);
                }
            }
        }

        double sum = 0.0;
        double max;
        int maxIdx;
        for (int i = 0; i < particlesNum_; ++i) {
            // The log sum is converted to the probability.
            double w = exp(particles_[i].getW());
            particles_[i].setW(w);
            sum += w;
            if (i == 0) {
                max = w;
                maxIdx = i;
            } else if (max < w) {
                max = w;
                maxIdx = i;
            }
        }
        totalLikelihood_ = sum;
        averageLikelihood_ = sum / (double)particlesNum_;
        maxLikelihood_ = max;
        maxLikelihoodParticleIdx_ = maxIdx;

        // augmented MCL
//        std::cout << "totalLikelihood_ = " << totalLikelihood_ << std::endl;
//        std::cout << "maxLikelihood_ = " << maxLikelihood_ << std::endl;
//        std::cout << "averageLikelihood_ = " << averageLikelihood_ << std::endl;
        omegaSlow_ += alphaSlow_ * (averageLikelihood_ - omegaSlow_);
        omegaFast_ += alphaFast_ * (averageLikelihood_ - omegaFast_);
        amclRandomParticlesRate_ = 1.0 - omegaFast_ / omegaSlow_;
        if (amclRandomParticlesRate_ < 0.0 || std::isnan(amclRandomParticlesRate_))
            amclRandomParticlesRate_ = 0.0;

        // If publishUnknownScan_ is true and the class conditional measurement model is used,
        // unknown scan measurements are estimated based on the maximum likelihood particle.
        if (publishUnknownScan_ && measurementModelType_ == 2) {
            Pose mlPose = particles_[maxLikelihoodParticleIdx_].getPose();
            estimateUnknownScanWithClassConditionalMeasurementModel(mlPose);
        }
    }

    void calculateEffectiveSampleSize(void) {
        // calculate the effective sample size
        double sum = 0.0;
        double wo = 1.0 / (double)particlesNum_;
        for (int i = 0; i < particlesNum_; ++i) {
            double w = particles_[i].getW() / totalLikelihood_;
            if (std::isnan(w))
                w = wo;
            particles_[i].setW(w);
            sum += w * w;
        }
        effectiveSampleSize_ = 1.0 / sum;
    }

    void estimatePose(void) {
        double tmpYaw = mclPose_.getYaw();
        double x = 0.0, y = 0.0, yaw = 0.0;
        for (size_t i = 0; i < particlesNum_; ++i) {
            double w = particles_[i].getW();
            x += particles_[i].getX() * w;
            y += particles_[i].getY() * w;
            double dyaw = tmpYaw - particles_[i].getYaw();
            while (dyaw < -M_PI)
                dyaw += 2.0 * M_PI;
            while (dyaw > M_PI)
                dyaw -= 2.0 * M_PI;
            yaw += dyaw * w;
        }
        yaw = tmpYaw - yaw;
        mclPose_.setPose(x, y, yaw);
    }

    void resampleParticles(void) {
        double threshold = (double)particlesNum_ * resampleThresholdESS_;
        if (effectiveSampleSize_ > threshold)
            return;

        if (deltaXSum_ < resampleThresholds_[0] && deltaYSum_ < resampleThresholds_[1] &&
            deltaDistSum_ < resampleThresholds_[2] && deltaYawSum_ < resampleThresholds_[3] &&
            deltaTimeSum_ < resampleThresholds_[4])
            return;

        deltaXSum_ = deltaYSum_ = deltaDistSum_ = deltaYSum_ = deltaTimeSum_ = 0.0;
        std::vector<double> wBuffer(particlesNum_);
        wBuffer[0] = particles_[0].getW();
        for (int i = 1; i < particlesNum_; ++i)
            wBuffer[i] = particles_[i].getW() + wBuffer[i - 1];

        std::vector<Particle> tmpParticles = particles_;
        double wo = 1.0 / (double)particlesNum_;

        if (!addRandomParticlesInResampling_ && !useAugmentedMCL_) {
            // normal resampling
            for (int i = 0; i < particlesNum_; ++i) {
                double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                for (int j = 0; j < particlesNum_; ++j) {
                    if (darts < wBuffer[j]) {
                        particles_[i].setPose(tmpParticles[j].getPose());
                        particles_[i].setW(wo);
                        resampleIndices_[i] = j;
                        break;
                    }
                }
            }
        } else {
            // resampling and add random particles
            double randomParticlesRate = randomParticlesRate_;
            if (useAugmentedMCL_ && amclRandomParticlesRate_ > 0.0) {
                omegaSlow_ = omegaFast_ = 0.0;
                randomParticlesRate = amclRandomParticlesRate_;
            } else if (!addRandomParticlesInResampling_) {
                randomParticlesRate = 0.0;
            }
            int resampledParticlesNum = (int)((1.0 - randomParticlesRate) * (double)particlesNum_);
            int randomParticlesNum = particlesNum_ - resampledParticlesNum;
            for (int i = 0; i < resampledParticlesNum; ++i) {
                double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                for (int j = 0; j < particlesNum_; ++j) {
                    if (darts < wBuffer[j]) {
                        particles_[i].setPose(tmpParticles[j].getPose());
                        particles_[i].setW(wo);
                        resampleIndices_[i] = j;
                        break;
                    }
                }
            }
            double xo = mclPose_.getX();
            double yo = mclPose_.getY();
            double yawo = mclPose_.getYaw();
            for (int i = resampledParticlesNum; i < resampledParticlesNum + randomParticlesNum; ++i) {
                double x = xo + nrand(randomParticlesNoise_[0]);
                double y = yo + nrand(randomParticlesNoise_[1]);
                double yaw = yawo + nrand(randomParticlesNoise_[2]);
                particles_[i].setPose(x, y, yaw);
                particles_[i].setW(wo);
                resampleIndices_[i] = -1;
            }
        }
    }

    std::vector<float> getResidualErrors(void) {
        double yaw = mclPose_.getYaw();
        double sensorX = baseLink2Laser_.getX() * cos(yaw) - baseLink2Laser_.getY() * sin(yaw) + mclPose_.getX();
        double sensorY = baseLink2Laser_.getX() * sin(yaw) + baseLink2Laser_.getY() * cos(yaw) + mclPose_.getY();
        double sensorYaw = baseLink2Laser_.getYaw() + yaw;
        std::vector<float> residualErrors((int)scan_->ranges.size());
        for (int i = 0; i < (int)scan_->ranges.size(); ++i) {
            double r = scan_->ranges[i];
            if (r <= scan_->range_min || scan_->range_max <= r) {
                residualErrors[i] = -1.0;
                continue;
            }
            double t = (double)i * scan_->angle_increment + scan_->angle_min + sensorYaw;
            double x = r * cos(t) + sensorX;
            double y = r * sin(t) + sensorY;
            int u, v;
            xy2uv(x, y, &u, &v);
            if (onMap(u, v)) {
                float dist = distMap_.at<float>(v, u);
                residualErrors[i] = dist;
            } else {
                residualErrors[i] = -1.0;
            }
        }
        return residualErrors;
    }

    float getMeanAbsoluteError(std::vector<float> residualErrors) {
        float sum = 0.0f;
        int validRENum = 0;
        for (int i = 0; i < (int)residualErrors.size(); ++i) {
            float e = residualErrors[i];
            if (0.0 <= e && e < maxResidualError_) {
                sum += e;
                validRENum++;
            }
        }
        if (validRENum < minValidResidualErrorNum_)
            return -1.0f;
        else
            return (sum / (float)validRENum);
    }

    void estimateLocalizationCorrectness(void) {
        if (!useEstimationResetting_)
            return;

        static int failureCnt = 0;
        std::vector<float> residualErrors = getResidualErrors();
        float mae = getMeanAbsoluteError(residualErrors);
        if (mae < 0.0f || meanAbsoluteErrorThreshold_ < mae)
            failureCnt++;
        else
            failureCnt = 0;
        meanAbsoluteError_ = mae;
        failureCnt_ = failureCnt;
        if (failureCnt >= failureCntThreshold_) {
            if (performGlobalLocalization_)
                resetParticlesDistributionGlobally();
            else
                resetParticlesDistribution();
            failureCnt = 0;
        }
    }

    void printResult(void) {
        std::cout << "MCL: x = " << mclPose_.getX() << " [m], y = " << mclPose_.getY() << " [m], yaw = " << mclPose_.getYaw() * rad2deg_ << " [deg]" << std::endl;
        std::cout << "Odom: x = " << odomPose_.getX() << " [m], y = " << odomPose_.getY() << " [m], yaw = " << odomPose_.getYaw() * rad2deg_ << " [deg]" << std::endl;
        std::cout << "total likelihood = " << totalLikelihood_ << std::endl;
        std::cout << "average likelihood = " << averageLikelihood_ << std::endl;
        std::cout << "max likelihood = " << maxLikelihood_ << std::endl;
        std::cout << "effective sample size = " << effectiveSampleSize_ << std::endl;
        if (useAugmentedMCL_)
            std::cout << "amcl random particles rate = " << amclRandomParticlesRate_ << std::endl;
        if (useEstimationResetting_)
            std::cout << "mean absolute error = " << meanAbsoluteError_ << " [m] (failureCnt = " << failureCnt_ << ")" << std::endl;
        std::cout << std::endl;
    }

    void publishROSMessages(void) {
        // pose
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = mapFrame_;
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = mclPose_.getX();
        pose.pose.position.y = mclPose_.getY();
        double yaw = mclPose_.getYaw(); // Assume getYaw() returns yaw in radians
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw); // Roll and pitch are zero; yaw is provided

        // Convert tf2 quaternion to geometry_msgs quaternion
        geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(q);
        pose.pose.orientation = quaternion_msg;
        posePub_->publish(pose);

        // particles
        geometry_msgs::msg::PoseArray particlesPoses;
        particlesPoses.header.frame_id = mapFrame_;
        particlesPoses.header.stamp = this->get_clock()->now();
        particlesPoses.poses.resize(particlesNum_);
        for (int i = 0; i < particlesNum_; ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = particles_[i].getX();
            pose.position.y = particles_[i].getY();
            // Assuming particles_ is a collection of objects that have a getYaw() method returning yaw in radians
            double yaw = particles_[i].getYaw();
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw); // Roll and pitch are 0, and yaw is the angle

            // Convert the tf2 Quaternion into a geometry_msgs::msg::Quaternion
            geometry_msgs::msg::Quaternion quaternion_msg = tf2::toMsg(q);

            // Now assign it to the pose.orientation
            pose.orientation = quaternion_msg;

            particlesPoses.poses[i] = pose;
        }
        particlesPub_->publish(particlesPoses);

        // unknown scan
        if (publishUnknownScan_ && (rejectUnknownScan_ || measurementModelType_ == 2))
            unknownScanPub_->publish(*unknownScan_);

        // residual errors
        // if (publishResidualErrors_) {
        //     sensor_msgs::LaserScan residualErrors = scan_;
        //     residualErrors.ranges = getResidualErrors();    
        //     residualErrorsPub_->publish(residualErrors);
        // }
    }

    void broadcastTF() {
        // static tf2_ros::TransformBroadcaster br(this->get_node_base_interface());

        if (!broadcastTF_)
            return;

        if (useOdomFrame_) {
            if (!useOdomMsg_) {
                geometry_msgs::msg::TransformStamped odom2baseLinkTrans;
                odom2baseLinkTrans.header.stamp = this->get_clock()->now();
                odom2baseLinkTrans.header.frame_id = odomFrame_;
                odom2baseLinkTrans.child_frame_id = baseLinkFrame_;
                odom2baseLinkTrans.transform.translation.x = odomPose_.getX();
                odom2baseLinkTrans.transform.translation.y = odomPose_.getY();
                odom2baseLinkTrans.transform.translation.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, odomPose_.getYaw());
                odom2baseLinkTrans.transform.rotation.x = q.x();
                odom2baseLinkTrans.transform.rotation.y = q.y();
                odom2baseLinkTrans.transform.rotation.z = q.z();
                odom2baseLinkTrans.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(odom2baseLinkTrans);
            }

            geometry_msgs::msg::Pose poseOnMap;
            poseOnMap.position.x = mclPose_.getX();
            poseOnMap.position.y = mclPose_.getY();
            poseOnMap.position.z = 0.0;
            tf2::Quaternion q_map;
            q_map.setRPY(0, 0, mclPose_.getYaw());
            poseOnMap.orientation.x = q_map.x();
            poseOnMap.orientation.y = q_map.y();
            poseOnMap.orientation.z = q_map.z();
            poseOnMap.orientation.w = q_map.w();

            tf2::Transform map2baseTrans;
            tf2::fromMsg(poseOnMap, map2baseTrans);

            geometry_msgs::msg::Pose poseOnOdom;
            poseOnOdom.position.x = odomPose_.getX();
            poseOnOdom.position.y = odomPose_.getY();
            poseOnOdom.position.z = 0.0;
            tf2::Quaternion q_odom;
            q_odom.setRPY(0, 0, odomPose_.getYaw());
            poseOnOdom.orientation.x = q_odom.x();
            poseOnOdom.orientation.y = q_odom.y();
            poseOnOdom.orientation.z = q_odom.z();
            poseOnOdom.orientation.w = q_odom.w();

            tf2::Transform odom2baseTrans;
            tf2::fromMsg(poseOnOdom, odom2baseTrans);

            tf2::Transform map2odomTrans = map2baseTrans * odom2baseTrans.inverse();
            geometry_msgs::msg::TransformStamped map2odomStampedTrans;
            map2odomStampedTrans.header.stamp = this->get_clock()->now();
            map2odomStampedTrans.header.frame_id = mapFrame_;
            map2odomStampedTrans.child_frame_id = odomFrame_;
            tf2::convert(map2odomTrans, map2odomStampedTrans.transform);
            tf_broadcaster_->sendTransform(map2odomStampedTrans);
        } else {
            geometry_msgs::msg::TransformStamped tfStamped;
            tfStamped.header.stamp = this->get_clock()->now();
            tfStamped.header.frame_id = mapFrame_;
            tfStamped.child_frame_id = baseLinkFrame_;
            tfStamped.transform.translation.x = mclPose_.getX();
            tfStamped.transform.translation.y = mclPose_.getY();
            tfStamped.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, mclPose_.getYaw());
            tfStamped.transform.rotation.x = q.x();
            tfStamped.transform.rotation.y = q.y();
            tfStamped.transform.rotation.z = q.z();
            tfStamped.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(tfStamped);
        }
    }

    void timer_callback() {
        // This function will be called every 1 second
        
        if (initial_mcl_ready_ == false){
            if (stage_run_init == 0){
                RCLCPP_INFO(this->get_logger(), "Initialing map .........");
                if (gotMap_){
                    stage_run_init = 1;
                }
                mapFailedCnt++;
                if (mapFailedCnt >= 500) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot get a map message.");
                    RCLCPP_ERROR(this->get_logger(), "Did you publish the map? Expected map topic name is: %s", mapName_.c_str());
                    exit(1);
                }
            }
            else if(stage_run_init == 1){
                if (gotScan_)
                    stage_run_init = 2;
                scanFailedCnt++;
                if (scanFailedCnt >= 100) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot get a scan message.");
                    exit(1);
                }
            }
            else if (stage_run_init == 2){
                if (performGlobalLocalization_)
                    resetParticlesDistributionGlobally();
                else
                    resetParticlesDistribution();
                resampleIndices_.resize(particlesNum_);
                        // measurement model
                normConstHit_ = 1.0 / sqrt(2.0 * varHit_ * M_PI);
                denomHit_ = 1.0 / (2.0 * varHit_);
                pRand_ = 1.0 / (scan_->range_max / mapResolution_);
                measurementModelRandom_ = zRand_ * pRand_;
                measurementModelInvalidScan_ = zMax_ + zRand_ * pRand_;

                isInitialized_ = true;
                if (!useOdomMsg_)
                    isInitialized_ = false;
                RCLCPP_INFO(this->get_logger(), "MCL ready to localize");
                initial_mcl_ready_ = true;
            }
            
        }
        else {
            updateParticlesByMotionModel();
            setCanUpdateScan(false);
            calculateLikelihoodsByMeasurementModel();
            calculateEffectiveSampleSize();
            estimatePose();
            resampleParticles();
            estimateLocalizationCorrectness();
            publishROSMessages();
            broadcastTF();
            setCanUpdateScan(true);
            printResult();
        }



        // RCLCPP_INFO(this->get_logger(), "Timer callback triggered!");
    }

private :

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPoseSub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlesPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr unknownScanPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr residualErrorsPub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // lisener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;

    // subscribers
    std::string scanName_, odomName_, mapName_;
    // publishers
    std::string poseName_, particlesName_, unknownScanName_, residualErrorsName_;
    // frames
    std::string laserFrame_, baseLinkFrame_, mapFrame_, odomFrame_;
    bool useOdomFrame_;
    bool use_cmd_vel_;

    std::vector<double> initialPose_;
    Pose mclPose_, prevMCLPose_, baseLink2Laser_, odomPose_;

    // particles
    int particlesNum_;
    std::vector<Particle> particles_;
    std::vector<int> resampleIndices_;
    std::vector<double> initialNoise_;
    bool useAugmentedMCL_, addRandomParticlesInResampling_;
    double randomParticlesRate_;
    std::vector<double> randomParticlesNoise_;

    // map
    nav_msgs::msg::OccupancyGrid::SharedPtr ogm_;
    cv::Mat distMap_;
    double mapResolution_;
    Pose mapOrigin_;
    int mapWidth_, mapHeight_;
    double freeSpaceMinX_, freeSpaceMaxX_, freeSpaceMinY_, freeSpaceMaxY_;
    bool gotMap_;
    // motion
    double deltaX_, deltaY_, deltaDist_, deltaYaw_;
    double deltaXSum_, deltaYSum_, deltaDistSum_, deltaYawSum_, deltaTimeSum_;
    std::vector<double> resampleThresholds_;
    std::vector<double> odomNoiseDDM_, odomNoiseODM_;

    bool useOdomMsg_;
    geometry_msgs::msg::Twist::SharedPtr cmd_vel_;

    // measurements
    sensor_msgs::msg::LaserScan::SharedPtr scan_, unknownScan_;
    bool canUpdateScan_;
    bool initial_mcl_ready_;
    int stage_run_init = 0;

    // measurement model
    // 0: likelihood field model, 1: beam model, 2: class conditional measurement model
    int measurementModelType_;
    double zHit_, zShort_, zMax_, zRand_;
    double varHit_, lambdaShort_, lambdaUnknown_;
    double normConstHit_, denomHit_, pRand_;
    double measurementModelRandom_, measurementModelInvalidScan_;
    double pKnownPrior_, pUnknownPrior_, unknownScanProbThreshold_;
    double alphaSlow_, alphaFast_, omegaSlow_, omegaFast_;
    int scanStep_;
    bool rejectUnknownScan_, publishUnknownScan_, publishResidualErrors_;
    bool gotScan_;
    double resampleThresholdESS_;

    // localization type
    bool performGlobalLocalization_;

    // broadcast tf
    bool broadcastTF_;

    // localization result
    double totalLikelihood_, averageLikelihood_, maxLikelihood_;
    double amclRandomParticlesRate_, effectiveSampleSize_;
    int maxLikelihoodParticleIdx_;

    // localization correctness estimation
    float maxResidualError_;
    int minValidResidualErrorNum_;
    float meanAbsoluteErrorThreshold_;
    int failureCntThreshold_;
    bool useEstimationResetting_;
    double meanAbsoluteError_;
    int failureCnt_;

    // other paramerter
    // tf2_ros::TransformBroadcaster tfBroadcaster_;
    // tf2_ros::Buffer tfBuffer_;
    // tf2_ros::TransformListener tfListener_(tfBuffer_);
    bool isInitialized_;
    double localizationHz_;

    int mapFailedCnt = 0;
    int scanFailedCnt = 0;

    // constant parameters
    const double rad2deg_ = 180.0 / M_PI;

    //  ######################################### private part 2
    inline double nrand(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }
    inline double urand(double min, double max) { return ((max - min)  * (double)rand() / RAND_MAX + min); }

    inline bool onMap(int u, int v) {
        if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_)
            return true;
        else
            return false;
    }

    inline bool onFreeSpace(int u, int v) {
        if (!onMap(u, v))
            return false;
        int node = v * mapWidth_ + u;
        if (ogm_->data[node] == 0)
            return true;
        else
            return false;
    }

    inline void xy2uv(double x, double y, int *u, int *v) {
        double dx = x - mapOrigin_.getX();
        double dy = y - mapOrigin_.getY();
        double yaw = -mapOrigin_.getYaw();
        double xx = dx * cos(yaw) - dy * sin(yaw);
        double yy = dx * sin(yaw) + dy * cos(yaw);
        *u = (int)(xx / mapResolution_);
        *v = (int)(yy / mapResolution_);
    }

    inline void uv2xy(int u, int v, double *x, double *y) {
        double xx = (double)u * mapResolution_;
        double yy = (double)v * mapResolution_;
        double yaw = mapOrigin_.getYaw();
        double dx = xx * cos(yaw) - yy * sin(yaw);
        double dy = xx * sin(yaw) + yy * cos(yaw);
        *x = dx + mapOrigin_.getX();
        *y = dy + mapOrigin_.getY();
    }

    void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (canUpdateScan_)
            scan_ = msg;
        if (!gotScan_)
            gotScan_ = true;
    }

    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!useOdomMsg_)
            return;

        static double prevTime;
        double currTime = msg->header.stamp.sec;
        if (isInitialized_) {
            prevTime = currTime;
            isInitialized_ = false;
            return;
        }

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        odomPose_.setPose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
        // test here
        double deltaTime = currTime - prevTime;
        deltaX_ = msg->twist.twist.linear.x;
        deltaY_ = msg->twist.twist.linear.y;
        deltaDist_ = msg->twist.twist.linear.x;
        deltaYaw_ = msg->twist.twist.angular.z;

        deltaTimeSum_ += deltaTime;

        prevTime = currTime;
    }

    void mapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "map!! sub");
        // store the map information
        ogm_ = msg;
        mapWidth_ = msg->info.width;
        mapHeight_ = msg->info.height;
        mapResolution_ = msg->info.resolution;
        tf2::Quaternion q;
        tf2::fromMsg(msg->info.origin.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        mapOrigin_.setX(msg->info.origin.position.x);
        mapOrigin_.setY(msg->info.origin.position.y);
        mapOrigin_.setYaw(yaw);

        // perform distance transform to build the distance field
        // the min and max points of the free space are also obtained
        cv::Mat binMap(mapHeight_, mapWidth_, CV_8UC1);
        double minX, maxX, minY, maxY;
        bool isFirst = true;
        for (int v = 0; v < mapHeight_; v++) {
            for (int u = 0; u < mapWidth_; u++) {
                int node = v * mapWidth_ + u;
                int val = msg->data[node];
                if (val == 100) {
                    // occupied grid
                    binMap.at<uchar>(v, u) = 0;
                } else {
                    binMap.at<uchar>(v, u) = 1;
                    if (val == 0) {
                        double x, y;
                        uv2xy(u, v, &x, &y);
                        if (isFirst) {
                            minX = maxX = x;
                            minY = maxY = y;
                            isFirst = false;
                        } else {
                            if (x < minX)
                                minX = x;
                            if (x > maxX)
                                maxX = x;
                            if (y < minY)
                                minY = y;
                            if (y > maxY)
                                maxY = y;
                        }
                    }
                }
            }
        }
        freeSpaceMinX_ = minX;
        freeSpaceMaxX_ = maxX;
        freeSpaceMinY_ = minY;
        freeSpaceMaxY_ = maxY;
        cv::Mat distMap(mapHeight_, mapWidth_, CV_32FC1);
        cv::distanceTransform(binMap, distMap, cv::DIST_L2, 5);
        for (int v = 0; v < mapHeight_; v++) {
            for (int u = 0; u < mapWidth_; u++) {
                float d = distMap.at<float>(v, u) * (float)mapResolution_;
                distMap.at<float>(v, u) = d;
            }
        }
        distMap_ = distMap;
        gotMap_ = true;
    }

    void cmd_vel_CB(const geometry_msgs::msg::Twist::SharedPtr msg){
        cmd_vel_ = msg;
    }

    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // Assuming mclPose_ is an instance of a class or struct that has a method setPose
        mclPose_.setPose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

        resetParticlesDistribution();
        omegaSlow_ = omegaFast_ = 0.0;
        isInitialized_ = true;
    }

    void resetParticlesDistribution(void) {
        particles_.resize(particlesNum_);
        double xo = mclPose_.getX();
        double yo = mclPose_.getY();
        double yawo = mclPose_.getYaw();
        double wo = 1.0 / (double)particlesNum_;
        for (int i = 0; i < particlesNum_; ++i) {
            double x = xo + nrand(initialNoise_[0]);
            double y = yo + nrand(initialNoise_[1]);
            double yaw = yawo + nrand(initialNoise_[2]);
            particles_[i].setPose(x, y, yaw);
            particles_[i].setW(wo);
        }
    }

    void resetParticlesDistributionGlobally(void) {
        particles_.resize(particlesNum_);
        double wo = 1.0 / (double)particlesNum_;
        for (int i = 0; i < particlesNum_; ++i) {
            for (;;) {
                double x = urand(freeSpaceMinX_, freeSpaceMaxX_);
                double y = urand(freeSpaceMinY_, freeSpaceMaxY_);
                int u, v;
                xy2uv(x, y, &u, &v);
                if (onFreeSpace(u, v)) {
                    double yaw = urand(-M_PI, M_PI);
                    particles_[i].setPose(x, y, yaw);
                    particles_[i].setW(wo);
                    break;
                }
            }
        }
    }

    void rejectUnknownScan(void) {
        unknownScan_ = scan_;
        double xo = baseLink2Laser_.getX();
        double yo = baseLink2Laser_.getY();
        double yawo = baseLink2Laser_.getYaw();
        double hitThreshold = 0.5 * mapResolution_;
        for (int i = 0; i < (int)unknownScan_->ranges.size(); ++i) {
            if (i % scanStep_ != 0) {
                unknownScan_->ranges[i] = 0.0;
                continue;
            }
            double r = unknownScan_->ranges[i];
            if (r <= unknownScan_->range_min || unknownScan_->range_max <= r) {
                unknownScan_->ranges[i] = 0.0;
                continue;
            }
            double laserYaw = (double)i * unknownScan_->angle_increment + unknownScan_->angle_min;
            double pShortSum = 0.0, pBeamSum = 0.0;
            for (int j = 0; j < particlesNum_; ++j) {
                double yaw = particles_[j].getYaw();
                double x = xo * cos(yaw) - yo * sin(yaw) + particles_[j].getX();
                double y = xo * sin(yaw) + yo * cos(yaw) + particles_[j].getY();
                double t = yawo + yaw + laserYaw;
                double dx = mapResolution_ * cos(t);
                double dy = mapResolution_ * sin(t);
                int u, v;
                double expectedRange = -1.0;
                for (double range = 0.0; range <= unknownScan_->range_max; range += mapResolution_) {
                    xy2uv(x, y, &u, &v);
                    if (onMap(u, v)) {
                        double dist = (double)distMap_.at<float>(v, u);
                        if (dist < hitThreshold) {
                            expectedRange = range;
                            break;
                        }
                    } else {
                        break;
                    }
                    x += dx;
                    y += dy;
                }
                if (r <= expectedRange) {
                    double error = expectedRange - r;
                    double pHit = normConstHit_ * exp(-(error * error) * denomHit_) * mapResolution_;
                    double pShort = lambdaShort_ * exp(-lambdaShort_ * r) / (1.0 - exp(-lambdaShort_ * unknownScan_->range_max)) * mapResolution_;
                    pShortSum += pShort;
                    pBeamSum += zHit_ * pHit + zShort_ * pShort + measurementModelRandom_;
                } else {
                    pBeamSum += measurementModelRandom_;
                }
            }
            double pShort = pShortSum;
            double pBeam = pBeamSum;
            double pUnknown = pShortSum / pBeamSum;
            if (pUnknown < unknownScanProbThreshold_) {
                unknownScan_->ranges[i] = 0.0;
            } else {
                // unknown scan is rejected from the scan message used for localization
                scan_->ranges[i] = 0.0;
            }
        }
    }

    double calculateLikelihoodFieldModel(Pose pose, double range, double rangeAngle) {
        if (range <= scan_->range_min || scan_->range_max <= range)
            return measurementModelInvalidScan_;

        double t = pose.getYaw() + rangeAngle;
        double x = range * cos(t) + pose.getX();
        double y = range * sin(t) + pose.getY();
        int u, v;
        xy2uv(x, y, &u, &v);
        double p;
        if (onMap(u, v)) {
            double dist = (double)distMap_.at<float>(v, u);
            double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
            p = zHit_ * pHit + measurementModelRandom_;
        } else {
            p = measurementModelRandom_;
        }
        if (p > 1.0)
            p = 1.0;
        return p;
    }

    double calculateBeamModel(Pose pose, double range, double rangeAngle) {
        if (range <= scan_->range_min || scan_->range_max <= range)
            return measurementModelInvalidScan_;

        double t = pose.getYaw() + rangeAngle;
        double x = pose.getX();
        double y = pose.getY();
        double dx = mapResolution_ * cos(t);
        double dy = mapResolution_ * sin(t);
        int u, v;
        double expectedRange = -1.0;
        double hitThreshold = 0.5 * mapResolution_;
        for (double r = 0.0; r < scan_->range_max; r += mapResolution_) {
            xy2uv(x, y, &u, &v);
            if (onMap(u, v)) {
                double dist = (double)distMap_.at<float>(v, u);
                if (dist < hitThreshold) {
                    expectedRange = r;
                    break;
                }
            } else {
                break;
            }
            x += dx;
            y += dy;
        }

        double p;
        if (range <= expectedRange) {
            double error = expectedRange - range;
            double pHit = normConstHit_ * exp(-(error * error) * denomHit_) * mapResolution_;
            double pShort = lambdaShort_ * exp(-lambdaShort_ * range) / (1.0 - exp(-lambdaShort_ * scan_->range_max)) * mapResolution_;
            p = zHit_ * pHit + zShort_ * pShort + measurementModelRandom_;
        } else {
            p = measurementModelRandom_;
        }
        if (p > 1.0)
            p = 1.0;
        return p;
    }

    double calculateClassConditionalMeasurementModel(Pose pose, double range, double rangeAngle) {
        if (range <= scan_->range_min || scan_->range_max <= range)
            return measurementModelInvalidScan_;

        double t = pose.getYaw() + rangeAngle;
        double x = range * cos(t) + pose.getX();
        double y = range * sin(t) + pose.getY();
        double pUnknown = lambdaUnknown_ * exp(-lambdaUnknown_ * range) / (1.0 - exp(-lambdaUnknown_ * scan_->range_max)) * mapResolution_ * pUnknownPrior_;
        int u, v;
        xy2uv(x, y, &u, &v);
        double p = pUnknown;
        if (onMap(u, v)) {
            double dist = (double)distMap_.at<float>(v, u);
            double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
            p += (zHit_ * pHit + measurementModelRandom_) * pKnownPrior_;
        } else {
            p += measurementModelRandom_ * pKnownPrior_;
        }
        if (p > 1.0)
            p = 1.0;
        return p;
    }

    void estimateUnknownScanWithClassConditionalMeasurementModel(Pose pose) {
        unknownScan_ = scan_;
        double yaw = pose.getYaw();
        double sensorX = baseLink2Laser_.getX() * cos(yaw) - baseLink2Laser_.getY() * sin(yaw) + pose.getX();
        double sensorY = baseLink2Laser_.getX() * sin(yaw) + baseLink2Laser_.getY() * cos(yaw) + pose.getY();
        double sensorYaw = baseLink2Laser_.getYaw() + yaw;
        for (int i = 0; i < (int)unknownScan_->ranges.size(); ++i) {
            double r = unknownScan_->ranges[i];
            if (r <= unknownScan_->range_min || unknownScan_->range_max <= r) {
                unknownScan_->ranges[i] = 0.0;
                continue;
            }
            double t = sensorYaw + (double)i * unknownScan_->angle_increment + unknownScan_->angle_min;
            double x = r * cos(t) + sensorX;
            double y = r * sin(t) + sensorY;
            int u, v;
            xy2uv(x, y, &u, &v);
            double pKnown;
            double pUnknown = lambdaUnknown_ * exp(-lambdaUnknown_ * r) / (1.0 - exp(-lambdaUnknown_ * unknownScan_->range_max)) * mapResolution_ * pUnknownPrior_;
            if (onMap(u, v)) {
                double dist = (double)distMap_.at<float>(v, u);
                double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
                pKnown = (zHit_ * pHit + measurementModelRandom_) * pKnownPrior_;
            } else {
                pKnown = measurementModelRandom_ * pKnownPrior_;
            }
            double sum = pKnown + pUnknown;
            pUnknown /= sum;
            if (pUnknown < unknownScanProbThreshold_)
                unknownScan_->ranges[i] = 0.0;
        }
    }




};

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_run = std::make_shared<robot_mcl_cpp::MCL_Node>();
  rclcpp::spin(node_run);
  rclcpp::shutdown();
  return 0;
}