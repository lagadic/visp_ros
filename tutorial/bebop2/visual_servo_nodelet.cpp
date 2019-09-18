/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2019 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr/ for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * https://team.inria.fr/rainbow/fr/
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Visual servoing nodelet to control Parrot Bebop 2 drone.
 *
 * Authors:
 * Gatien Gaumerais
 * Fabien Spindler
 *
 *****************************************************************************/

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTime.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include <iostream>
#include <string>

static double getTagSize(ros::NodeHandle nh)
{
    double res = 0;
	
    if (nh.hasParam("/vs/tagSize")) {
        nh.getParam("/vs/tagSize", res);
        return res;
    } else {
        ROS_WARN("No tag size parameter has been found, using default value : 0.14 meters.");
        return 0.14;
    }
}

static double getDistanceToTag(ros::NodeHandle nh)
{
    double res = 0;
    if (nh.hasParam("/vs/distanceToTag")) {
        nh.getParam("/vs/distanceToTag", res);
        return res;
    } else {
        ROS_WARN("No distance to tag parameter has been found, using default value : 1.5 meters.");
        return 1.5;
    }
}

bool compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2)
{
    return (p1.second.get_v() < p2.second.get_v());
}

class bebopVSNodelet : public nodelet::Nodelet
{
public:
    bebopVSNodelet();
    virtual ~bebopVSNodelet();

    virtual void onInit();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pubTwist;
    ros::Subscriber m_subImg;
    ros::Publisher m_pubCam;

    bool bebop_has_been_setup;

    vpDisplayX m_display;
    bool m_display_setup;
    vpImage<unsigned char> I;

    double m_tagSize;
    double m_Z_d;

    double m_cameraTilt;
    double m_cameraPan;

    vpServo m_task;
    vpDetectorAprilTag m_detector;
    vpCameraParameters m_cam;

    bool m_vec_ip_has_been_sorted;
    std::vector<std::pair<size_t, vpImagePoint>> m_vec_ip_sorted;

    vpVelocityTwistMatrix m_cVe;
    vpMatrix m_eJe;

    double area;

    // Desired plane
    double A;
    double B;
    double C;

    vpMomentObject m_obj, m_obj_d;
    vpMomentDatabase mdb, mdb_d;
    vpMomentBasic mb_d;
    vpMomentGravityCenter mg, mg_d;
    vpMomentCentered mc, mc_d;
    vpMomentAreaNormalized man, man_d;
    vpMomentGravityCenterNormalized mgn, mgn_d;

    vpFeatureMomentGravityCenterNormalized s_mgn, s_mgn_d;
    vpFeatureMomentAreaNormalized s_man, s_man_d;
    vpFeatureVanishingPoint s_vp, s_vp_d;

    void initVS();
    vpColVector velocityToPosition(vpColVector &vel_cmd, double delta_t);
};

PLUGINLIB_EXPORT_CLASS(bebopVSNodelet, nodelet::Nodelet)

bebopVSNodelet::bebopVSNodelet()
    : bebop_has_been_setup(false)
    , m_display_setup(false)
    , m_tagSize(getTagSize(m_nh))   // Fetching theses values
    , m_Z_d(getDistanceToTag(m_nh)) // from the parameter server
    , m_cameraTilt(0.0)
    , m_cameraPan(0.0)
    , m_detector(vpDetectorAprilTag::TAG_36h11)
    , m_vec_ip_has_been_sorted(false)
    , m_eJe(vpMatrix(6, 4, 0))
    , area(0)
    , A(0.0)
    , B(0.0)
    , C(1.0 / m_Z_d)
    , m_obj(3)
    , m_obj_d(3)
    , man(0, m_Z_d)
    , man_d(0, m_Z_d)
    , s_mgn(mdb, A, B, C)
    , s_mgn_d(mdb_d, A, B, C)
    , s_man(mdb, A, B, C)
    , s_man_d(mdb_d, A, B, C)
{
    
}

bebopVSNodelet::~bebopVSNodelet()
{
    m_task.kill();
}

void bebopVSNodelet::onInit(){

    m_nh = getNodeHandle();
    std::stringstream params_str;
    params_str << "TAG PARAMETERS (meters) : tagSize : " << m_tagSize
               << " - distanceToTag : " << m_Z_d << std::endl;
    NODELET_INFO("%s", params_str.str().c_str());

    initVS();

    NODELET_INFO("Setting up publisher and subscriber...");
    m_pubTwist = m_nh.advertise<geometry_msgs::Twist>("cmd_moveby", 1000);
    m_subImg = m_nh.subscribe("image_raw", 1000, &bebopVSNodelet::imageCallback, this);
    m_pubCam = m_nh.advertise<geometry_msgs::Twist>("camera_control", 1000);
    NODELET_INFO("Publisher and subscriber set up. Waiting for image callbacks...");
}

void bebopVSNodelet::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if (!bebop_has_been_setup) {
        geometry_msgs::Twist out_cam_ctrl;
        out_cam_ctrl.linear.x = 0;
        out_cam_ctrl.linear.y = 0;
        out_cam_ctrl.linear.z = 0;
        out_cam_ctrl.angular.x = 0;
        out_cam_ctrl.angular.y = m_cameraTilt;
        out_cam_ctrl.angular.z = m_cameraPan;
        m_pubCam.publish(out_cam_ctrl);

        bebop_has_been_setup = true;
        NODELET_INFO("Setting desired camera orientation...");
    }

    const cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    vpImageConvert::convert(frame, I);

    if (!m_display_setup) {
        m_display.init(I, 100, 100, "DRONE VIEW");
        m_display_setup = true;
    }
    vpDisplay::display(I);

    std::vector<vpHomogeneousMatrix> cMo_vec;
    m_detector.detect(I, m_tagSize, m_cam, cMo_vec); // Detect AprilTags in current image

    geometry_msgs::Twist out_cmd_pos;
    
    if (m_detector.getNbObjects() == 1) {
        // Update current points used to compute the moments
        std::vector<vpImagePoint> vec_ip = m_detector.getPolygon(0);
        std::vector<vpPoint> vec_P;
        for (size_t i = 0; i < vec_ip.size(); i++) { // size = 4
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(m_cam, vec_ip[i], x, y);
            vpPoint P;
            P.set_x(x);
            P.set_y(y);
            vec_P.push_back(P);
        }

        // Current moments
        m_obj.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
        m_obj.fromVector(vec_P); // Initialize the object with the points coordinates

        mg.linkTo(mdb);       // Add gravity center to database
        mc.linkTo(mdb);       // Add centered moments to database
        man.linkTo(mdb);      // Add area normalized to database
        mgn.linkTo(mdb);      // Add gravity center normalized to database
        mdb.updateAll(m_obj); // All of the moments must be updated, not just an_d
        mg.compute();         // Compute gravity center moment
        mc.compute();         // Compute centered moments AFTER gravity center
        man.setDesiredArea(
            area); // Desired area was init at 0 (unknow at contruction), need to be updated here
        man.compute(); // Compute area normalized moment AFTER centered moment
        mgn.compute(); // Compute gravity center normalized moment AFTER area normalized moment

        s_mgn.update(A, B, C);
        s_mgn.compute_interaction();
        s_man.update(A, B, C);
        s_man.compute_interaction();

        /* Sort points from their height in the image, and keep original indexes.
		This is done once, in order to be independent from the orientation of the tag
		when detecting vanishing points. */
        if (!m_vec_ip_has_been_sorted) {
            for (size_t i = 0; i < vec_ip.size(); i++) {
                // Add the points and their corresponding index
                std::pair<size_t, vpImagePoint> index_pair
                    = std::pair<size_t, vpImagePoint>(i, vec_ip[i]);
                m_vec_ip_sorted.push_back(index_pair);
            }

            // Sort the points and indexes from the v value of the points
            std::sort(m_vec_ip_sorted.begin(), m_vec_ip_sorted.end(), compareImagePoint);

            m_vec_ip_has_been_sorted = true;
        }

        // Use the two highest points for the first line, and the two others for the second line.
        vpFeatureBuilder::create(s_vp,
                                 m_cam,
                                 vec_ip[m_vec_ip_sorted[0].first],
                                 vec_ip[m_vec_ip_sorted[1].first],
                                 vec_ip[m_vec_ip_sorted[2].first],
                                 vec_ip[m_vec_ip_sorted[3].first],
                                 vpFeatureVanishingPoint::selectAtanOneOverRho());

        m_task.set_cVe(m_cVe);
        m_task.set_eJe(m_eJe);

        // Compute the control law. Velocities are computed in the mobile robot reference frame
        vpColVector ve = m_task.computeControlLaw();
        std::stringstream ve_str;
        ve_str << "ve: " << ve.t() << std::endl;
        NODELET_INFO("%s", ve_str.str().c_str());

        // Computing the corresponding displacement
        vpColVector cmd_pos = velocityToPosition(ve, 1.0);

        //Sending the movement command to the drone

        out_cmd_pos.linear.x = cmd_pos[0];
        out_cmd_pos.linear.y = cmd_pos[1];
        out_cmd_pos.linear.z = cmd_pos[2];
        out_cmd_pos.angular.x = 0;
        out_cmd_pos.angular.y = 0;
        out_cmd_pos.angular.z = cmd_pos[3];
        m_pubTwist.publish(out_cmd_pos);

        for (size_t i = 0; i < 4; i++) {
            vpDisplay::displayCross(I, vec_ip[i], 15, vpColor::red, 1);
            std::stringstream ss;
            ss << i;
            vpDisplay::displayText(I, vec_ip[i] + vpImagePoint(15, 15), ss.str(), vpColor::green);
        }

        // Display visual features
        vpDisplay::displayPolygon(I,
                                  vec_ip,
                                  vpColor::green,
                                  3); // Current polygon used to compure an moment
        vpDisplay::displayCross(I,
                                m_detector.getCog(0),
                                15,
                                vpColor::green,
                                3); // Current polygon used to compute a moment
        vpDisplay::displayLine(I,
                               0,
                               static_cast<int>(m_cam.get_u0()),
                               static_cast<int>(I.getHeight()) - 1,
                               static_cast<int>(m_cam.get_u0()),
                               vpColor::red,
                               3); // Vertical line as desired x position
        vpDisplay::displayLine(I,
                               static_cast<int>(m_cam.get_v0()),
                               0,
                               static_cast<int>(m_cam.get_v0()),
                               static_cast<int>(I.getWidth()) - 1,
                               vpColor::red,
                               3); // Horizontal line as desired y position

        // Display lines corresponding to the vanishing point for the horizontal lines
        vpDisplay::displayLine(I,
                               vec_ip[m_vec_ip_sorted[0].first],
                               vec_ip[m_vec_ip_sorted[1].first],
                               vpColor::red,
                               1,
                               false);
        vpDisplay::displayLine(I,
                               vec_ip[m_vec_ip_sorted[2].first],
                               vec_ip[m_vec_ip_sorted[3].first],
                               vpColor::red,
                               1,
                               false);

        vpDisplay::displayText(I, 10, 10, "Click to exit", vpColor::red);
        vpDisplay::flush(I);

    } else {
        
        std::stringstream sserr;
        sserr << "Failed to detect an Apriltag, or detected multiple ones";
        vpDisplay::displayText(I, 120, 20, sserr.str(), vpColor::red);
        vpDisplay::flush(I);

        //Stoping drone movement
        out_cmd_pos.linear.x = 0;
        out_cmd_pos.linear.y = 0;
        out_cmd_pos.linear.z = 0;
        out_cmd_pos.angular.x = 0;
        out_cmd_pos.angular.y = 0;
        out_cmd_pos.angular.z = 0;
        m_pubTwist.publish(out_cmd_pos);
        
    }

    if (vpDisplay::getClick(I, false)) {
        //Stoping drone movement
        out_cmd_pos.linear.x = 0;
        out_cmd_pos.linear.y = 0;
        out_cmd_pos.linear.z = 0;
        out_cmd_pos.angular.x = 0;
        out_cmd_pos.angular.y = 0;
        out_cmd_pos.angular.z = 0;
        m_pubTwist.publish(out_cmd_pos);

        m_pubTwist.shutdown();
        m_subImg.shutdown();
        vpDisplay::close(I);
    }
}

void bebopVSNodelet::initVS()
{
    NODELET_INFO("Setting up visual servoing...");

    m_detector.setAprilTagQuadDecimate(4.0);
    m_detector.setAprilTagNbThreads(4);
    m_detector.setDisplayTag(true);

    m_cam.initPersProjWithoutDistortion(531.9213063, 520.8495788, 429.133986, 240.9464457);
    m_cam.printParameters();

    //double lambda = 0.5;
    vpAdaptiveGain lambda = vpAdaptiveGain(1.5, 0.7, 30);
    m_task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    m_task.setInteractionMatrixType(vpServo::CURRENT);
    m_task.setLambda(lambda);

    if (m_nh.hasParam("/vs/cameraTilt")) {
        m_nh.getParam("/vs/cameraTilt", m_cameraTilt);
    } else {
        ROS_WARN("No camera tilt value found, using default value : -15 degrees.");
        m_cameraTilt = -15.0;
    }

    if (m_nh.hasParam("/vs/cameraPan")) {
        m_nh.getParam("/vs/cameraPan", m_cameraPan);
    } else {
        ROS_WARN("No camera pan value found, using default value : 0 degrees.");
        m_cameraPan = 0.0;
    }

    std::stringstream camparams_str;
    camparams_str << "CAMERA PARAMETERS (degrees) : camera Tilt : " << m_cameraTilt
                  << " - camera pan : " << m_cameraPan << std::endl;
    NODELET_INFO("%s", camparams_str.str().c_str());

    vpRxyzVector c1_rxyz_c2(vpMath::rad(m_cameraTilt), vpMath::rad(m_cameraPan), 0);

    vpRotationMatrix c1Rc2(c1_rxyz_c2);                      // Rotation between camera 1 and 2
    vpHomogeneousMatrix c1Mc2(vpTranslationVector(), c1Rc2); // Homogeneous matrix between c1 and c2

    vpRotationMatrix c1Re{0, 1, 0, 0, 0, 1, 1, 0, 0}; // Rotation between camera 1 and E
    vpTranslationVector c1te(0, 0, -0.09);            // Translation between camera 1 and E
    vpHomogeneousMatrix c1Me(c1te, c1Re);             // Homogeneous matrix between c1 and E

    vpHomogeneousMatrix c2Me = c1Mc2.inverse() * c1Me; // Homogeneous matrix between c2 and E

    //vpVelocityTwistMatrix
    m_cVe = vpVelocityTwistMatrix(c2Me);
    m_task.set_cVe(m_cVe);

    m_eJe[0][0] = 1;
    m_eJe[1][1] = 1;
    m_eJe[2][2] = 1;
    m_eJe[5][3] = 1;

    // Define the desired polygon corresponding the the AprilTag CLOCKWISE
    double X[4] = {m_tagSize / 2., m_tagSize / 2., -m_tagSize / 2., -m_tagSize / 2.};
    double Y[4] = {m_tagSize / 2., -m_tagSize / 2., -m_tagSize / 2., m_tagSize / 2.};

    std::vector<vpPoint> vec_P, vec_P_d;
    for (int i = 0; i < 4; i++) {
        vpPoint P_d(X[i], Y[i], 0);
        vpHomogeneousMatrix cdMo(0, 0, m_Z_d, 0, 0, 0);
        P_d.track(cdMo); //
        vec_P_d.push_back(P_d);
    }

    // Desired moments
    m_obj_d.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
    m_obj_d.fromVector(vec_P_d); // Initialize the object with the points coordinates

    mb_d.linkTo(mdb_d);       // Add basic moments to database
    mg_d.linkTo(mdb_d);       // Add gravity center to database
    mc_d.linkTo(mdb_d);       // Add centered moments to database
    man_d.linkTo(mdb_d);      // Add area normalized to database
    mgn_d.linkTo(mdb_d);      // Add gravity center normalized to database
    mdb_d.updateAll(m_obj_d); // All of the moments must be updated, not just an_d
    mg_d.compute();           // Compute gravity center moment
    mc_d.compute();           // Compute centered moments AFTER gravity center

    if (m_obj_d.getType() == vpMomentObject::DISCRETE)
        area = mb_d.get(2, 0) + mb_d.get(0, 2);
    else
        area = mb_d.get(0, 0);
    // Update moment with the desired area
    man_d.setDesiredArea(area);

    man_d.compute(); // Compute area normalized moment AFTER centered moments
    mgn_d.compute(); // Compute gravity center normalized moment AFTER area normalized moment

    // Add the features
    m_task.addFeature(s_mgn, s_mgn_d);
    m_task.addFeature(s_man, s_man_d);
    m_task.addFeature(s_vp, s_vp_d, vpFeatureVanishingPoint::selectAtanOneOverRho());

    // Update desired gravity center normalized feature
    s_mgn_d.update(A, B, C);
    s_mgn_d.compute_interaction();
    // Update desired area normalized feature
    s_man_d.update(A, B, C);
    s_man_d.compute_interaction();

    // Update desired vanishing point feature for the horizontal line
    s_vp_d.setAtanOneOverRho(0);
    s_vp_d.setAlpha(0);

    NODELET_INFO("Visual servoing settup.");
}

vpColVector bebopVSNodelet::velocityToPosition(vpColVector &vel_cmd, double delta_t)
{
    vpColVector res;
    if (vel_cmd.size() != 4) {
        ROS_ERROR(
            "Can't compute velocity : dimension of the velocity vector should be equal to 4.");
        res << 0., 0., 0., 0.;
        return res;
    }

    vpColVector ve(6);

    ve[0] = vel_cmd[0];
    ve[1] = vel_cmd[1];
    ve[2] = vel_cmd[2];
    ve[5] = vel_cmd[3];

    vpHomogeneousMatrix M = vpExponentialMap::direct(ve, delta_t);

    double epsilon = (std::numeric_limits<double>::epsilon());
    if (std::abs(M.getRotationMatrix().getThetaUVector()[0]) >= epsilon) {
        ROS_ERROR("Can't compute velocity : rotation around X axis should be 0.");
        res << 0., 0., 0., 0.;
        return res;
    }
    if (std::abs(M.getRotationMatrix().getThetaUVector()[1]) >= epsilon) {
        ROS_ERROR("Can't compute velocity : rotation around Y axis should be 0.");
        res << 0., 0., 0., 0.;
        return res;
    }

    float dThetaZ = static_cast<float>(M.getRotationMatrix().getThetaUVector()[2]);
    vpTranslationVector t = M.getTranslationVector();

    res << t[0], t[1], t[2], dThetaZ;
    return res;
}
