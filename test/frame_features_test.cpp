#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <chrono>
#include <sstream>

#include "frame_features.h"

TEST(FrameFeaturesTest, TwoConsecutiveImages) {
    auto detector = cv::FeatureDetector::create("ORB");
    detector->set("nFeatures", 50);
    detector->set("nLevels", 12);
    detector->set("scaleFactor", 1.2f);
    detector->set("WTA_K", 4);
    auto extractor = cv::DescriptorExtractor::create("ORB");
    
//    cv::Ptr<cv::DescriptorMatcher> matcher(new cv::BFMatcher(cv::NORM_HAMMING2, true));
//    auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    cv::Ptr<cv::DescriptorMatcher> matcher(new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20,10,2)));
    
    double scale = 0.8;
    
    cv::Mat img1 = cv::imread("/Users/tomaskrejci/kitti/2011_09_30/2011_09_30_drive_0028_sync/image_00/data/0000000000.png");
    cv::Mat img1_resized;
    cv::resize(img1, img1_resized, cv::Size(), scale, scale);
    FrameFeatures frame_features_1 = FrameFeatures::fromImage(detector, extractor, img1_resized);
    
    cv::Mat img2 = cv::imread("/Users/tomaskrejci/kitti/2011_09_30/2011_09_30_drive_0028_sync/image_00/data/0000000001.png");
    cv::Mat img2_resized;
    cv::resize(img2, img2_resized, cv::Size(), scale, scale);
    FrameFeatures frame_features_2 = FrameFeatures::fromImage(detector, extractor, img2_resized);
    
    std::vector<cv::DMatch> matches = frame_features_2.match(matcher, frame_features_1);
    
    
    
    cv::Mat matches_img;
    cv::drawMatches(img1_resized, frame_features_1.keypoints(), img2_resized, frame_features_2.keypoints(), matches, matches_img);
    cv::imwrite("/Users/tomaskrejci/FrameFeaturesTest_TwoConsecutiveImages.png", matches_img);
    
    
}

TEST(FrameFeaturesTest, DISABLED_MatchFeatures_Detector_ORB_Extractor_ORB_Matcher_BF) {
    std::vector<int> n_features_to_try = { 25, 50, 75, 100, 150, 200, 250, 300, 350, 400, 450, 500, 600, 700, 800, 900, 1000, 1200, 1400, 1600, 1800, 2000 };
    for (int n_features : n_features_to_try) {
        std::cout << " !!! processing " << n_features << std::endl;
        
        cv::ORB detector(n_features);
        cv::ORB descriptor(n_features);
        cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));
        
        std::string base_dir_name = "/Users/tomaskrejci/kitti/2011_09_30/2011_09_30_drive_0028_sync/image_00/data/";
        cv::Mat train_img = cv::imread(base_dir_name + "0000000000.png");
        std::vector<cv::KeyPoint> train_keypoints;
        detector.detect(train_img, train_keypoints);
        cv::Mat train_descriptor;
        descriptor.compute(train_img, train_keypoints, train_descriptor);
        
        auto total_detect_duration = std::chrono::duration<double>::zero();
        auto total_describe_duration = std::chrono::duration<double>::zero();
        auto total_match_duration = std::chrono::duration<double>::zero();
        auto total_homography_duration = std::chrono::duration<double>::zero();
        
        int good_features_count = 0;
        int total_features_count = 0;
        
        std::size_t num_images_to_process = 50;
        for (int query_image_number = 0; query_image_number < num_images_to_process; ++query_image_number) {
            // LOAD
            
            std::stringstream ss;
            ss << std::setw(10) << std::setfill('0') << query_image_number;
            std::string query_img_file_name = base_dir_name + ss.str() + ".png";
            
            cv::Mat query_img = cv::imread(query_img_file_name);
            
            // DETECT
            
            std::vector<cv::KeyPoint> query_keypoints;
            
            auto start_detection = std::chrono::high_resolution_clock::now();
            detector.detect(query_img, query_keypoints);
            auto end_detection = std::chrono::high_resolution_clock::now();
            total_detect_duration += end_detection - start_detection;
            
            // DESCRIBE
            
            cv::Mat query_descriptor;
            auto start_description = std::chrono::high_resolution_clock::now();
            descriptor.compute(query_img, query_keypoints, query_descriptor);
            auto end_description = std::chrono::high_resolution_clock::now();
            total_describe_duration += end_description - start_description;
            
            // MATCH
            
            std::vector<cv::DMatch> matches;
            auto start_matching = std::chrono::high_resolution_clock::now();
            matcher.match(query_descriptor, train_descriptor, matches);
            auto end_matching = std::chrono::high_resolution_clock::now();
            total_match_duration += end_matching - start_matching;
            // std::cout << "Matches: " << matches.size() << std::endl;
            
            // FIND HOMOGRAPHY
            
            std::vector<cv::Point2d> train_pts(matches.size());
            std::vector<cv::Point2d> query_pts(matches.size());
            for (std::size_t i = 0; i < matches.size(); ++i) {
                train_pts[i] = train_keypoints[matches[i].trainIdx].pt;
                query_pts[i] = query_keypoints[matches[i].queryIdx].pt;
            }
            cv::Mat good_features_mask;
            auto start_homography = std::chrono::high_resolution_clock::now();
            cv::Mat H = cv::findHomography(query_pts, train_pts, CV_RANSAC, 3, good_features_mask);
            auto end_homography = std::chrono::high_resolution_clock::now();
            total_homography_duration += end_homography - start_homography;
            // std::cout << "Good features mask size: " << good_features_mask.rows << std::endl;
            total_features_count += matches.size();
            for (std::size_t i = 0; i < matches.size(); ++i) {
                if (good_features_mask.at<bool>(i, 0)) {
                    good_features_count += 1;
                }
            }
            
            // DRAW MATCHES
            
            cv::Scalar match_color = cv::Scalar::all(-1);
            cv::Scalar single_point_color = cv::Scalar::all(-1);
            
            std::string base_output_dir = "/Users/tomaskrejci/";
            cv::Mat matches_img;
            cv::drawMatches(query_img, query_keypoints, train_img, train_keypoints, matches, matches_img, match_color, single_point_color, good_features_mask);
            cv::imwrite(base_output_dir + "/matches_" + ss.str() + ".png", matches_img);
        }
        
        std::cout << "Average detection time " << std::chrono::duration_cast<std::chrono::microseconds>(total_detect_duration).count() / num_images_to_process << " us" << std::endl;
        std::cout << "Average description time " << std::chrono::duration_cast<std::chrono::microseconds>(total_describe_duration).count() / num_images_to_process << " us" << std::endl;
        std::cout << "Average match time " << std::chrono::duration_cast<std::chrono::microseconds>(total_match_duration).count() / num_images_to_process << " us" << std::endl;
        std::cout << "Average homography time " << std::chrono::duration_cast<std::chrono::microseconds>(total_homography_duration).count() / num_images_to_process << " us" << std::endl;
        std::cout << "Homography test passed by " << good_features_count << " out of " << total_features_count << " features." << std::endl;
    }
}







