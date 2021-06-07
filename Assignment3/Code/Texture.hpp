//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getTexureColor(float u, float v){
        auto color = image_data.at<cv::Vec3b>(v, u);
        return Eigen::Vector3f(color[0], color[1], color[2]);
        
    }

    Eigen::Vector3f getColorBilinear (float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        
        Eigen::Vector3f texel_01 = getTexureColor(std::floor(u_img), std::ceil(v_img));
        Eigen::Vector3f texel_00 = getTexureColor(std::floor(u_img), std::floor(v_img));
        Eigen::Vector3f texel_10 = getTexureColor(std::ceil(u_img), std::floor(v_img));
        Eigen::Vector3f texel_11 = getTexureColor(std::ceil(u_img), std::ceil(v_img));
        


        float s = u_img - std::floor(u_img);
        float t = v_img - std::floor(v_img);

        // horizontal interpolation twice
        Eigen::Vector3f u_0 = lerp(s, texel_00, texel_10);
        Eigen::Vector3f u_1 = lerp(s, texel_01, texel_11);
        // vertical interpolation
        Eigen::Vector3f final = lerp(t, u_0, u_1);

        return Eigen::Vector3f(final.x(), final.y(), final.z());

    }

    Eigen::Vector3f lerp(float x, Eigen::Vector3f v_0, Eigen::Vector3f v_1)
    {
        return v_0 + x* (v_1 - v_0);
    }

};
#endif //RASTERIZER_TEXTURE_H
