#include "medianFilter.hpp"

medianFilter::medianFilter()
{
}

medianFilter::~medianFilter()
{
}

float medianFilter::getStdDev(std::vector<cv::Point2f> temp)
{

    float std_dev = 0, mean = 0, variance = 0, sum = 0;

    points = getL2Norm(temp);

    for (int i = 0; i < points.size(); i++)
    {
        sum = sum + points[i];
    }

    mean = sum / points.size();

    for (int i = 0; i < points.size(); i++)
    {
        variance = variance + pow((points[i] - mean), 2);
    }

    variance = variance / points.size();
    std_dev = sqrt(variance);
    points.clear();
    return std_dev;
}

cv::Point2f medianFilter::getMedian(std::vector<cv::Point2f> temp, cv::Point2f origin)
{
    points = getL2Norm(temp);
    std::vector<float> min_dis;
    float origin_dis = sqrt((pow(origin.x, 2)) + (pow(origin.y, 2)));
    for (int i = 0; i < points.size(); i++)
    {
        min_dis.push_back(abs(origin_dis - points[i]));
    }
    int min_pos = distance(min_dis.begin(), min_element(min_dis.begin(), min_dis.end()));
    points.clear();
    return temp[min_pos];
}

std::vector<float> medianFilter::getL2Norm(std::vector<cv::Point2f> temp)
{
    std::vector<float> points_of_interest;

    for (int i = 0; i < temp.size(); i++)
    {
        points_of_interest.push_back(sqrt((pow(temp[i].x, 2)) + pow(temp[i].y, 2)));
    }
    return points_of_interest;
}
