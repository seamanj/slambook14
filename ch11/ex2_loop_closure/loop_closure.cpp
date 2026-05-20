#include "fbow/fbow.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

/***************************************************
 * 使用fbow进行回环检测演示
 * 根据fbow的API重新实现
 * ************************************************/
int main(int argc, char** argv)
{
    // 读取预训练的词汇表
    cout << "reading vocabulary" << endl;
    fbow::Vocabulary vocab;
    
    // 尝试读取词汇表文件
    try {
        vocab.readFromFile("../vocabulary_fbow.yml");
    } catch (const exception& e) {
        cerr << "Failed to load vocabulary: " << e.what() << endl;
        return 1;
    }
    
    if (!vocab.isValid() || vocab.size() == 0)
    {
        cerr << "Vocabulary does not exist or is empty." << endl;
        return 1;
    }
    
    cout << "Vocabulary info: " << endl;
    cout << "  Size (number of blocks): " << vocab.size() << endl;
    cout << "  K (branching factor): " << vocab.getK() << endl;
    cout << "  Descriptor type: " << vocab.getDescType() << endl;
    cout << "  Descriptor size: " << vocab.getDescSize() << " bytes" << endl;
    cout << "  Descriptor name: " << vocab.getDescName() << endl;
    
    // 读取图像
    cout << "\nreading images... " << endl;
    vector<Mat> images;
    for (int i = 0; i < 10; i++)
    {
        string path = "../data/" + to_string(i + 1) + ".png";
        Mat img = imread(path);
        if (img.empty())
        {
            cerr << "Failed to load image: " << path << endl;
            continue;
        }
        images.push_back(img);
    }
    
    if (images.empty())
    {
        cerr << "No images loaded!" << endl;
        return 1;
    }
    
    // 检测ORB特征
    cout << "\ndetecting ORB features ... " << endl;
    Ptr<Feature2D> detector = ORB::create();
    vector<cv::Mat> descriptors;
    vector<vector<cv::KeyPoint>> allKeypoints;
    
    for (Mat& image : images)
    {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        
        if (descriptor.empty())
        {
            cerr << "Warning: No features detected in one image!" << endl;
            descriptor = cv::Mat();
        }
        
        allKeypoints.push_back(keypoints);
        descriptors.push_back(descriptor);
    }
    
    // 将所有描述符转换为词袋向量 (fBow)
    cout << "\ntransforming descriptors to BoW vectors..." << endl;
    vector<fbow::fBow> bow_vectors;
    for (size_t i = 0; i < descriptors.size(); i++)
    {
        if (descriptors[i].empty())
        {
            bow_vectors.push_back(fbow::fBow());
            continue;
        }
        
        // 使用transform函数将描述符转换为词袋向量
        fbow::fBow bow_vector = vocab.transform(descriptors[i]); // tj : map<id, weight>
        bow_vectors.push_back(bow_vector);
        
        cout << "Image " << i << " has " << bow_vector.size() << " words" << endl;
    }
    
    // 使用fBow::score函数计算图像之间的相似度
    cout << "\ncomparing images with images (using fBow::score) " << endl;
    cout << "Note: Higher score means more similar" << endl;
    cout << "----------------------------------------" << endl;
    
    for (size_t i = 0; i < bow_vectors.size(); i++)
    {
        if (bow_vectors[i].empty()) continue;
        
        for (size_t j = i; j < bow_vectors.size(); j++)
        {
            if (bow_vectors[j].empty()) continue;
            
            // 使用fBow::score计算相似度分数
            // 根据fbow.h，score函数返回两个词袋向量之间的相似度
            double score = fbow::fBow::score(bow_vectors[i], bow_vectors[j]);
            cout << "image " << i << " vs image " << j << " : score = " << score << endl;
        }
        cout << endl;
    }
    
    // 输出相似度矩阵（可选）
    cout << "\n========================================" << endl;
    cout << "Similarity matrix:" << endl;
    cout << "========================================" << endl;
    
 

    cout << "     ";
    for (int j = 0; j < bow_vectors.size(); j++)
    {
        cout << "  img" << j << "  ";
    }
    cout << endl;
    
    for (int i = 0; i < bow_vectors.size(); i++)
    {
        if (bow_vectors[i].empty()) continue;
        cout << "img" << i << " ";
        for (int j = 0; j < bow_vectors.size(); j++)
        {
            if (bow_vectors[j].empty()) continue;
            double score = fbow::fBow::score(bow_vectors[i], bow_vectors[j]);
            printf(" %6.3f ", score);
        }
        cout << endl;
    }
    
    cout << "\ndone." << endl;
    return 0;
}