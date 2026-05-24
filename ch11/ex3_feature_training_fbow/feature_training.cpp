#include <fbow/fbow.h>
#include <fbow/vocabulary_creator.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // read the images
    cout << "reading images... " << endl;
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
        return -1;
    }
    
    // detect ORB features
    cout << "detecting ORB features ... " << endl;
    Ptr<Feature2D> detector = ORB::create();
    
    vector<Mat> allDescriptors;
    
    for (size_t i = 0; i < images.size(); i++)
    {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(images[i], Mat(), keypoints, descriptor);
        if (!descriptor.empty())
        {
            allDescriptors.push_back(descriptor);
            cout << "Image " << i + 1 << ": " << keypoints.size() << " features" << endl;
        }
        else
        {
            cerr << "No features detected in image " << i + 1 << endl;
        }
    }
    
    if (allDescriptors.empty())
    {
        cerr << "No descriptors extracted!" << endl;
        return -1;
    }
    
    // 合并所有描述子
    Mat allFeatures;
    vconcat(allDescriptors, allFeatures);
    cout << "Total features: " << allFeatures.rows << endl;
    cout << "Feature type: " << allFeatures.type() << endl;
    cout << "Feature size: " << allFeatures.cols << endl;
    
    // 使用 VocabularyCreator 创建词汇表
    cout << "creating vocabulary with fbow... " << endl;
    
    fbow::VocabularyCreator creator;
    fbow::Vocabulary vocab;
    
    // 设置参数
    fbow::VocabularyCreator::Params params;
    params.k = 10;           // 分支数 (branching factor)
    params.L = 6;            // 层级数 (depth level) - 词汇量 = k^L = 10^6
    params.nthreads = 4;     // 线程数
    params.maxIters = 11;    // 最大迭代次数
    params.verbose = true;   // 显示详细信息
    
    cout << "Vocabulary parameters: k=" << params.k << ", L=" << params.L << endl;
    cout << "This will create a vocabulary with up to " << pow(params.k, params.L) << " words" << endl;
    
    // 创建词汇表（注意：第一个参数是引用，会被填充）
    // 描述子类型名称，对于 ORB 通常使用 "ORB"
    string desc_name = "ORB";
    
    creator.create(vocab, allFeatures, desc_name, params);
    
    // 保存词汇表
    vocab.saveToFile("vocabulary_fbow.yml");
    cout << "Vocabulary saved to vocabulary_fbow.yml" << endl;
    
    // 输出词汇表信息
    if (vocab.isValid())
    {
        cout << "\nVocabulary info: " << endl;
        cout << "  - Valid: yes" << endl;
        cout << "  - Size (blocks): " << vocab.size() << endl;
        cout << "  - K (branching factor): " << vocab.getK() << endl;
        cout << "  - Descriptor type: " << vocab.getDescType() << endl;
        cout << "  - Descriptor size: " << vocab.getDescSize() << endl;
        cout << "  - Descriptor name: " << vocab.getDescName() << endl;
        
        // 测试转换
        if (!allDescriptors.empty())
        {
            cout << "\nTesting transform on first image..." << endl;
            fbow::fBow fbow_vector = vocab.transform(allDescriptors[0]);
            cout << "  - Number of words in BOW vector: " << fbow_vector.size() << endl;
            
            // 显示前5个单词
            int count = 0;
            for (auto& w : fbow_vector)
            {
                cout << "  - Word " << w.first << ": weight = " << w.second << endl;
                if (++count >= 5) break;
            }
        }
    }
    else
    {
        cout << "Vocabulary is invalid!" << endl;
    }
    

/*
总图像数：100 张
当前图像总特征数：100 个

单词 "车轮" (Word 0):
- 在当前图中出现 10 次
- 只在 20 张图中出现过
IDF = log(100/20) = 1.61
TF  = 10/100 = 0.1 
weight = TF × IDF = 0.1 × 1.61 = 0.161 (较高, 有区分度)

单词 "天空" (Word 1):
- 在当前图中出现 20 次  
- 在 95 张图中都出现过
IDF = log(100/95) = 0.051
TF = 20/100 = 0.2 
weight = TF × IDF = 0.2 × 0.051 = 0.0102 (较低)


Weight 值	含义	说明
高 (0.08+)	单词很重要	该视觉元素在图中出现多次，且有区分度
中 (0.03-0.07)	普通重要	正常出现频率
低 (0.03-)	不太重要	该元素在图中很少，或很多图都有
 */

cout << "\n=== Detailed Analysis ===" << endl;
cout << "Blocks: " << vocab.size() << endl;
cout << "Max words possible (theoretical): " << pow(vocab.getK(), 6) << endl;
cout << "Current words in first image: 100" << endl;
cout << "\nExplanation: " << endl;
cout << "- Each block contains up to " << vocab.getK() << " nodes" << endl;
cout << "- " << vocab.size() << " blocks × " << vocab.getK() 
     << " nodes/block = up to " << vocab.size() * vocab.getK() << " nodes" << endl;
cout << "- The actual number of leaf nodes (words) is ~100" << endl;

    cout << "\nDone!" << endl;
    
    return 0;
}