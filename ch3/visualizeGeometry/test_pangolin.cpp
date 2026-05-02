#include <pangolin/pangolin.h>

int main() {
    pangolin::CreateWindowAndBind("Test", 640, 480);
    glEnable(GL_DEPTH_TEST);
    
    while(!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        pangolin::FinishFrame();
    }
    return 0;
}