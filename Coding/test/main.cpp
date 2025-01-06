#include <iostream>
#include <filesystem>
#include <iomanip>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include "cloth_renderer.hpp"
#include "cloth_simulator.hpp"
#include "world_frame.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "sphere.h"

#define SAVE_IMAGE true

// constants
const int Width = 1024;
const int Height = 768;
extern const float timeStep = 1.0f / 120.0f;

// global
glm::vec3 mouseRay = glm::vec3(0.0f);
bool scratching = false;


void processCameraInput(GLFWwindow* window, FirstPersonCamera* camera);
void saveImage(const char* filepath, GLFWwindow* w);
void deleteDirectoryContents(const std::filesystem::path& dir)
{
    for (const auto& entry : std::filesystem::directory_iterator(dir)) 
        std::filesystem::remove_all(entry.path());
}
std::ostream& operator<<(std::ostream& os, const glm::vec3& vec)
{
    os << vec.x << " " << vec.y << " " << vec.z << '\n';
    return os;
}
std::ostream& operator<<(std::ostream& os, const glm::mat4& mat) {
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            os << mat[row][col] << " ";
        }
        os << '\n';
    }
    return os;
}

int main(int argc, char* argv[])
{

    /// Clean last kept images
    unsigned long long countFrames = 1; // use for store images
    deleteDirectoryContents("../Coding/imgs");

    GLFWwindow* window;

    // Window setup
    {
        if (!glfwInit()) // Initialize glfw library
            return -1;

        // setting glfw window hints and global configurations
        {
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // Use Core Mode
            // glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE); // Use Debug Context
        #ifdef __APPLE__
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // fix compilation on OS X
        #endif
        }

        // Create a windowed mode window and its OpenGL context
        window = glfwCreateWindow(Width, Height, "Cloth Simulation with Mass Spring", NULL, NULL);
        if (!window) {
            glfwTerminate();
            return -1;
        }

        // window configurations
        {
            // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }

        // Make the window's context current
        glfwMakeContextCurrent(window);

        // Load Opengl
        if (!gladLoadGL()) {
            glfwTerminate();
            return -1;
        };

        glEnable(GL_DEPTH_TEST);
    }

    // Main Loop
    {
        // TODO: Tune overall and cloth settings here (especially shader path)

        // Overall settings
        auto vertexShader = "../Coding/res/shader/cloth.vs";
        auto fragmentShader = "../Coding/res/shader/cloth.fs";

        /// axis settings
        WorldFrame wf;
        Shader axisShader("../Coding/res/shader/axis.vs",
            "../Coding/res/shader/axis.fs");

        // Cloth settings
        unsigned int nWidth = 30;
        unsigned int nHeight = 20;
        float dx = 0.1f;
        auto clothTransform = glm::rotate(glm::mat4(1.0f),
                                          glm::radians(60.0f), {1.0f, 0.0f, 0.0f}); // Represents a rotation of 60 degrees around the x-axis.
        float totalMass = 1.0f;
        float stiffnessReference = 10.0f;
        float airResistanceCoefficient = 0.001f;
        glm::vec3 gravity = {0.0f, -9.81f, 0.0f};

        // Create objects
        Shader shader(vertexShader, fragmentShader);
        FirstPersonCamera camera;
        RectCloth cloth(nWidth, nHeight, dx, clothTransform);
        RectClothRenderer renderer(&shader, &camera, &cloth);
        RectClothSimulator simulator(&cloth, totalMass, stiffnessReference, airResistanceCoefficient, gravity);
        Sphere sphere({ 0.5f, -1.5f, 0.0f }, 0.47f);
        // Setup iteration variables
        float currentTime = (float)glfwGetTime();
        float lastTime = currentTime;
        float deltaTime = 0.0f;

        int totalIterCount = 0;
        float totalIterTime = 0.0f;
        float overTakenTime = 0.0f;

        // Loop until the user closes the window
        while (!glfwWindowShouldClose(window))
        {
            // Terminate condition
            if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            // Updating
            {
                // Calculate dt
                currentTime = static_cast<float>(glfwGetTime());
                deltaTime = currentTime - lastTime - overTakenTime; // maintain the deltaTime not get too large
                overTakenTime = 0.0f;
                lastTime = currentTime;

                processCameraInput(window, &camera);

                /// Debug Update here only when p is pressed
                // if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
                if (true)
                {
                    // A fixed time step which should not be too large in order to stabilize the simulation
                    totalIterTime += deltaTime;
                    float curIterTime = totalIterTime - (float)totalIterCount * timeStep;
                    int iterCount = (int)roundf(curIterTime / timeStep);


                    iterCount = 1; //here
                    for (int i = 0; i < iterCount; ++i) {
                        totalIterCount += 1;

                        // Simulate one step
                        simulator.step_fast();
                        simulator.updateScratchPoint(camera.getCameraPos(), mouseRay, scratching);
                        // std::cout << camera.getCameraPos() << camera.getView();

                        float timeTaken = static_cast<float>(glfwGetTime()) - currentTime;
                        if (timeTaken > deltaTime) {
                            overTakenTime = timeTaken - deltaTime;
                            break;
                        }
                    }
                }
            }

            glClearColor(0.25F, 0.25F, 0.25F, 0.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Draw here
            renderer.draw();
            sphere.draw();
            /// axis
            axisShader.use();
            axisShader.setMat4("projection", camera.getProjection());
            axisShader.setMat4("view", camera.getView());
            wf.draw(axisShader.id);

        #if SAVE_IMAGE
            /// write window images to dir
            if (countFrames % 2ull == 0)
            {
                std::ostringstream oss;
                oss << "../Coding/imgs/" 
                    << std::setw(5) << std::setfill('0') << countFrames << ".jpeg";
                std::string filePath = oss.str();
                saveImage(filePath.c_str(), window);
            }
            ++countFrames;
        #endif

            // Swap front and back buffers
            glfwSwapBuffers(window);

            // Poll for and process events
            glfwPollEvents();
        }
    }

    glfwTerminate();
    return 0;
}

void processCameraInput(GLFWwindow* window, FirstPersonCamera* camera)
{
    static bool firstRun {true};
    static float lastFrame {0};

    static float lastCursorX {0};
    static float lastCursorY {0};

    double curCursorX, curCursorY;
    glfwGetCursorPos(window, &curCursorX, &curCursorY);

    float currentFrame = static_cast<float>(glfwGetTime());

    if (firstRun) {
        lastFrame = currentFrame;
        firstRun = false;

        lastCursorX = static_cast<float>(curCursorX);
        lastCursorY = static_cast<float>(curCursorY);

        return; // everything zero, so we return directly
    }

    float deltaTime = currentFrame - lastFrame;
    float deltaCursorX = curCursorX - lastCursorX;
    float deltaCursorY = curCursorY - lastCursorY;

    float cameraMoveSpeed = 3.5f * deltaTime;
    float cameraRotateSpeed = 1.0f * deltaTime;

	if((glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_S) != GLFW_PRESS))
        camera->moveForward(cameraMoveSpeed);
    if((glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_W) != GLFW_PRESS))
        camera->moveBackward(cameraMoveSpeed);
    if((glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_A) != GLFW_PRESS))
        camera->moveRight(cameraMoveSpeed);
    if((glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_D) != GLFW_PRESS))
        camera->moveLeft(cameraMoveSpeed);
    if((glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) != GLFW_PRESS))
        camera->moveUp(cameraMoveSpeed);
    if((glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_SPACE) != GLFW_PRESS))
        camera->moveDown(cameraMoveSpeed);

    if((glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_RIGHT) != GLFW_PRESS))
        camera->lookRight(cameraRotateSpeed);
    if((glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_LEFT) != GLFW_PRESS))
        camera->lookLeft(cameraRotateSpeed);
    if((glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_DOWN) != GLFW_PRESS))
        camera->lookUp(cameraRotateSpeed);
    if((glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_UP) != GLFW_PRESS))
        camera->lookDown(cameraRotateSpeed);
//    if((glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_E) != GLFW_PRESS))
//        camera->rotateLeft(cameraRotateSpeed);
//    if((glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) && (glfwGetKey(window, GLFW_KEY_Q) != GLFW_PRESS))
//        camera->rotateRight(cameraRotateSpeed);

    // camera->lookLeft(cameraRotateSpeed * deltaCursorX * 0.2f);
    // camera->lookDown(cameraRotateSpeed * deltaCursorY * 0.2f);


    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        mouseRay = camera->getMouseRay(curCursorX, curCursorY, Width, Height);
        scratching = true;
    }
    else {
        scratching = false;
    }

    // update record
    lastCursorX = static_cast<float>(curCursorX);
    lastCursorY = static_cast<float>(curCursorY);

    lastFrame = currentFrame;
}

/// @brief https://lencerf.github.io/post/2019-09-21-save-the-opengl-rendering-to-image-file/
void saveImage(const char* filepath, GLFWwindow* w) {
    int width, height;
    glfwGetFramebufferSize(w, &width, &height);
    GLsizei nrChannels = 3;
    GLsizei stride = nrChannels * width;
    stride += (stride % 4) ? (4 - stride % 4) : 0;
    GLsizei bufferSize = stride * height;
    std::vector<char> buffer(bufferSize);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buffer.data());
    stbi_flip_vertically_on_write(true);
    stbi_write_jpg(filepath, width, height, nrChannels, buffer.data(), stride);
}
