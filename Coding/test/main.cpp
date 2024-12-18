#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include "cloth_renderer.hpp"
#include "cloth_simulator.hpp"


glm::vec3 mouseRay = glm::vec3(0.0f);
bool scratching = false;
int Width = 1024;
int Height = 768;
bool lock = false;



void processCameraInput(GLFWwindow* window, FirstPersonCamera* camera);

int main(int argc, char* argv[])
{
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
        window = glfwCreateWindow(1024, 768, "Cloth Simulation with Mass Spring", NULL, NULL);
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
        auto vertexShader = "D:/class/CS171/hw5/cs171-hw5-CoLLinZzZ-master_final/cs171-hw5-CoLLinZzZ-master/Coding/res/shader/cloth.vs";
        auto fragmentShader = "D:/class/CS171/hw5/cs171-hw5-CoLLinZzZ-master_final/cs171-hw5-CoLLinZzZ-master/Coding/res/shader/cloth.fs";
        const float timeStep = 0.002f;

        // Cloth settings
        unsigned int nWidth = 10;
        unsigned int nHeight = 10;
        float dx = 0.1f;
        auto clothTransform = glm::rotate(glm::mat4(1.0f),
                                          glm::radians(60.0f), {1.0f, 0.0f, 0.0f}); // Represents a rotation of 60 degrees around the x-axis.
        float totalMass = 1.0f;
        float stiffnessReference = 40.0f;
        float airResistanceCoefficient = 0.001f;
        glm::vec3 gravity = {0.0f, -9.81f, 0.0f};

        // Create objects
        Shader shader(vertexShader, fragmentShader);
        FirstPersonCamera camera;
        RectCloth cloth(nWidth, nHeight, dx, clothTransform);
        RectClothRenderer renderer(&shader, &camera, &cloth);
        RectClothSimulator simulator(&cloth, totalMass, stiffnessReference, airResistanceCoefficient, gravity);

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

                // Debug Update here only when p is pressed
                //if(glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
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
                        simulator.updateScratchPoint(camera.getCameraPos(), mouseRay, scratching, lock);
                        


                        float timeTaken = static_cast<float>(glfwGetTime()) - currentTime;
                        if (timeTaken > deltaTime) {
                            overTakenTime = timeTaken - deltaTime;
                            break;
                        }
                    }
                }
            }

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Draw here
            renderer.draw();
            lock = false;

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

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        mouseRay = camera->getMouseRay(curCursorX, curCursorY, Width, Height);
        lock = true;
    }

    // update record
    lastCursorX = static_cast<float>(curCursorX);
    lastCursorY = static_cast<float>(curCursorY);

    lastFrame = currentFrame;
}
