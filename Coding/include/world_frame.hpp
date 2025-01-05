#ifndef WORLD_FRAME_
#define WORLD_FRAME_

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

float axisVertices[] = {
    // X axis (red)
    0.0f, 0.0f, 0.0f,  // Origin
    1.0f, 0.0f, 0.0f,  // X-axis

    // Y axis (green)
    0.0f, 0.0f, 0.0f,  // Origin
    0.0f, 1.0f, 0.0f,  // Y-axis

    // Z axis (blue)
    0.0f, 0.0f, 0.0f,  // Origin
    0.0f, 0.0f, 1.0f   // Z-axis
};

class WorldFrame {
public:
    WorldFrame() : data(axisVertices) 
    {
        setup();
    }

    void setup()
    {
        glGenVertexArrays(1, &this->VAO_);
        glGenBuffers(1, &this->VBO_);

        glBindVertexArray(this->VAO_);
        glBindBuffer(GL_ARRAY_BUFFER, this->VBO_);

        // Upload the vertex data
        glBufferData(GL_ARRAY_BUFFER, sizeof(axisVertices), axisVertices, GL_STATIC_DRAW);

        // Define the vertex attributes (position)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }

    void draw(unsigned int shaderProgram)
    {
        glBindVertexArray(this->VAO_);

        // Draw X-axis (red)
        glUniform3f(glGetUniformLocation(shaderProgram, "color"), 1.0f, 0.0f, 0.0f);  // Red color
        glDrawArrays(GL_LINES, 0, 2);  // Draw the first 2 vertices (X-axis)

        // Draw Y-axis (green)
        glUniform3f(glGetUniformLocation(shaderProgram, "color"), 0.0f, 1.0f, 0.0f);  // Green color
        glDrawArrays(GL_LINES, 2, 2);  // Draw the next 2 vertices (Y-axis)

        // Draw Z-axis (blue)
        glUniform3f(glGetUniformLocation(shaderProgram, "color"), 0.0f, 0.0f, 1.0f);  // Blue color
        glDrawArrays(GL_LINES, 4, 2);  // Draw the last 2 vertices (Z-axis)

        glBindVertexArray(0);
    }

private:
    unsigned int VAO_, VBO_;
    float *data;
};

#endif