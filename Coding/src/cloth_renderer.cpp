#include "cloth_renderer.hpp"

RectClothRenderer::
RectClothRenderer(
    Shader* shader,
    FirstPersonCamera* camera,
    RectCloth* cloth
) {
    this->shader = shader;
    this->camera = camera;
    this->cloth = cloth;

    this->initVertices();
    this->initIndices();
    this->updateNormals();

    this->glo.initData();
}


GLint previous;
void RectClothRenderer::
draw() {
    this->updatePositions();
    this->updateNormals();


    const unsigned int nh = this->cloth->nh;
    const unsigned int nw = this->cloth->nw;
    unsigned int total = nh * nw;


    std::vector<GLuint> cur_indices;
    std::vector<VertexData> cur_vertices;


    for (int i = 0; i < this->glo.indices.size(); i += 3) {
        int vertice_idx_0 = this->glo.indices[i];
        int vertice_idx_1 = this->glo.indices[i + 1];
        int vertice_idx_2 = this->glo.indices[i + 2];
        bool flag = false;
        if (this->cloth->cutted[vertice_idx_0]) {
            flag = true;
            //add two triangle
            if (vertice_idx_0 >= nw && vertice_idx_0 % nw > 0) {
                if (!this->cloth->cutted[vertice_idx_0 - 1] && !this->cloth->cutted[vertice_idx_0 - nw] && !this->cloth->cutted[vertice_idx_0 - nw - 1]) {
                    cur_indices.push_back(vertice_idx_0 - 1);
                    cur_indices.push_back(vertice_idx_0 - nw);
                    cur_indices.push_back(vertice_idx_0 - nw - 1);
                }
            }
            if (vertice_idx_0 + nw < total && vertice_idx_0 % nw < nw - 1) {
                if (!this->cloth->cutted[vertice_idx_0 + 1] && !this->cloth->cutted[vertice_idx_0 + nw] && !this->cloth->cutted[vertice_idx_0 + nw + 1]) {
                    cur_indices.push_back(vertice_idx_0 + 1);
                    cur_indices.push_back(vertice_idx_0 + nw);
                    cur_indices.push_back(vertice_idx_0 + nw + 1);
                }
            }

        }
        if (this->cloth->cutted[vertice_idx_1]) {
            flag = true;
            if (vertice_idx_1 >= nw && vertice_idx_1 % nw > 0) {
                if (!this->cloth->cutted[vertice_idx_1 - 1] && !this->cloth->cutted[vertice_idx_1 - nw] && !this->cloth->cutted[vertice_idx_1 - nw - 1]) {
                    cur_indices.push_back(vertice_idx_1 - 1);
                    cur_indices.push_back(vertice_idx_1 - nw);
                    cur_indices.push_back(vertice_idx_1 - nw - 1);
                }
            }
            if (vertice_idx_1 + nw < total && vertice_idx_1 % nw < nw - 1) {
                if (!this->cloth->cutted[vertice_idx_1 + 1] && !this->cloth->cutted[vertice_idx_1 + nw] && !this->cloth->cutted[vertice_idx_1 + nw + 1]) {
                    cur_indices.push_back(vertice_idx_1 + 1);
                    cur_indices.push_back(vertice_idx_1 + nw);
                    cur_indices.push_back(vertice_idx_1 + nw + 1);
                }
            }
        }
        if (this->cloth->cutted[vertice_idx_2]) {
            flag = true;
            if (vertice_idx_2 >= nw && vertice_idx_2 % nw > 0) {
                if (!this->cloth->cutted[vertice_idx_2 - 1] && !this->cloth->cutted[vertice_idx_2 - nw] && !this->cloth->cutted[vertice_idx_2 - nw - 1]) {
                    cur_indices.push_back(vertice_idx_2 - 1);
                    cur_indices.push_back(vertice_idx_2 - nw);
                    cur_indices.push_back(vertice_idx_2 - nw - 1);
                }
            }
            if (vertice_idx_2 + nw < total && vertice_idx_2 % nw < nw - 1) {
                if (!this->cloth->cutted[vertice_idx_2 + 1] && !this->cloth->cutted[vertice_idx_2 + nw] && !this->cloth->cutted[vertice_idx_2 + nw + 1]) {
                    cur_indices.push_back(vertice_idx_2 + 1);
                    cur_indices.push_back(vertice_idx_2 + nw);
                    cur_indices.push_back(vertice_idx_2 + nw + 1);
                }
            }
        }
        if (!flag) {
            cur_indices.push_back(vertice_idx_0);
            cur_indices.push_back(vertice_idx_1);
            cur_indices.push_back(vertice_idx_2);
        }
        else {
        }
    }







    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->glo.EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * cur_indices.size(), cur_indices.data(), GL_STATIC_DRAW);








    // Update Data
    glBindBuffer(GL_ARRAY_BUFFER, glo.VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexData) * this->glo.vertices.size(), this->glo.vertices.data(), GL_STREAM_DRAW);

    
    glGetIntegerv(GL_POLYGON_MODE, &previous);

    this->shader->use();
    this->shader->setMat4("Projection", this->camera->getProjection());
    this->shader->setMat4("View", this->camera->getView());
    this->shader->setVec3("CameraPos", this->camera->getCameraPos());

    glBindVertexArray(glo.VAO);
    glBindBuffer(GL_ARRAY_BUFFER, glo.VBO);

    this->shader->setBool("DrawLine", false);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FLAT); // We want flat mode
    glDrawElements(GL_TRIANGLES, (int)cur_indices.size(), GL_UNSIGNED_INT, NULL);

    // this->shader->setBool("DrawLine", true);
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // We want line mode
    // glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(glo.indices.size()), GL_UNSIGNED_INT, 0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glPolygonMode(GL_FRONT_AND_BACK, previous); // restore previous mode
}

void RectClothRenderer::
initVertices() {
    const unsigned int nh = this->cloth->nh;
    const unsigned int nw = this->cloth->nw;
    const unsigned int total = nh * nw;

    this->glo.vertices.clear();
    this->glo.vertices.resize(total);

    for (unsigned int i = 0; i < total; ++i) {
        this->glo.vertices[i].position = this->cloth->getPosition(i);
    }
}


void RectClothRenderer::
initIndices() {
    this->glo.indices.clear();

    const unsigned int nh = this->cloth->nh;
    const unsigned int nw = this->cloth->nw;

    this->glo.indices.reserve((nh - 1) * (nw - 1) * 6);
    for (unsigned int ih = 0; ih < nh - 1; ++ih) {
        for (unsigned int iw = 0; iw < nw - 1; ++iw) {
            unsigned int leftDownIdx = this->cloth->idxFromCoord(iw, ih);
            unsigned int rightDownIdx = this->cloth->idxFromCoord(iw + 1, ih);
            unsigned int leftUpIdx = this->cloth->idxFromCoord(iw, ih + 1);
            unsigned int rightUpIdx = this->cloth->idxFromCoord(iw + 1, ih + 1);

            this->glo.indices.push_back(leftDownIdx);
            this->glo.indices.push_back(rightDownIdx);
            this->glo.indices.push_back(rightUpIdx);

            this->glo.indices.push_back(leftDownIdx);
            this->glo.indices.push_back(rightUpIdx);
            this->glo.indices.push_back(leftUpIdx);
        }
    }
}

void RectClothRenderer::
updatePositions() {
    const unsigned int nh = this->cloth->nh;
    const unsigned int nw = this->cloth->nw;
    const unsigned int total = nh * nw;

    for (unsigned int i = 0; i < total; ++i) {
        this->glo.vertices[i].position = this->cloth->getPosition(i);
    }
}

void RectClothRenderer::
updateNormals() {
    const unsigned int nh = this->cloth->nh;
    const unsigned int nw = this->cloth->nw;
    const unsigned int total = nh * nw;

    for (unsigned int i = 0; i < total; ++i) {
        this->glo.vertices[i].normal = glm::vec3(0);
    }

    for (unsigned int ih = 0; ih < nh - 1; ++ih) {
        for (unsigned int iw = 0; iw < nw - 1; ++iw) {
            unsigned int leftDownIdx = this->cloth->idxFromCoord(iw, ih);
            unsigned int rightDownIdx = this->cloth->idxFromCoord(iw + 1, ih);
            unsigned int leftUpIdx = this->cloth->idxFromCoord(iw, ih + 1);
            unsigned int rightUpIdx = this->cloth->idxFromCoord(iw + 1, ih + 1);

            unsigned int lower[3] = {leftDownIdx, rightDownIdx, rightUpIdx};
            unsigned int upper[3] = {leftDownIdx, rightUpIdx, leftUpIdx};
            glm::vec3 lowerNormal = calcNormal(
                this->glo.vertices[leftDownIdx].position,
                this->glo.vertices[rightDownIdx].position,
                this->glo.vertices[rightUpIdx].position
            );
            glm::vec3 upperNormal = calcNormal(
                this->glo.vertices[leftDownIdx].position,
                this->glo.vertices[rightUpIdx].position,
                this->glo.vertices[leftUpIdx].position
            );
            for (int i = 0; i < 3; ++i) {
                this->glo.vertices[lower[i]].normal += lowerNormal;
                this->glo.vertices[upper[i]].normal += upperNormal;
            }
        }
    }

    for (unsigned int i = 0; i < total; ++i) {
        this->glo.vertices[i].normal = glm::normalize(this->glo.vertices[i].normal);
    }
}