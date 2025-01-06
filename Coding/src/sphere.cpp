#include <sphere.h>
#include <string>
#include <cstdio>
#include <fstream>
#include <vector>



std::string getPath(const std::string& target, int depth) {
	std::string path = target;
	for (int i = 0; i < depth; ++i) {
		FILE* file = fopen(path.c_str(), "r");
		if (file != nullptr) {
			fclose(file);
			return path;
		}
		path = "../" + path;
	}
	return target;
}
Sphere::Sphere(const glm::vec3& center, float radius) {
    generateSphereObjFile(center,radius);
	Initial();
}

void Sphere::Initial() {
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, this->vertices_.size() * sizeof(Vertex), &this->vertices_[0], GL_STATIC_DRAW);
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * this->indices_.size(), &this->indices_[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
        (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
        (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

}

void Sphere::generateSphereObjFile(const glm::vec3& center, float radius) {
    const unsigned int rings = 30;  
    const unsigned int sectors = 30; 
    const float PI = 3.14159265359f;
    std::vector<vec3> vertices_po;
    std::vector<vec3> vertices_no;
    for (unsigned int r = 0; r <= rings; ++r) {
        float phi = PI * r / rings;
        for (unsigned int s = 0; s <= sectors; ++s) {
            float theta = 2.0f * PI * s / sectors;

            Vertex vertex;
            vec3 new_po;
            new_po.x = radius * sinf(phi) * cosf(theta) + center.x;
            new_po.y = radius * cosf(phi) + center.y;
            new_po.z = radius * sinf(phi) * sinf(theta) + center.z;

            glm::vec3 normal = glm::normalize(new_po - center);
            vertices_no.push_back(normal);

            vertices_po.push_back(new_po);
        }
    }
    for (unsigned int i = 0; i < (rings-1); ++i) {
        for (unsigned int j = 0; j < sectors; ++j) {
            if (j != sectors - 1) {
                unsigned int first = i * sectors + j;
                unsigned int second = first + sectors;

                this->indices_.push_back(first);
                this->indices_.push_back(first);
                this->indices_.push_back(second);
                this->indices_.push_back(second);
                this->indices_.push_back(second + 1);
                this->indices_.push_back(second + 1);

                this->indices_.push_back(first);
                this->indices_.push_back(first);
                this->indices_.push_back(first + 1);
                this->indices_.push_back(first + 1);
                this->indices_.push_back(second + 1);
                this->indices_.push_back(second + 1);
            }
            else {
                unsigned int first = i * sectors + j;
                unsigned int second = first + sectors;

                this->indices_.push_back(first);
                this->indices_.push_back(first);
                this->indices_.push_back(second);
                this->indices_.push_back(second);
                this->indices_.push_back(first + 1);
                this->indices_.push_back(second + 1);

                this->indices_.push_back(first);
                this->indices_.push_back(first);
                this->indices_.push_back(first + 1);
                this->indices_.push_back(first + 1);
                this->indices_.push_back(first+1-sectors);
                this->indices_.push_back(first + 1 - sectors);
            }

        }
    }
    for (size_t i = 0; i < this->indices_.size(); i += 2) {
        Vertex new_vertice;
        new_vertice.position = vertices_po[this->indices_[i]];
        new_vertice.normal = vertices_no[this->indices_[i+1]];
        this->vertices_.push_back(new_vertice);
    }
}

void Sphere::loadDataFromFile(const std::string& path) {
    std::ifstream infile(path);
    if (!infile) {
        printf("Failure");
        return;
    }
    std::string line;
    std::vector<vec3> vertices_po;
    std::vector<vec3> vertices_no;
    unsigned int ver_co = 0;
    unsigned int no_co = 0;
    unsigned int fa_co = 0;
    int judge = 0;
    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        }
        else if (ver_co != 0 || no_co != 0) {
            if (ver_co != 0) {
                vec3 new_po;
                const char* cstr = line.c_str();
                sscanf_s(cstr, "%f %f %fn", &new_po.x, &new_po.y, &new_po.z);
                vertices_po.push_back(new_po);
                ver_co -= 1;
            }
            else {
                vec3 new_no;
                const char* cstr = line.c_str();
                sscanf_s(cstr, "%f %f %fn", &new_no.x, &new_no.y, &new_no.z);
                vertices_no.push_back(new_no);
                judge = 1;
                no_co -= 1;
            }
        }
        else {
            if (judge == 0) {
                const char* cstr = line.c_str();
                sscanf_s(cstr, "%d %d %dn", &ver_co, &no_co, &fa_co);
            }
            else {
                GLuint vi_1, vi_2, vi_3, ni_1, ni_2, ni_3;
                const char* cstr = line.c_str();
                sscanf_s(cstr, "%d %d %d %d %d %dn", &vi_1, &ni_1, &vi_2, &ni_2, &vi_3,
                    &ni_3);
                this->indices_.push_back(vi_1);
                this->indices_.push_back(ni_1);
                this->indices_.push_back(vi_2);
                this->indices_.push_back(ni_2);
                this->indices_.push_back(vi_3);
                this->indices_.push_back(ni_3);
            }
        }
    }
    for (size_t i = 0; i < this->indices_.size(); i += 2) {
        Vertex new_vertice;
        new_vertice.position = vertices_po[this->indices_[i]];
        new_vertice.normal = vertices_no[this->indices_[i + 1]];
        this->vertices_.push_back(new_vertice);
    }
    infile.close();
}

void Sphere::draw() const {
    glBindVertexArray(this->VAO);
    glDrawArrays(GL_TRIANGLES, 0, this->vertices_.size());
    glBindVertexArray(0);
}
