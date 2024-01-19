#pragma once
#include "StdAfx.h"
#include "OpenFBX/ofbx.h"
#include "SOIL/SOIL.h"

ofbx::IScene* cubeFBX = nullptr;
bool initOpenFBX(const char* filepath) {
    static char s_TimeString[256];
    FILE* fp = fopen(filepath, "rb");

    if (!fp) return false;

    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    auto* content = new ofbx::u8[file_size];
    fread(content, 1, file_size, fp);

	
    // Ignoring certain nodes will only stop them from being processed not tokenised (i.e. they will still be in the tree)
    ofbx::LoadFlags flags =
//		ofbx::LoadFlags::IGNORE_MODELS |
        ofbx::LoadFlags::IGNORE_BLEND_SHAPES |
        ofbx::LoadFlags::IGNORE_CAMERAS |
        ofbx::LoadFlags::IGNORE_LIGHTS |
//		ofbx::LoadFlags::IGNORE_TEXTURES |
        ofbx::LoadFlags::IGNORE_SKIN |
        ofbx::LoadFlags::IGNORE_BONES |
        ofbx::LoadFlags::IGNORE_PIVOTS |
//		ofbx::LoadFlags::IGNORE_MATERIALS |
        ofbx::LoadFlags::IGNORE_POSES |
        ofbx::LoadFlags::IGNORE_VIDEOS |
        ofbx::LoadFlags::IGNORE_LIMBS |
//		ofbx::LoadFlags::IGNORE_MESHES |
        ofbx::LoadFlags::IGNORE_ANIMATIONS;

    cubeFBX = ofbx::load((ofbx::u8*)content, file_size, (ofbx::u16)flags);


    if(!cubeFBX) {
        printf(ofbx::getError());
        //OutputDebugString(ofbx::getError());
    }
    delete[] content;
    fclose(fp);
    return true;
}
static void drawCube(Vector3* size) {
    glPushMatrix();
    glScalef(size->x,size->y,size->z);
    //FBX Format uses 4 indices for every square rather than 6. So we work accordingly
    const ofbx::Vec3* vertices = cubeFBX->getMesh(0)->getGeometry()->getGeometryData().getPositions().values;
    const GLint* indices = cubeFBX->getMesh(0)->getGeometry()->getGeometryData().getPositions().indices;
    glEnableClientState(GL_VERTEX_ARRAY);
    // Bind the vertex array
    glVertexPointer(3, GL_FLOAT, 0, vertices);
    // Loop through each face and render it using the corresponding indices
    for (int i = 0; i < 6; i++) {
        glDrawElements(GL_QUADS, 4, GL_UNSIGNED_INT, &indices[i * 4]);
        glShadeModel(GL_SMOOTH);
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    glPopMatrix();
}
static float Distance(Vector3* obj1, Vector3* obj2)
{
    return sqrt(pow(obj2->x - obj1->x,2) + pow(obj2->y - obj1->y,2) + pow(obj2->z - obj1->z,2));
}

static double Clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

static void LoadTexture(GLuint &texture_ID, const Texture& tex)
{
    glGenTextures(1, &texture_ID);
    glBindTexture(GL_TEXTURE_2D, texture_ID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // Set texture wrapping to repeat in both directions
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S    , GL_REPEAT );
    glTexImage2D(
    GL_TEXTURE_2D,  // Target
0,              // Level of detail (0 for base level)
GL_RGBA,        // Internal format (depends on your image format)
tex.width,          // Width of the texture
tex.height,         // Height of the texture
0,              // Border (always 0)
GL_RGBA,        // Format of the pixel data in your image (depends on your image format)
GL_UNSIGNED_BYTE, // Data type of the pixel data
tex.texture          // Pointer to the image data
);
}

