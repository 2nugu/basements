#include "basements/graphics/framebuffer.h"
#include <iostream>
#include "basements/graphics/gl_loader.h"

namespace basements {
namespace editor {

Framebuffer::Framebuffer(const FramebufferSpecification& spec) : spec_(spec) {
    invalidate();
}

Framebuffer::~Framebuffer() {
    if (renderer_id_) {
        glDeleteFramebuffers(1, &renderer_id_);
        glDeleteTextures(1, &color_attachment_);
        glDeleteTextures(1, &depth_attachment_);
    }
}

void Framebuffer::invalidate() {
    if (renderer_id_) {
        glDeleteFramebuffers(1, &renderer_id_);
        glDeleteTextures(1, &color_attachment_);
        glDeleteTextures(1, &depth_attachment_);
    }

    glGenFramebuffers(1, &renderer_id_);
    glBindFramebuffer(GL_FRAMEBUFFER, renderer_id_);

    // Color Attachment
    glGenTextures(1, &color_attachment_);
    glBindTexture(GL_TEXTURE_2D, color_attachment_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, spec_.width, spec_.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_attachment_, 0);

    // Depth Attachment
    glGenTextures(1, &depth_attachment_);
    glBindTexture(GL_TEXTURE_2D, depth_attachment_);
    // Use GL_DEPTH24_STENCIL8 for depth and stencil
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, spec_.width, spec_.height, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, nullptr);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, depth_attachment_, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Framebuffer is incomplete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::resize(int width, int height) {
    if (width <= 0 || height <= 0 || (width == spec_.width && height == spec_.height))
        return;

    spec_.width = width;
    spec_.height = height;
    invalidate();
}

void Framebuffer::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, renderer_id_);
    glViewport(0, 0, spec_.width, spec_.height);
}

void Framebuffer::unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace editor
} // namespace basements
