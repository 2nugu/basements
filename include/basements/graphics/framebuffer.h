#ifndef BASEMENTS_EDITOR_FRAMEBUFFER_H
#define BASEMENTS_EDITOR_FRAMEBUFFER_H

namespace basements {
namespace editor {

struct FramebufferSpecification {
    int width = 0;
    int height = 0;
    // We can add multisampling, formats later
};

class Framebuffer {
public:
    Framebuffer(const FramebufferSpecification& spec);
    ~Framebuffer();

    void resize(int width, int height);
    void bind();
    void unbind();

    unsigned int get_color_attachment_id() const { return color_attachment_; }
    const FramebufferSpecification& get_specification() const { return spec_; }

private:
    void invalidate();

    unsigned int renderer_id_ = 0;
    unsigned int color_attachment_ = 0;
    unsigned int depth_attachment_ = 0;
    FramebufferSpecification spec_;
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_FRAMEBUFFER_H
