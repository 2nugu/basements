#pragma once
#include <string>
#include <vector>
#include <variant>

namespace basements {
namespace editor {

struct Property {
    enum class Type { FLOAT, INT, BOOL, VEC3, STRING };
    std::string name;
    Type type;
    std::variant<float, int, bool, std::string> value;
};

class Component {
public:
    virtual ~Component() = default;
    virtual std::string type_name() const = 0;
    virtual std::vector<Property> get_properties() const { return {}; }
    virtual void set_property(const std::string& /*name*/, const Property& /*prop*/) {}
};

} // namespace editor
} // namespace basements
