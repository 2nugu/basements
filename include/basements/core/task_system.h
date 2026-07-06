#ifndef BASEMENTS_CORE_TASK_SYSTEM_H
#define BASEMENTS_CORE_TASK_SYSTEM_H

#include <string>
#include <functional>
#include <future>
#include <vector>
#include <mutex>

namespace basements {
namespace core {

/**
 * @brief Represents a single asynchronous task
 */
class Task {
public:
    enum class Status { PENDING, RUNNING, COMPLETED, FAILED };

    Task(const std::string& name, std::function<void()> work)
        : name_(name), work_(work), status_(Status::PENDING) {}

    void execute() {
        status_ = Status::RUNNING;
        try {
            work_();
            status_ = Status::COMPLETED;
        } catch (...) {
            status_ = Status::FAILED;
        }
    }

    const std::string& name() const { return name_; }
    Status status() const { return status_; }
    float progress() const { return progress_; }
    void set_progress(float p) { progress_ = p; }

private:
    std::string name_;
    std::function<void()> work_;
    Status status_;
    float progress_ = 0.0f;
};

/**
 * @brief Simple task queue for background work
 */
class TaskSystem {
public:
    static TaskSystem& instance();

    void enqueue(std::unique_ptr<Task> task);
    void update(); // Check for completed tasks

    const std::vector<std::shared_ptr<Task>>& get_active_tasks() const { return active_tasks_; }

private:
    TaskSystem() = default;
    std::vector<std::shared_ptr<Task>> active_tasks_;
    std::mutex mutex_;
};

} // namespace core
} // namespace basements

#endif // BASEMENTS_CORE_TASK_SYSTEM_H
