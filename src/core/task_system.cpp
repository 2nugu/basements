#include "basements/core/task_system.h"
#include <iostream>
#include <thread>

namespace basements {
namespace core {

TaskSystem& TaskSystem::instance() {
    static TaskSystem instance;
    return instance;
}

void TaskSystem::enqueue(std::unique_ptr<Task> task) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto task_shared = std::shared_ptr<Task>(std::move(task));
    active_tasks_.push_back(task_shared);
    
    // Launch background thread for this task
    std::thread([task_shared]() {
        std::cout << "[Task] Starting: " << task_shared->name() << std::endl;
        task_shared->execute();
        std::cout << "[Task] Finished: " << task_shared->name() << std::endl;
    }).detach();
}

void TaskSystem::update() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Remove completed or failed tasks from the monitoring list
    active_tasks_.erase(
        std::remove_if(active_tasks_.begin(), active_tasks_.end(),
            [](const std::shared_ptr<Task>& t) {
                return t->status() == Task::Status::COMPLETED || t->status() == Task::Status::FAILED;
            }),
        active_tasks_.end()
    );
}

} // namespace core
} // namespace basements
