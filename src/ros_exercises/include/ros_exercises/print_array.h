#include <string>
#include <ros_exercises/find_min_array.h>
#include <ros_exercises/compute_statistics.h>
#include <ros_exercises/primes_list.h>

namespace print_array {

    template<class T>
    std::string to_string(const std::vector<T>  &vec);

    std::string to_string(const ros_exercises::find_min_array::Request  &req);

    std::string to_string(const ros_exercises::compute_statistics::Request  &req);

    std::string to_string(const ros_exercises::primes_list  &req);

}
