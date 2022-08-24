#include <ros_exercises/print_array.h>

template<class T>
std::string print_array::to_string(const std::vector<T>  &vec) {

    std::stringstream output;
    size_t size = vec.size();
    output << "request: array = [";
    for (int i = 0; i < size; i++) {
        output << std::to_string(vec[i]);
        if (i < (size - 1)) output << ", ";
    }
    output << "]";
    return output.str();

}

std::string print_array::to_string(const ros_exercises::find_min_array::Request  &req) {

    return to_string(req.array);

}

std::string print_array::to_string(const ros_exercises::compute_statistics::Request  &req) {

    return to_string(req.input);

}

std::string print_array::to_string(const ros_exercises::primes_list  &req) {

    return to_string(req.data);

}
