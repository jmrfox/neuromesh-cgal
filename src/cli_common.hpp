#pragma once

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

namespace neuromesh {

inline void print_help(const char* program, std::string_view help_text) {
    std::cout << help_text;
    (void)program;
}

inline void print_error(const std::string& message) {
    std::cerr << "Error: " << message << std::endl;
}

inline bool require_flag_value(int& index, int argc, char** argv, const char* flag_name) {
    if (index + 1 >= argc) {
        print_error(std::string("Missing value for flag: ") + flag_name);
        return false;
    }
    ++index;
    return true;
}

inline bool is_help_flag(const std::string& arg) {
    return arg == "--help" || arg == "-h";
}

inline void reject_positional_args(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (!arg.empty() && arg[0] != '-') {
            print_error(
                "Positional arguments are not supported. Use --input, --output, "
                "--output-prefix, or --output-dir flags instead.");
            std::exit(1);
        }
    }
}

}  // namespace neuromesh
