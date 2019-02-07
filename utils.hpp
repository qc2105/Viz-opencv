#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <fstream>
//#include <unistd.h>
#include <string>
#include <vector>

using namespace std;

size_t split_str(const std::string &txt, vector<std::string> &strs, char ch);

size_t nlines(string &obj_file_path, string marker);

#endif