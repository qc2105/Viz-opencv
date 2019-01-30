#include "utils.hpp"

size_t split_str(const std::string &txt, vector<std::string> &strs, char ch)
{
    size_t pos = txt.find(ch);
    size_t initialPos = 0;
    strs.clear();

    // Decompose statement
    while (pos != std::string::npos)
    {
        strs.push_back(txt.substr(initialPos, pos - initialPos));
        initialPos = pos + 1;
        while(txt[initialPos] == ch)
        {
            initialPos++;
        }
        pos = txt.find(ch, initialPos);
    }

    // Add the last one
    strs.push_back(txt.substr(initialPos, std::min(pos, txt.size()) - initialPos + 1));

    return strs.size();
}

size_t lines(string &obj_file_path, string marker)
{
    ifstream ifs(obj_file_path, ifstream::in);
    string str;
    size_t n = 0;
    while(ifs.good())
    {
        getline(ifs, str);
        if (string::npos == str.find(marker))
            continue;
        n++;
    }
    return n;
}