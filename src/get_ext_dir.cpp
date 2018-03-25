#include "get_ext_dir.h"

void get_ext_dir(const ::boost::filesystem::path& root, const std::string& ext, std::vector<::boost::filesystem::path>& ret)
{
    if(!::boost::filesystem::exists(root) || !::boost::filesystem::is_directory(root)) return;
    ::boost::filesystem::recursive_directory_iterator it(root);
    ::boost::filesystem::recursive_directory_iterator endit;
    while(it != endit)
    {
        if(::boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
        ++it;
    }
    std::sort(ret.begin(), ret.end());
}