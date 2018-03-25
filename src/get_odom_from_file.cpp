#include "get_odom_from_file.h"

void get_odom_from_file(std::ifstream &fin, float odom_init[], float odom_final[])
{
    std::string str_1, str_2, str_buffer;
    int line_count = 0;

    while(!fin.eof() && line_count < 20){
    if(line_count == 16) {
      std::getline(fin, str_1);
      // std::cout << "line_count: " << line_count << " str_1: " << str_1  << std::endl;
    }
    else if (line_count == 18) {
      std::getline(fin, str_2);
      // std::cout << "line_count: " << line_count << " str_2: " << str_2 << std::endl;
    }
    else {
      std:getline(fin, str_buffer);
      // std::cout << "line_count: " << line_count << " str_buffer: " << str_buffer << std::endl; 
      // std::cout << "line_count: " << line_count << std::endl;
    }
    line_count++;
  }

  sscanf(str_1.c_str(), "%f %f %f %f %f %f", &odom_init[0], &odom_init[1], &odom_init[2], &odom_init[3], &odom_init[4], &odom_init[5]);
  sscanf(str_2.c_str(), "%f %f %f %f %f %f", &odom_final[0], &odom_final[1], &odom_final[2], &odom_final[3], &odom_final[4], &odom_final[5]);
}
