#include "read_mat_from_file.h"

Eigen::MatrixXf read_mat_from_file(std::ifstream &fin, int start, int stop, int rows, int cols)
{
  // first line considered to be at index 0

  Eigen::MatrixXf T(rows,cols);
  int line_count = -1;
  float T_arr[rows*cols];
  std::string string_read;

  while(!fin.eof()) {
    line_count = line_count + 1;

    if (line_count < start) {
      std::getline(fin, string_read);
    }
    else if ( (line_count >= start) && (line_count < stop) ) {
      fin >> T_arr[line_count-start];
      // std::cout << T_arr[line_count+1-start] << std::endl;
    }
    else {
      break;
    }
  }

  for (int i = 0; i < rows; i++) { 
    for (int j = 0; j < cols; j++) {
      T(i,j) = T_arr[i*cols+j];
    } 
  }

  return T;
}