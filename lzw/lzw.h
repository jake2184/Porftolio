
#ifndef LZW_H
#define LZW_H

#include<vector>
#include <string>
#include <map>

typedef std::vector<std::vector<unsigned char> > Dictionary;

typedef std::pair<std::vector<unsigned char>, short> DictionaryEntry;
typedef std::map<std::vector<unsigned char>, short> FastDictionary;


enum error {
    INPUT_FILE_ERROR = 1, OUTPUT_FILE_ERROR
};

// Compress given file, write to output file
int compress(std::string inputFileName, std::string outputFileName = "output.z");

// Compress given file, write to output file
int fastCompress(std::string inputFileName, std::string outputFileName = "output.z");

// Decompress given file, write to output file
int decompress(std::string inputFileName, std::string outputFileName = "output.txt");


#endif //LZW_H
