#include <iostream>
#include <iomanip>
#include <vector>
#include <bitset>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <string>
#include "lzw.h"

using namespace std;

void printUsage(){
    cerr << "Usage:\t./lzw\n";
    cerr << "\t<inputFile>\n";
    cerr << "\t-h : Display this usage guide.\n";
    cerr << "\t-c : Compress file.\n";
    cerr << "\t-d : Decompress file (default).\n";
    cerr << "\t-o [File Name] : File output name.\n";
    cerr << "\t--test : Test tool on provided sample files.\n";
}


int main(int argc, char* argv[]) {

    string inputFileName;
    string outputFileName = "output.txt";
    string mode = "decompress"; //decompress by default

    int ai = 1;
    if(argc < 2){
        cerr << "Please provide input file.\n";
        printUsage();
        std::exit(1);
    }
    while(ai<argc){
        if(!strcmp("-h", argv[ai])){
            printUsage();
            std::exit(1);
        } else if(!strcmp("-o", argv[ai])) {
            if (ai + 1 == argc) {
                cerr << "No output file name provided.\n";
                std::exit(1);
            }
            outputFileName = argv[ai + 1];
            ai += 2;
        } else if(!strcmp("-d", argv[ai])) {
            mode = "decompress";
            ai++;
        } else if (!strcmp("-c", argv[ai])) {
            mode = "compress";
            ai++;
        } else if (!strcmp("--test", argv[ai])){
            mode = "test";
            ai++;
        } else if('-' == argv[ai][0]){
            cerr << "Command not recognised.\n";
            std::exit(1);
        } else{
            inputFileName = argv[ai];
            ai+=1;
        }
    }


    if(mode == "decompress"){
        // Will return error code if (file) failure, no visual printing
        return decompress(inputFileName, outputFileName);
    } else if (mode == "compress"){
        return fastCompress(inputFileName, "output.z");
    } else if (mode == "test") {

        // Decompress all inputs
        cout << "\nDecompressing:" << endl;
        for (int i = 1; i <= 4; i++) {
            string input = "LzwInputData/compressedfile" + to_string(i) + ".z";
            string output = "LzwOutputData/decompressedfile" + to_string(i) + ".txt";
            int failure = decompress(input, output);
            if (failure) {
                cerr << "Error decompressing file " << i << "  Code:" << failure << endl;
            } else {
                cout << "\tCompleted " << i << endl;
            }
        }

        // Recompress
        cout << "\nRecompressing:" << endl;
        for (int i = 1; i <= 4; i++) {
            string input = "LzwOutputData/decompressedfile" + to_string(i) + ".txt";
            string output = "LzwOutputData/recompressedfile" + to_string(i) + ".z";
            int failure = fastCompress(input, output);
            if (failure) {
                cerr << "Error recompressing file " << i << "  Code:" << failure << endl;
            } else {
                cout << "\tCompleted " << i << endl;
            }
        }
    }

    return 0;


}
