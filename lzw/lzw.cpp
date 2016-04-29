#include "lzw.h"
#include <iostream>
#include <bitset>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#define DICTIONARY_SIZE 4096

// Print vector, used for quick debugging
void pv(std::vector<unsigned char>v){
    for (int i=0;i<v.size();i++){
        std::cout<<v[i];
    }std::cout<<std::endl;
}

// Initialise the dictionary with the initial one-byte values
void initialiseDictionary(Dictionary &dictionary){
    for(uint i = 0; i <= 0xFF; i++){
        std::vector<unsigned char> string;
        string.push_back((unsigned char) i);
        dictionary.push_back(string);
    }
}

// Add a dictionary entry
void addToDictionary(Dictionary &dictionary, std::vector<unsigned char>&toAdd){
    dictionary.push_back(toAdd);
    if(dictionary.size() >= DICTIONARY_SIZE){
        dictionary.clear();
        initialiseDictionary(dictionary);
    }
}

// Search dictionary for an entry, return the index (code)
int searchDictionary(Dictionary &dictionary, std::vector<unsigned char>&toSearchFor){
    for(int i = 0; i < dictionary.size(); i++){
        if(dictionary[i] == toSearchFor){
            return i;
        }
    }
    return -1;
}

// Initialise the dictionary with the initial one-byte values
void initialiseFastDictionary(FastDictionary &dictionary){
    for(uint i = 0; i <= 0xFF; i++){
        std::vector<unsigned char> string;
        string.push_back((unsigned char)i);
        dictionary.insert(DictionaryEntry(string, i));
    }
}

// Add a dictionary entry
int addToFastDictionary(FastDictionary &dictionary, std::vector<unsigned char>&toAdd){
    dictionary.insert(DictionaryEntry(toAdd, dictionary.size()));
    if(dictionary.size() >= DICTIONARY_SIZE){
        dictionary.clear();
        initialiseFastDictionary(dictionary);
    }
}

// Search dictionary for an entry, return the index (code)
int searchFastDictionary(FastDictionary &dictionary, std::vector<unsigned char>&toSearchFor){
    try{
        return dictionary.at(toSearchFor);
    }catch(std::out_of_range exception){
        return -1;
    }
}



// Compress the given input, save the result in given output
int compress(std::string inputFileName, std::string outputFileName){

    // Open Files
    FILE* uncompressedFile;
    FILE* outputFile;

    uncompressedFile = fopen(inputFileName.c_str(), "rb");
    outputFile = fopen(outputFileName.c_str(), "wb");

    if(uncompressedFile == NULL){
        //std::cerr << "Cannot open input file: " << inputFileName << "\n";
        return INPUT_FILE_ERROR;
    } else if(outputFile == NULL){
        //std::cerr << "Cannot open output file: " << outputFileName << "\n";
        return OUTPUT_FILE_ERROR;
    }

    fseek(uncompressedFile, 0L, SEEK_END);
    long size = ftell(uncompressedFile);
    fseek(uncompressedFile, 0L, SEEK_SET);


    bool toggle = false;
    Dictionary dictionary;
    dictionary.reserve(DICTIONARY_SIZE);
    initialiseDictionary(dictionary);

    unsigned char nextChar; std::vector<unsigned char> currentInput(1);

    fread(&currentInput[0], 1, 1, uncompressedFile);
    int oldCode;
    while(ftell(uncompressedFile) < size ){

        // Buffer read in until we don't have code for W+w
        do {
            if(fread(&nextChar, 1, 1, uncompressedFile)){
                currentInput.push_back(nextChar);
            } else{ // Have got to end of file
                currentInput.push_back(0);
                break;
            }

        }
        while(searchDictionary(dictionary, currentInput) != -1);

        // When found, save W+w for later (or dictionary may be prematurely reset)
        std::vector<unsigned char> toAdd = currentInput;

        // Remove and keep w
        unsigned char toKeep;
        toKeep = currentInput.back();
        currentInput.pop_back();

        //Obtain code for W - could (should) be saved from the last but one search from the loop above
        int newCode = searchDictionary(dictionary, currentInput);


        currentInput.clear();
        currentInput.push_back(toKeep);

        // Add W+w to dictionary
        addToDictionary(dictionary, toAdd);

        // Output code for W
        // Write codes as a pair
        if(toggle){
            unsigned char twoCodes[3];
            twoCodes[0] = (oldCode & 0xFF0) >> 4;
            twoCodes[1] = (oldCode & 0x00F) << 4 | (newCode & 0xF00)>> 8;
            twoCodes[2] = newCode & 0xFF;

            fwrite(twoCodes, 1, 3, outputFile);
        } else if(ftell(uncompressedFile) >= size){
            // Have hit end of file.

            // If last char read was w (from W+w) rather than a recognised W
            // Need to add code for it. Single char so code guaranteed.
            if(toKeep != 0){
                oldCode = newCode;
                std::vector<unsigned char> quick;
                quick.push_back(toKeep);
                newCode = searchDictionary(dictionary, quick);

                unsigned char twoCodes[3];
                twoCodes[0] = (oldCode & 0xFF0) >> 4;
                twoCodes[1] = (oldCode & 0x00F) << 4 | (newCode & 0xF00)>> 8;
                twoCodes[2] = newCode & 0xFF;

                fwrite(twoCodes, 1, 3, outputFile);
            } else {
                unsigned char oneCode[2];
                oneCode[0] = (newCode & 0xF00) >> 8;
                oneCode[1] = newCode & 0xFF;
                fwrite(oneCode, 1, 2, outputFile);
            }
        }
        toggle = !toggle;
        oldCode = newCode;
    }

    fclose(uncompressedFile);
    fclose(outputFile);

    //std::cout << "Successfully compressed." << std::endl;
    return 0;
}

// Decompress the given input, save the result in given output
int decompress(std::string inputFileName, std::string outputFileName) {

    // Open Files
    FILE* compressedFile;
    FILE* outputFile;

    compressedFile = fopen(inputFileName.c_str(), "rb");
    outputFile = fopen(outputFileName.c_str(), "wb");

    if(compressedFile == NULL){
        //std::cerr << "Cannot open input file: " << inputFileName << "\n";
        return INPUT_FILE_ERROR;
    } else if(outputFile == NULL){
        //std::cerr << "Cannot open output file: " << outputFileName << "\n";
        return OUTPUT_FILE_ERROR;
    }

    // Obtain number of codes - only needed if there is an odd number of codes
    fseek(compressedFile, 0L, SEEK_END);
    long size = ftell(compressedFile);
    fseek(compressedFile, 0L, SEEK_SET);

    // Obtaining number of codes at start is easier way to manage reading
    long lastCode;
    if(size%3){
        lastCode = (size-2)/3*2;
    }else{
        lastCode = -1;
    }

    unsigned char firstByte, secondByte; uint code;
    bool toggle = true; bool ended = false;

    std::vector<unsigned char> conjecture, decodedSequence;

    Dictionary dictionary;
    dictionary.reserve(DICTIONARY_SIZE);
    initialiseDictionary(dictionary);

    long codeCounter = 0;
    while(fread(&firstByte,1,1,compressedFile) & !ended ){
        // Toggle represents if the code starts at start/middle of a byte
        if(toggle) {
            fread(&secondByte,1,1,compressedFile);
            if(codeCounter == lastCode){
                code = firstByte << 8 | secondByte;
                ended = true;
            } else {
                code = firstByte << 4 | secondByte >> 4;
            }
        } else {
            unsigned char tmp = firstByte;
            firstByte = secondByte;
            secondByte = tmp;
            code = (firstByte & 0x0F) << 8 |  secondByte;

        }


        try {
            decodedSequence = dictionary.at(code);
            if(codeCounter != 0) {
                conjecture.push_back(decodedSequence[0]);
                addToDictionary(dictionary, conjecture);
            }
        }catch(std::out_of_range exception){
            decodedSequence = conjecture;
            decodedSequence.push_back(conjecture[0]);
            addToDictionary(dictionary, decodedSequence);
        }


        conjecture = decodedSequence;

        fwrite(&decodedSequence[0],decodedSequence.size(),1,outputFile);

        toggle = !toggle;
        codeCounter++;
    }
    fclose(compressedFile);
    fclose(outputFile);

    return 0;
}

// Compress the given input, save the result in given output
int fastCompress(std::string inputFileName, std::string outputFileName){

    // Open Files
    FILE* uncompressedFile;
    FILE* outputFile;

    uncompressedFile = fopen(inputFileName.c_str(), "rb");
    outputFile = fopen(outputFileName.c_str(), "wb");

    if(uncompressedFile == NULL){
        //std::cerr << "Cannot open input file: " << inputFileName << "\n";
        return INPUT_FILE_ERROR;
    } else if(outputFile == NULL){
        //std::cerr << "Cannot open output file: " << outputFileName << "\n";
        return OUTPUT_FILE_ERROR;
    }

    fseek(uncompressedFile, 0L, SEEK_END);
    long size = ftell(uncompressedFile);
    fseek(uncompressedFile, 0L, SEEK_SET);


    bool toggle = false;
    FastDictionary dictionary;
    initialiseFastDictionary(dictionary);

    unsigned char nextChar; std::vector<unsigned char> currentInput(1);

    fread(&currentInput[0], 1, 1, uncompressedFile);
    int oldCode;
    while(ftell(uncompressedFile) < size ){

        // Buffer read in until we don't have code for W+w
        do {
            if(fread(&nextChar, 1, 1, uncompressedFile)){
                currentInput.push_back(nextChar);
            } else{ // Have got to end of file
                currentInput.push_back(0);
                break;
            }

        }
        while(searchFastDictionary(dictionary, currentInput) != -1);

        // When found, save W+w for later (or dictionary may be prematurely reset)
        std::vector<unsigned char> toAdd = currentInput;

        // Remove and keep w
        unsigned char toKeep;
        toKeep = currentInput.back();
        currentInput.pop_back();

        //Obtain code for W - could (should) be saved from the last but one search from the loop above
        int newCode = searchFastDictionary(dictionary, currentInput);


        currentInput.clear();
        currentInput.push_back(toKeep);

        // Add W+w to dictionary
        addToFastDictionary(dictionary, toAdd);

        // Output code for W
        // Write codes as a pair
        if(toggle){
            unsigned char twoCodes[3];
            twoCodes[0] = (oldCode & 0xFF0) >> 4;
            twoCodes[1] = (oldCode & 0x00F) << 4 | (newCode & 0xF00)>> 8;
            twoCodes[2] = newCode & 0xFF;

            fwrite(twoCodes, 1, 3, outputFile);
        } else if(ftell(uncompressedFile) >= size){
            // Have hit end of file.

            // If last char read was w (from W+w) rather than a recognised W
            // Need to add code for it. Single char so code guaranteed.
            if(toKeep != 0){
                oldCode = newCode;
                std::vector<unsigned char> quick;
                quick.push_back(toKeep);
                newCode = searchFastDictionary(dictionary, quick);

                unsigned char twoCodes[3];
                twoCodes[0] = (oldCode & 0xFF0) >> 4;
                twoCodes[1] = (oldCode & 0x00F) << 4 | (newCode & 0xF00)>> 8;
                twoCodes[2] = newCode & 0xFF;

                fwrite(twoCodes, 1, 3, outputFile);
            } else {
                unsigned char oneCode[2];
                oneCode[0] = (newCode & 0xF00) >> 8;
                oneCode[1] = newCode & 0xFF;
                fwrite(oneCode, 1, 2, outputFile);
            }
        }
        toggle = !toggle;
        oldCode = newCode;
    }

    fclose(uncompressedFile);
    fclose(outputFile);

    //std::cout << "Successfully compressed." << std::endl;
    return 0;
}
