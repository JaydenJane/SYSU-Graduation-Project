#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <string>

using namespace std;

const int MAX_FILE = 10;
const int fileNum = 2;

string fileName[MAX_FILE];
int main() {
    int fileCount = 2;
    fileName[0] = "d25cm.txt";
    fileName[1] = "d35cm.txt";

    ofstream fOut("data.txt");
    for(int i = 0; i < fileCount; i++){
        const char *theFile = fileName[i].c_str();
        ifstream fIn(theFile);
        string lines;
        while(getline(fIn, lines)){
            fOut << lines << endl;
        }
        fIn.close();
    }
    fOut.close();
    
    return 0;
}
