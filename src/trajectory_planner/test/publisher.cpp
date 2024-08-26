#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>

int main() {
    const char* pipeName = "/tmp/my_named_pipe";

    // Create the named pipe (FIFO)
    mkfifo(pipeName, 0666);

    // Open the named pipe for writing
    int pipeFd = open(pipeName, O_WRONLY);

    if (pipeFd == -1) {
        std::cerr << "Error opening the named pipe for writing." << std::endl;
        return 1;
    }

    // Data to send to the Python client
    int dataToSend = 0;

    while (true) {
        // Convert data to a string
        std::string dataStr = std::to_string((float)(rand()) / (float)(RAND_MAX) * 5) + ',' + std::to_string(rand() % 5) + '\n';

        // Write the data to the named pipe
        write(pipeFd, dataStr.c_str(), dataStr.size());

        printf("Written data: %s", dataStr.c_str());

        // Increment data for the next loop
        dataToSend++;

        // Sleep for a short duration before sending the next data
        usleep(100000); // in microsecond
    }

    // Close the pipe and remove it
    close(pipeFd);
    unlink(pipeName);

    return 0;
}

