#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>
#include <arpa/inet.h>
#include <queue>
#include <mutex>

std::queue<std::string> dataBuffer;
std::mutex bufferMutex;

// Function to send data to Python server
void send_data_to_python(const char* serverAddress, int serverPort) {

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cerr << "Error creating socket." << std::endl;
        return;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);
    inet_pton(AF_INET, serverAddress, &serverAddr.sin_addr);

    if (connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error connecting to the server." << std::endl;
        close(sockfd);
        return;
    }

    std::string dataToSend = "0,0";

    while (true) {

        if(!dataBuffer.empty())
        {
          dataToSend = dataBuffer.front();//std::to_string((float)(rand()) / (float)(RAND_MAX) * 5) + ',' + std::to_string(rand() % 5);
          dataBuffer.pop();
        }

        send(sockfd, dataToSend.c_str(), dataToSend.size(), 0);

        char buffer[1024];
        int bytesRead = recv(sockfd, buffer, sizeof(buffer), 0);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            std::cout << "Received response from Python server: " << buffer << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(sockfd);
}

int main() {
    const char* serverAddress = "10.42.0.1"; // Replace this with the server's IP address
    const int serverPort = 12345; // Replace this with the server's port number

    // Run the client function in a separate thread
    std::thread clientThread(send_data_to_python, serverAddress, serverPort);

    int ii = 0;
    while(ii <= 200){
      {
        std::lock_guard<std::mutex> lock(bufferMutex);
        dataBuffer.push(std::to_string((float)(rand()) / (float)(RAND_MAX) * 5) + ',' + std::to_string(ii % 100));
      }
      ii++;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait for the thread to finish
    clientThread.join();

    return 0;
}

