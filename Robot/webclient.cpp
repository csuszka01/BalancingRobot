#ifndef WEBCLIENT_CPP
#define WEBCLIENT_CPP

#include <curl/curl.h>
#include <string>
#include <mutex>

std::mutex mtx;

struct WebClient {
    std::string serveraddr;
    CURL *curl = nullptr;
    CURLcode res;
    std::string readBuffer;
    struct curl_slist *headers = nullptr;

    WebClient() {}
    
    WebClient(std::string serveraddr) : serveraddr(serveraddr) {
        curl = curl_easy_init();
        if(curl) {
            curl_easy_setopt(curl, CURLOPT_URL, serveraddr.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            // --- KEEP-ALIVE & LATENCY OPTIMIZATIONS ---
            headers = curl_slist_append(headers, "Content-Type: application/json");
            headers = curl_slist_append(headers, "Connection: keep-alive");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            // Disable Nagle's algorithm (TCP delay) for high-frequency PID ticks
            curl_easy_setopt(curl, CURLOPT_TCP_NODELAY, 1L);
            
            // Enable TCP keep-alive probes
            curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);
        }
    }
    
    // Prevent copying the raw CURL pointer (fixes the destructor crash on rollback)
    WebClient(const WebClient& other) : serveraddr(other.serveraddr) {
        if (!other.serveraddr.empty()) {
            *this = WebClient(other.serveraddr);
        }
    }

    WebClient& operator=(const WebClient& other) {
        if (this != &other) {
            cleanup();
            serveraddr = other.serveraddr;
            if (!serveraddr.empty()) {
                *this = WebClient(serveraddr);
            }
        }
        return *this;
    }

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
    {
        mtx.lock();
        ((std::string*)userp)->append((char*)contents, size * nmemb);
        mtx.unlock();
        return size * nmemb;
    }
    
    void post(std::string data) {
        if (!curl) return;
        readBuffer.clear(); // Keep allocated capacity, just clear text
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, data.length());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
        res = curl_easy_perform(curl);
    }
    
    std::string getResponse() {
        return readBuffer;
    }
    
    void cleanup() {
        if (headers) {
            curl_slist_free_all(headers);
            headers = nullptr;
        }
        if (curl) {
            curl_easy_cleanup(curl);
            curl = nullptr;
        }
    }

    ~WebClient() {
        cleanup();
    }
};

#endif