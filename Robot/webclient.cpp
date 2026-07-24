#ifndef WEBCLIENT_CPP
#define WEBCLIENT_CPP

#include <curl/curl.h>
#include <string>
#include <mutex>

std::mutex mtx;

struct WebClient {
    std::string serveraddr;
    CURL *curl;
    CURLcode res;
    std::string readBuffer;
    WebClient() {}
    
    WebClient(std::string serveraddr) : serveraddr(serveraddr) {
	curl = curl_easy_init();
	if(curl) {
	    curl_easy_setopt(curl, CURLOPT_URL, serveraddr.c_str());
	    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
	    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
	}
    }
    
    //WebClient(const &WebClient) {}
    
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
    {
	mtx.lock();
	((std::string*)userp)->append((char*)contents, size * nmemb);
	mtx.unlock();
	return size * nmemb;
    }
    
    void post(std::string data) {
	readBuffer.erase();
	//readBuffer = std::string(1000, ' ');
	curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, data.length());
	curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
	res = curl_easy_perform(curl);
	//std::cout<<"response code: "<<res<<std::endl;
	//curl_easy_cleanup(curl);
    }
    
    std::string getResponse() {
	return readBuffer;
    }
    
    ~WebClient() {
	curl_easy_cleanup(curl);
    }
};


/*int main() {
    WebClient wc = WebClient("http://localhost:5000/pid");
    wc.post("Hello");
    std::cout<<wc.getResponse()<<std::endl;
    return 0;
}*/
#endif
