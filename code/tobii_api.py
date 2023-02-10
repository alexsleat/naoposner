import requests, time, json

class TobiiAPI:

    def __init__(self, host, port=80):
        self.host = host
        self.port = port 
        self.base_api_url = "http://" + str(host) + "/rest"
    
    def get_api_list(self):
        print("Listing API: ")
        response = requests.get(self.base_api_url)
        print(response.json())

    def test_api(self):
        print("Testing API: ")
        req = "/recorder"
        action_name = "stop"
        body = []
        id = 1337
        
        ws_dict = {
            "path": f"{req}!{action_name}",
            "id": id,
            "method": "POST",
            "body": body,
        }
        ws_json = json.dumps(ws_dict)
        
        # sending get request and saving the response as response object
        response = requests.get(url=self.base_api_url + req, data = ws_json)

        try:
            print(response.json())
        except:
            print("Not printable response")
            try:
                print(response)
            except:
                print("Err")


    def start_recorder(self):
        print("Start Recording: ")
        req = "/recorder"
        body = []
        # response = requests.get(url=self.base_api_url + req, params=body)

        # defining a params dict for the parameters to be sent to the API
        PARAMS = {"path": req + "!start",
                  'body': []}
        
        # sending get request and saving the response as response object
        response = requests.get(url=self.base_api_url, params = PARAMS)

        try:
            print(response.json())
        except:
            print("Not printable response")
            try:
                print(response)
            except:
                print("Err")


    def stop_recorder(self):
        print("Stop Recording: ")
        req = "/recorder!stop"
        body = []
        response = requests.post(url=self.base_api_url + req, data=body)
        try:
            print(response.json())
        except:
            print("Not printable response")
            try:
                print(response)
            except:
                print("Err")

    def sync_time():
        print("Sync Time")
        
if __name__ == '__main__':
    ta = TobiiAPI("192.168.100.134")
    ta.test_api()