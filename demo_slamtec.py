import requests
import json 
import time 


class SlamtecNode(object):
    def __init__(self):
        self.ip = "192.168.11.1"
        self.port = 1448
        self.headers = {
            "accept": "application/json",
            "Content-Type": "application/json"
        }

    def send_request(self, api, method="GET", json_data={}):
        try:
            url = f"http://{self.ip}:{self.port}{api}"
            response = requests.request(
                method=method,
                url=url, 
                headers=self.headers,
                json=json_data
            )
            response.raise_for_status()
            return response.json()
        except Exception as e:
            print(e)
            return None
        
    def get_power_status(self):
        return self.send_request("/api/core/system/v1/power/status")
    
    def get_network_status(self):
        return self.send_request("/api/core/system/v1/network/status")

    def get_network_route(self):
        return self.send_request("/api/core/system/v1/network/route")
    
    def get_network_apn(self):
        return self.send_request("/api/core/system/v1/network/apn")
    
    def get_localization_pose(self):
        return self.send_request("/api/core/slam/v1/localization/pose")
    
    def get_localization_quality(self):
        return self.send_request("/api/core/slam/v1/localization/quality")
    
    def get_localization_odopose(self):
        return self.send_request("/api/core/slam/v1/localization/odopose")
        
    def get_localization_enable(self):
        return self.send_request("/api/core/slam/v1/localization/:enable")
    
    def get_mapping_enable(self):
        return self.send_request("/api/core/slam/v1/mapping/:enable")
    
    def get_loopclosure_enable(self):
        return self.send_request("/api/core/slam/v1/loopclosure/:enable")
    
    def get_point_of_interest(self):
        return self.send_request("/api/core/artifact/v1/pois")
    
    def get_action_factories(self):
        return self.send_request("/api/core/motion/v1/action-factories")
    
    def get_current_action(self):
        return self.send_request("/api/core/motion/v1/actions/:current")
    
    def move_to(self, x, y, yaw=0.0):
        return self.send_request("/api/core/motion/v1/actions", "POST", {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": {
                "target": {
                    "x": x,
                    "y": y,
                    "z": 0
                },
                "move_options": {
                    "mode": 0,
                    "flags": ["with_yaw"],
                    "yaw": yaw,
                    "acceptable_precision": 0,
                    "fail_retry_count": 0,
                    "speed_ratio": 0
                }
            }
        })
    
    def get_action_status(self, action_id):
        return self.send_request(f"/api/core/motion/v1/actions/{action_id}")
    
    def stop_current_action(self):
        return self.send_request("/api/core/motion/v1/actions/:current", "DELETE")
    
    def move_to_home(self):
        return self.send_request("/api/core/motion/v1/actions", "POST", {
            "action_name": "slamtec.agent.actions.GoHomeAction",
            "gohome_options": {
                "flags": "dock",
                "back_to_landing": True,
                "charging_retry_coount": 3
            }
        })
    
    def get_poi_by_name(self, name):
        res = self.get_point_of_interest()
        for obj in res:
            if obj["metadata"]["display_name"] == name:
                return obj["pose"]["x"], obj["pose"]["y"], obj["pose"]["yaw"]
        return None 

    def set_localization_pose(self, x, y, yaw):
        return self.send_request("/api/core/slam/v1/localization/pose", "PUT", {
            "x": x,
            "y": y,
            "z": 0,
            "yaw": yaw,
            "pitch": 0,
            "roll": 0
        })

    def set_localization_enable(self, enable):
        return self.send_request("/api/core/slam/v1/localization/:enable", "PUT", {
            "enable": enable
        })

    def set_mapping_enable(self, enable):
        return self.send_request("/api/core/slam/v1/mapping/:enable", "PUT", {
            "enable": enable
        })

    def set_loopclosure_enable(self, enable):
        return self.send_request("/api/core/slam/v1/loopclosure/:enable", "PUT", {
            "enable": enable
        })

    def reset_localization_status(self):
        return self.send_request("/api/core/slam/v1/localization/status/:reset", "POST")

    def set_stcm_map(self, filepath):
        with open(filepath, "rb") as f:
            try:
                url = f"http://{self.ip}:{self.port}/api/core/slam/v1/maps/stcm"
                response = requests.request(
                    method="PUT",
                    url=url, 
                    headers={
                        "accept": "*/*",
                        "Content-Type": "application/octet-stream"
                    },
                    data=f
                )
                response.raise_for_status()
                return True
            except Exception as e:
                print(e)
                return False
        return False

    def clear_stcm_map(self):
        return self.send_request("/api/core/slam/v1/maps", "DELETE")
    

if __name__ == "__main__":
    node = SlamtecNode()

    print(node.get_power_status())
    # print(node.clear_stcm_map())
    # print(node.get_localization_pose())

    # node.move_to_home()
    print(node.set_stcm_map("/home/ubuntu/workspace/map_001.stcm"))
    print(node.set_localization_pose(-2.74, 0.39, -0.05))
    # print(node.reset_localization_status())
    time.sleep(100)

    state = 0
    action = None
    while True:
        time.sleep(0.5)
        if state == 0:
            p1 = node.get_poi_by_name("P1")
            action = node.move_to(*p1)
            state = 1
        elif state == 1:
            action_state = node.get_action_status(action["action_id"])
            print(action_state)
            if action_state["state"]["status"] == 4:
                state = 2
        elif state == 2:
            p2 = node.get_poi_by_name("P2")
            action = node.move_to(*p2)
            state = 3
        elif state == 3:
            action_state = node.get_action_status(action["action_id"])
            print(action_state)
            if action_state["state"]["status"] == 4:
                state = 0
