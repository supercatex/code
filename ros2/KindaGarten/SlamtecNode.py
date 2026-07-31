from KindaGarten.basic import *


class SlamtecNode(object):
    def __init__(self, ip="192.168.11.1", port=1448):
        self.ip = ip
        self.port = port
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
    
    def move_to(self, x, y, yaw=None, mode=2, precise=False, fail_retry_count=None, find_path_ignoring_dynamic_obstacles=False, with_directed_virtual_track=False):
        '''
        mode:
            0: 自由導航
            1: 嚴格軌道
            2: 軌道優先
        yaw == None 時，到目的地時隨意朝向
        precise: 精確到點模式
        fail_retry_count: 搜路失敗後重試次數
        find_path_ignoring_dynamic_obstacles: 搜路時忽略動態障礙物
        with_directed_virtual_track: 當 mode 1/2 時，按軌道方向移動
        '''
        flags = []
        if yaw is None:
            yaw = 0
        else: 
            flags.append("with_yaw")
        if precise: flags.append("precise")
        if fail_retry_count is None:
            fail_retry_count = 0
        else:
            flags.append("fail_retry_count")
        if find_path_ignoring_dynamic_obstacles: flags.append("find_path_ignoring_dynamic_obstacles")
        if with_directed_virtual_track: flags.append("with_directed_virtual_track")

        return self.send_request("/api/core/motion/v1/actions", "POST", {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": {
                "target": {
                    "x": x,
                    "y": y,
                    "z": 0
                },
                "move_options": {
                    "mode": mode,
                    "flags": flags,
                    "yaw": yaw,
                    "acceptable_precision": 0,
                    "fail_retry_count": fail_retry_count,
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
