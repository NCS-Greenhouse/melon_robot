#! /usr/bin/env python3
import json
import requests
import rospkg, configparser
from wisepaasdatahubedgesdk.EdgeAgent import EdgeAgent
import wisepaasdatahubedgesdk.Common.Constants as constant
from wisepaasdatahubedgesdk.Model.Edge import EdgeAgentOptions, DCCSOptions, EdgeData, EdgeTag

class datahub_api_send_get():
    def __init__(self, usr_name, password, node_id, device_id, tag_name, tag_type, mode, array_size, **kwargs):
        self.headers = {'Content-Type': 'application/json'}
        self.session = self.login(usr_name, password)
        self.node_id = node_id
        self.device_id = device_id[mode]
        self.tag_name = tag_name
        self.tag_type = tag_type
        self.array_size = array_size

    def login(self, usr_name, password):
        login_json = {"username": usr_name, "password": password}
        login_json = json.dumps(login_json)
        session = requests.Session()
        url_login = 'https://portal-datahub-greenhouse-eks005.education.wise-paas.com/api/v1/Auth'
        resp = session.post(url_login, headers=self.headers, data=login_json)
        return session

    def new_tagname(self):
        new_json = {
            "nodeId": self.node_id,
            "deviceId": self.device_id,
            "tagName": self.tag_name,
            "tagType": self.tag_type,
            "description": 'Plant ID' + self.tag_name.split('_')[-1],
            "readOnly": False,
            "arraySize": self.array_size
        }
        new_json = json.dumps(new_json)
        url_new = 'https://portal-datahub-greenhouse-eks005.education.wise-paas.com/api/v1/Tags'
        resp = self.session.post(url_new, headers=self.headers, data=new_json)

    def read_last_data(self, index):
        self.new_tagname()
        if index == -1:
            search_json = {"nodeId": self.node_id, "deviceId": self.device_id, "tagName": self.tag_name}
        else:
            search_json = {"nodeId": self.node_id, "deviceId": self.device_id, "tagName": self.tag_name, "index": int(index)}
        search_json = json.dumps(search_json)
        url_post = 'https://portal-datahub-greenhouse-eks005.education.wise-paas.com/api/v1/RealData/raw'
        resp = self.session.post(url_post, headers=self.headers, data=search_json)
        return json.loads(resp.content.decode('utf8'))[0]['value']

    def close_connection(self):
        self.session.close()

rospack = rospkg.RosPack()
conf = configparser.ConfigParser() 
package_root = rospack.get_path("robot_control_pkg") #Get current package absolute location
conf.read(package_root + "/package.conf") # Try to load this configuration file

hyp = {
    'node_id': 'b76616d3-8377-404b-a919-3fdc3daced0b', # our node id
    'device_id': {
        'joint': 'zBXno1abY2Q6',
        'image': 'xBZEuZ2sKVUT',
        'ugv_bridge': 'lRYVEc5R9bcv'
    },
    'usr_name': 'ntu.bioagri-4@wisepaas.com',
    'password': 'Bioagri@2022',
    'edge_type': 'Gateway',
    'connect_type': 'DCCS',
    'api_url': 'https://api-dccs-ensaas.education.wise-paas.com/',
    'credential_key': conf['forge.datahub']['credential_ugv']
}

markerID_get = datahub_api_send_get(
    usr_name=hyp['usr_name'],
    password=hyp['password'],
    node_id=hyp['node_id'],
    device_id=hyp['device_id'],
    tag_name="UGV_Target_Marker_ID_B",
    tag_type=1,
    array_size=1,
    mode='ugv_bridge'
)

ARMrequest_get = datahub_api_send_get(
    usr_name=hyp['usr_name'],
    password=hyp['password'],
    node_id=hyp['node_id'],
    device_id=hyp['device_id'],
    tag_name="Arm_Requests_B",
    tag_type=1,
    array_size=1,
    mode='ugv_bridge'
)
#for debugging
#print("markerID is %s" %(str(markerID_get.read_last_data(0))))
#print("ARMrequest is %d" %(ARMrequest_get.read_last_data(0)))
