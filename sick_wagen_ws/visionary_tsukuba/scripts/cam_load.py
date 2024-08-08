import os
import sys
import time
import logging

import cv2
import torch
import platform
import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData, PlyElement
 
import harvesters
from harvesters.core import Harvester
from genicam.gentl import DEVICE_ACCESS_STATUS_LIST

import calc_cord

def validate_setup():
  MIN_PYTHON_VER = (3, 6)
  if sys.version_info < MIN_PYTHON_VER:
    sys.exit("Minimal required Python version for this script is {}.{}, your version is {}".format(*MIN_PYTHON_VER, sys.version))
  HARVESTERS_VERS = ['1.4.0', '0.post.dev4']
  if not harvesters.__version__ in HARVESTERS_VERS:
    sys.exit("Exact required Harvesters version for this script is {}, your version is {}".format(HARVESTERS_VERS, harvesters.__version__))

def force_suitable_ip_address(device_info):
  print("Attempting to force suitable IP address into the (currently unreachable) device {}".format(device_info.display_name))
  itf_node_map = device_info.parent.node_map

  device_id = device_info.id_
  for i in range(itf_node_map.DeviceSelector.max + 1):
    itf_node_map.DeviceSelector.value = i

    if device_id == itf_node_map.DeviceID.value:

      itf_node_map.GevDeviceProposeIP.execute()

      proposed_ip = itf_node_map.GevDeviceForceIPAddress.to_string()
      proposed_subnet = itf_node_map.GevDeviceForceSubnetMask.to_string()
      print("\tThe proposed IP address is {}, subnet mask {}, forcing it into the device".format(proposed_ip, proposed_subnet))

      itf_node_map.GevDeviceForceIP.execute()

      while not itf_node_map.GevDeviceForceIP.is_done():
        time.sleep(1)
      print("\tThe force-ip procedure completed")

def get_cti_dir_name():
  if platform.system() == "Windows":
    return "windows_x64"
  if platform.system() == "Linux" and platform.machine() == "x86_64":
    return "linux_x64"
  if platform.system() == "Linux" and platform.machine() == "aarch64":
    return "linux_aarch64"

  sys.exit("GenTL Producer not available on this platform")

def get_cti_path():
  cti_platform_dir_name = get_cti_dir_name()
  script_dir = os.path.dirname(os.path.realpath(__file__))
  CTI_FILENAME = "SICKGigEVisionTL.cti"
  return os.path.join(script_dir, "lib", "cti", cti_platform_dir_name, CTI_FILENAME)

def compute_bbox_distance_estimate(bbox_depth_arr):
    # bbox_depth_arr:深度情報(バウンディングボックス内)
    bbox_height, bbox_width = bbox_depth_arr.shape
    bbox_cropped_xcenter, bbox_cropped_ycenter = bbox_width / 2, bbox_height / 2

    x, y = np.meshgrid(np.arange(bbox_width), np.arange(bbox_height))

    GAUSSIAN_SPREAD_SCALE = 6.0
    GAUSSIAN_YSCALE_DISPLACEMENT_FACTOR = 1.25

    sigma_x = bbox_width / GAUSSIAN_SPREAD_SCALE  
    sigma_y = bbox_height / GAUSSIAN_SPREAD_SCALE 

    # 重みは2次元ガウス分布に従う
    weights = (1.0 / (2 * np.pi * sigma_x * sigma_y) * np.exp(-0.5 * ((x - bbox_cropped_xcenter)**2 / sigma_x**2 + (y - GAUSSIAN_YSCALE_DISPLACEMENT_FACTOR * bbox_cropped_ycenter)**2 / sigma_y**2)))

    depth_invalid_mask = bbox_depth_arr < 0.1
    weights = np.ma.masked_array(weights, mask=depth_invalid_mask)
    weights = weights / np.sum(weights) #正規化?

    distance_estimate_mill = np.sum(weights * bbox_depth_arr) #深度情報に重み付けしたものを距離としている
    return distance_estimate_mill

font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1.0
text_thickness = 2
thickness = 2

logging.basicConfig(level=logging.INFO)
validate_setup()

with Harvester() as h:
    cti = get_cti_path()

    h.add_file(cti, check_existence=True, check_validity=True)

    h.update()

    if len(h.device_info_list) == 0:
        sys.exit("No GigE Vision devices discovered in this system")
    for device_info in h.device_info_list:
        print("\t{}".format(device_info.display_name))

    if h.device_info_list[0].access_status == DEVICE_ACCESS_STATUS_LIST.DEVICE_ACCESS_STATUS_NOACCESS:
        force_suitable_ip_address(h.device_info_list[0])
        h.update()

    if len(h.device_info_list) == 0 or h.device_info_list[0].access_status != DEVICE_ACCESS_STATUS_LIST.DEVICE_ACCESS_STATUS_READWRITE:
        sys.exit("The discovered camera is not ready to open")

    with h.create() as ia:
        nm = ia.remote_device.node_map
        nm.ComponentSelector.value = 'Range'
        nm.ComponentEnable.value = True
        nm.ComponentSelector.value = 'Intensity'
        nm.ComponentEnable.value = True
        nm.ChunkModeActive.value = True

        nm.ExposureAuto.value = 'Continuous'

        nm.ExposureAutoFrameRateMin.value = 10.0

        nm.Scan3dDataFilterSelector.value = 'ValidationFilter'
        nm.Scan3dDataFilterEnable.value = True
        nm.Scan3dDepthValidationFilterLevel.value = 9

        if nm.GevSCPSPacketSize.value <= 1500:
            logging.warning("BEWARE: failed to negotiate larger packet size (>1500B), enable jumbo frame support on the network card for optimal performance")

        logging.info(f"GevSCPPacketSize is {nm.GevSCPSPacketSize.value}")
        ia.num_buffers = 1
        ia.start()
      
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model = torch.hub.load('ultralytics/yolov5', 'custom', path = "./yolov5m.pt") #学習モデルの読み込み(ここを差し替えてコーンを認識させる)
        
        """
        logging.info(f"Detection model is using {device}")
        if not torch.cuda.is_available():
            logging.warning("Application is NOT using a GPU!")
        """

        # in loop
        with ia.fetch(timeout=2.0) as buffer:
                cmp = next((cmp for cmp in buffer.payload.components if cmp.data_format == 'BGR8'))
                intens_img = cmp.data.reshape(cmp.height, cmp.width, 3)
                intens_img = cv2.cvtColor(intens_img, cv2.COLOR_BGR2RGB)

                depthcmp = next((cmp for cmp in buffer.payload.components if cmp.data_format == 'Coord3D_C16'))
                depth_arr_uint16 = depthcmp.data.reshape(depthcmp.height, depthcmp.width)

                original_img_height, original_img_width, _ = intens_img.shape # 縦576 横1024
                rescaled_img_height, rescaled_img_width = 640, 640
                hscale = original_img_height / rescaled_img_height
                wscale = original_img_width / rescaled_img_width

                hscale, wscale = 1.0, 1.0
                rescaled_max_size = max((rescaled_img_height, rescaled_img_width))

                results = model(intens_img, size=rescaled_max_size) # object detection
                clsnames = results.names
                results = results.pandas()

                for xyxy in results.xyxy:
                    boxes = xyxy[["xmin", "ymin", "xmax", "ymax"]].values
                    confs, classids = xyxy["confidence"], xyxy["class"]

                    for box, conf, classid in zip(boxes, confs, classids):
                        label = clsnames[classid]
                        xmin, ymin, xmax, ymax = box

                        xmin, xmax = wscale * xmin, wscale * xmax
                        ymin, ymax = hscale * ymin, hscale * ymax

                        xmin, xmax = int(xmin), int(xmax)
                        ymin, ymax = int(ymin), int(ymax)

                        depth_cropped_to_bbox = depth_arr_uint16[ymin: ymax, xmin:xmax] # depth (深度情報)
                        distance_estimate_mill = compute_bbox_distance_estimate(depth_cropped_to_bbox)
                        distance_estimate_mtrs = distance_estimate_mill / 1000

                        x_center, y_center = calc_cord.calc_x_center(xmin, xmax), calc_cord.calc_y_center(ymin, ymax)
                        theta, phi = calc_cord.xy2angle(x_center, y_center, is_radian = True)
                        x, y, z = calc_cord.calc_coordinates(distance_estimate_mtrs, theta, phi)


        # end of loop
