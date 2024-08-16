# Copyright (c) 2023 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

from cv2 import cvtColor, COLOR_BAYER_RGGB2RGB_EA
import numpy as np
from toml import loads
from harvesters.core import Harvester
from lib.gev_helper import apply_param, set_components
from os import path
import platform
from time import time
from logging import info, error

DEVICE_ACCESS_STATUS_READWRITE = 1 # GenICam/GenTL dfinition
FETCH_TIMEOUT = 3.0                # relaxed, better stability for bad network bandwidth


def data_map(maps, data_format):
    return next(filter(lambda x: x['data_format'] == data_format, maps))


def extract_depth(coord3d_data):
    width = coord3d_data['width']
    height = coord3d_data['height']
    col, row = np.meshgrid(np.arange(width), np.arange(height))
    depth = coord3d_data['data'].reshape(height, width)
    return col, row, depth


def extract_color(bgr8_data):
    width = bgr8_data['width']
    height = bgr8_data['height']
    bgr = bgr8_data['data'].reshape(height, width, 3)
    return bgr[..., ::-1].copy()


def extract_bayer(bayer_data):
    width = bayer_data['width']
    height = bayer_data['height']
    bayer = bayer_data['data'].reshape(height, width)
    return cvtColor(bayer, COLOR_BAYER_RGGB2RGB_EA)


def parse_config(config_file):
    with open(config_file, 'rb') as infile:
        file_bytes = infile.read()
        config = loads(file_bytes.decode('utf-8'))
        return config


def get_cti_path():
    """Helper to get cti file path corresponding with the platform the script is running on """
    # The directory with this script is located expected to contain "cti/" subdirectory further
    # containing one subdirectory for each platform supported by the SICK GenTL Producer for GigE Vision.
    # The actual filename of the GenTL Producer binary is always the same (SICKGigEVisionTL.cti).
    cti_platform_dir_name = get_cti_dir_name()
    script_dir = path.dirname(path.realpath(__file__))
    CTI_FILENAME = "SICKGigEVisionTL.cti"
    return path.join(script_dir, "cti", cti_platform_dir_name, CTI_FILENAME)


def get_cti_dir_name():
    """Helper to get cti file directory name corresponding with the platform the script is running on"""
    if platform.system() == "Windows":
        return "windows_x64"
    if platform.system() == "Linux" and platform.machine() == "x86_64":
        return "linux_x64"
    if platform.system() == "Linux" and platform.machine() == "aarch64":
        return "linux_aarch64"

    # Not one of our recognized platforms, cti file not available
    raise RuntimeError("GenTL Producer not available on this platform")


def init_harvester():
    try:
        h = Harvester()
        cti = get_cti_path()
        h.add_file(cti, check_existence=True, check_validity=True)
        info("CTI driver loaded...")
        h.update()
        info(f"Device discovery done, received {len(h.device_info_list)} answers")
        return h
    except Exception as err:
        error(f"No devices found: {err}")
        exit(1)


def setup_camera_object(harvester, device_idx):
    ia = harvester.create(device_idx)
    ia.num_buffers = 10
    ia.stop()
    device = harvester.device_info_list[device_idx]
    camera = {
        'name': f"{device.display_name}_{device.serial_number}",
        'ia': ia,
        'nm': ia.remote_device.node_map,
        'writer': None,  # init later after params config
        'frameCount': 0,
        'recordedCount': 0,
    }
    return camera


def select_device(device_list, config):
    for idx, device in enumerate(device_list):
        acc_stat = device.access_status
        if device.serial_number == config['cameras']['serial'][0]:
            if acc_stat == DEVICE_ACCESS_STATUS_READWRITE:
                info(f"Found camera: {device.display_name} ({device.serial_number})")
                return idx
    return None


def config_camera(camera, config):
    for name, val in config['gev_params'].items():
        apply_param(camera['nm'], name, val)
    set_components(camera['nm'], config['gev_config']['ComponentList'])
    info(f"Applied configuration for camera: {camera['name']}")

def doFetch(camera, config):
    with camera['ia'].fetch(timeout=FETCH_TIMEOUT) as buffer:
      camera['frameCount'] += 1
      if camera['frameCount'] % config['cameras']['recordingRate'] == 0:
          camera['recordedCount'] += 1
          camera['writer'].store(buffer, camera['nm'])

def maybe_capture_secs(cameras, config, t_start, duration):
    if not duration:
        return
    if len(cameras) == 1:
      info(f"Capture frames for {duration} seconds")
      cam = cameras[0]
      cam['ia'].start()
      while time() < (t_start + duration):
        doFetch(cam, config)
      cam['ia'].stop()
    else:
      # cameras list==0 handled outside/before this function
      info(f"Capture frames (round-robin for multiple cameras) for {duration} seconds")
      while time() < (t_start + duration):
          for cam in cameras:
              cam['ia'].start()
              doFetch(cam, config)
              cam['ia'].stop()


def maybe_capture_num_frames(cameras, config, num_frames):
    if not num_frames:
        return
    if len(cameras) == 1:
      info(f"Capture {num_frames} frames")
      cam = cameras[0]
      cam['ia'].start()
      while cameras[0]['recordedCount'] < num_frames:
        doFetch(cam, config)
      cam['ia'].stop()
    else:
      # cameras list==0 handled outside/before this function
      while cameras[0]['recordedCount'] < num_frames:
          for cam in cameras:
              cam['ia'].start()
              doFetch(cam, config)
              cam['ia'].stop()


def maybe_capture_auto_bracket(cameras, auto_bracket):
    if not auto_bracket:
        return
    info(f"Capture frames for: {auto_bracket.start}..{auto_bracket.stop-1} with steps of {auto_bracket.step} us")
    for exp_time in auto_bracket:
        for cam in cameras:
            apply_param(cam['nm'], 'ExposureTime', exp_time)
            cam['ia'].start()
            with cam['ia'].fetch(timeout=FETCH_TIMEOUT) as buffer:
                cam['frameCount'] += 1
                cam['recordedCount'] += 1
                cam['writer'].store(buffer, cam['nm'])
            cam['ia'].stop()

