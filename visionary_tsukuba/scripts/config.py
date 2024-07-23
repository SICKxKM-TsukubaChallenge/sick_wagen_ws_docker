class webapp_config:
    def __init__(self):

        # file settings
        #self.weights_file = './detect_model.pt'
        self.weights_file = './yolov5m.pt'
        self.is_save_log = True
        self.logfile_dir = "./detection_log" # activate if self.is_save_log is True

        # detection setting
        self.detect_thres = 0.5
        self.detect_class = ["person"]
        #self.detect_class = ["person", "corn"] # detected label is not in this list, object is not recorded
        self.is_manual_size_setting = False # if self.is_manual_size_setting is False, set imagesize 640×640
        self.input_image_size = 960 # activate if self.is_manual_size_setting is True

        # area setting
        # points must set list [x, y]. 
        self.areapoints = [(-0.9493701697317589, 1.695043540010472), (0.5910456859398118, 2.126948913025074), (-0.1586905578990529, 3.526114932051974)]


        # graphic settings
        self.text_color_detect = (255, 0, 0) # red
        self.text_color_nondetect = (255, 255, 255) # white

        # for developer use
        self.record_inconfident_image = True
        self.record_inconfident_path = "./inconfident_images"
        self.record_inconfident_class = ["corn"] #特定の物体のconfidenceが低いときだけ画像を保存

class calc_coordinates_config:
    def __init__(self):

        # calculation sensor coordinate settings
        self.horizontal_resolution = 1024
        self.vertical_resolution = 576
        self.horizontal_FOV = 120
        self.vertical_FOV = 105
